"""Edge core: protocol v1 server, telemetry, watchdog, battery monitor, perception input.

Process layout on the Pi (see docs/PRD.md, A4):
  edge core (this, highest priority)   owns the FC serial link and the station TCP link
  vision process (edge/vision, lower)  sends perception JSON over UDP to PERCEPTION_PORT
  flow process   (edge/flow, middle)   sends flow samples over UDP to FLOW_PORT
Killing vision or flow never stops the core loop, so the drone keeps holding/landing.
"""
from __future__ import annotations

import asyncio
import json
import logging
import time
from pathlib import Path

from shared import protocol as P

from .config import SafetyConfig, WatchdogConfig
from .executor import Executor
from .safety import SafetyGate
from .vehicle import Vehicle

log = logging.getLogger("edge")

PERCEPTION_PORT = 7001
FLOW_PORT = 7002
PERCEPTION_STALE_S = 1.0


class FlightLog:
    def __init__(self, path: Path | None):
        self.f = open(path, "a", buffering=1) if path else None

    def __call__(self, rec: dict) -> None:
        if self.f:
            self.f.write(json.dumps(rec) + "\n")


class EdgeServer:
    def __init__(self, vehicle: Vehicle, safety: SafetyConfig | None = None,
                 watchdog: WatchdogConfig | None = None, host: str = "0.0.0.0",
                 port: int = P.EDGE_PORT, perception_port: int | None = PERCEPTION_PORT,
                 flow_port: int | None = FLOW_PORT, log_path: Path | None = None):
        self.v = vehicle
        self.safety_cfg = safety or SafetyConfig()
        self.wd = watchdog or WatchdogConfig()
        self.host, self.port = host, port
        self.perception_port, self.flow_port = perception_port, flow_port
        self.flog = FlightLog(log_path)
        self.exec = Executor(vehicle, SafetyGate(self.safety_cfg), self._emit, log=self.flog)
        self._writer: asyncio.StreamWriter | None = None
        self._last_hb: float | None = None
        self._perception_t = 0.0
        self._palm_latched = False
        self._server: asyncio.base_events.Server | None = None
        self._tasks: list[asyncio.Task] = []
        self.loop_stats: list[float] = []  # control-loop period samples (s), for the jitter graph

    # --- lifecycle ---------------------------------------------------------------------------
    async def start(self) -> None:
        await self.v.start()
        self._server = await asyncio.start_server(self._on_client, self.host, self.port)
        self.port = self._server.sockets[0].getsockname()[1]
        loop = asyncio.get_running_loop()
        if self.perception_port is not None:
            await loop.create_datagram_endpoint(
                lambda: _Udp(self._on_perception), local_addr=("127.0.0.1", self.perception_port))
        if self.flow_port is not None:
            await loop.create_datagram_endpoint(
                lambda: _Udp(self._on_flow), local_addr=("127.0.0.1", self.flow_port))
        self._tasks = [asyncio.create_task(self._telemetry_loop()),
                       asyncio.create_task(self._safety_loop())]
        log.info("edge listening on %s:%d", self.host, self.port)

    async def close(self) -> None:
        for t in self._tasks:
            t.cancel()
        if self._server:
            self._server.close()
        await self.v.close()

    # --- station link ------------------------------------------------------------------------
    def _emit(self, msg: dict) -> None:
        w = self._writer
        if w is None or w.is_closing():
            return
        try:
            w.write(P.encode(msg))
        except Exception:  # station gone; watchdog handles it
            pass

    async def _on_client(self, reader: asyncio.StreamReader, writer: asyncio.StreamWriter):
        peer = writer.get_extra_info("peername")
        if self._writer and not self._writer.is_closing():
            log.warning("new station %s replaces the old one", peer)
            self._writer.close()
        self._writer = writer
        log.info("station connected: %s", peer)
        try:
            while line := await reader.readline():
                msg: dict = {}
                try:
                    msg = P.decode(line)
                    self._on_message(msg)
                except P.ProtocolError as e:
                    self._emit(P.ack(str(msg.get("id", "")), False, f"protocol error: {e}"))
        except (ConnectionError, asyncio.IncompleteReadError):
            pass
        finally:
            if self._writer is writer:
                self._writer = None
            writer.close()
            log.info("station disconnected: %s", peer)

    def _on_message(self, msg: dict) -> None:
        t = msg["type"]
        if t == P.HEARTBEAT:
            self._last_hb = time.monotonic()
            if self.exec.failsafe == "hover":
                self.exec.clear_hover_failsafe()
                self._emit(P.event("watchdog", "station link restored; holding position"))
        elif t == P.CMD:
            cid, op, args = P.validate_cmd(msg)
            self._last_hb = time.monotonic()  # a command is proof of life too
            self.exec.handle(cid, op, args)
        else:
            raise P.ProtocolError(f"unknown message type {t!r}")

    # --- loops -------------------------------------------------------------------------------
    async def _telemetry_loop(self) -> None:
        period = 1.0 / P.TELEMETRY_HZ
        while True:
            self._emit(self.telemetry())
            await asyncio.sleep(period)

    def telemetry(self) -> dict:
        s = self.v.state()
        return {"type": P.TELEMETRY, "t": time.time(), "state": self.exec.state,
                "armed": s.armed, "mode": s.mode, "n": round(s.n, 3), "e": round(s.e, 3),
                "alt": round(s.alt, 3), "yaw": round(s.yaw, 1), "vn": round(s.vn, 3),
                "ve": round(s.ve, 3), "vd": round(s.vd, 3), "battery_v": round(s.battery_v, 2),
                "flow_quality": s.flow_quality,
                "range_m": None if s.range_m is None else round(s.range_m, 3),
                "active_cmd": self.exec.active_cmd}

    async def _safety_loop(self, hz: float = 50.0) -> None:
        period = 1.0 / hz
        last = time.monotonic()
        while True:
            await asyncio.sleep(period)
            now = time.monotonic()
            self.loop_stats.append(now - last)
            if len(self.loop_stats) > 5000:
                del self.loop_stats[:1000]
            last = now
            self.safety_tick(now)

    def safety_tick(self, now: float) -> None:
        ex, s = self.exec, self.v.state()
        flying = ex.airborne
        # watchdog
        if flying and self._last_hb is not None:
            gap = now - self._last_hb
            if gap > self.wd.land_after_s:
                ex.trigger_failsafe("land", "watchdog",
                                    f"no station heartbeat for {gap:.1f} s: landing")
            elif gap > self.wd.hover_after_s and ex.failsafe is None:
                ex.trigger_failsafe("hover", "watchdog",
                                    f"no station heartbeat for {gap:.1f} s: holding")
        elif flying and self._last_hb is None:
            # flying with no station ever connected (e.g. RC takeoff): start the clock now
            self._last_hb = now
        # battery
        if flying and 0 < s.battery_v < self.safety_cfg.batt_land_v:
            ex.trigger_failsafe("land", "battery", f"battery {s.battery_v:.2f} V: landing")
        # stale perception
        if ex.perception.person_m is not None and now - self._perception_t > PERCEPTION_STALE_S:
            ex.perception.person_m = None

    # --- inputs from the AI / flow processes -------------------------------------------------
    def on_perception(self, msg: dict) -> None:
        """{"person_m": float|null, "palm": bool}"""
        self._perception_t = time.monotonic()
        ex = self.exec
        pm = msg.get("person_m")
        ex.perception.person_m = float(pm) if isinstance(pm, (int, float)) else None
        if (ex.perception.person_m is not None
                and ex.perception.person_m < self.safety_cfg.person_close_m
                and ex.state == "BUSY"):
            ex.stop(f"person {ex.perception.person_m:.1f} m ahead")
            self._emit(P.event("person_close", f"person {ex.perception.person_m:.1f} m ahead: "
                                               "stopped"))
        palm = bool(msg.get("palm"))
        if palm and not self._palm_latched and ex.airborne:
            ex.stop("palm stop")
            self._emit(P.event("palm_stop", "palm seen: stopped and holding"))
        self._palm_latched = palm

    def _on_perception(self, data: bytes) -> None:
        try:
            self.on_perception(json.loads(data))
        except (ValueError, TypeError):
            pass

    def _on_flow(self, data: bytes) -> None:
        try:
            m = json.loads(data)
            self.v.send_optical_flow(float(m["fx"]), float(m["fy"]), int(m["q"]),
                                     int(m["dt_us"]), m.get("ground_m"))
        except (ValueError, TypeError, KeyError):
            pass


class _Udp(asyncio.DatagramProtocol):
    def __init__(self, cb):
        self.cb = cb

    def datagram_received(self, data, addr):
        self.cb(data)
