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
import queue
import socket
import threading
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
OBSTACLE_STALE_S = 0.5   # the forward HC-SR04 reports at ~20 Hz


class FlightLog:
    """JSONL flight log. Writes happen on a background thread: an SD-card write that stalls
    the event loop would stop the hbwd pets (250 ms) and land the drone (review3 #8)."""

    def __init__(self, path: Path | None):
        self.f = open(path, "a", buffering=1) if path else None
        self._q: queue.SimpleQueue | None = None
        if self.f:
            self._q = queue.SimpleQueue()
            threading.Thread(target=self._writer, daemon=True, name="flight-log").start()

    def __call__(self, rec: dict) -> None:
        if self._q is not None:
            self._q.put(json.dumps(rec) + "\n")

    def _writer(self) -> None:
        while True:
            line = self._q.get()
            try:
                self.f.write(line)
            except (OSError, ValueError):
                pass  # disk full / closed: never take the flight loop down for a log line


class EdgeServer:
    def __init__(self, vehicle: Vehicle, safety: SafetyConfig | None = None,
                 watchdog: WatchdogConfig | None = None, host: str = "0.0.0.0",
                 port: int = P.EDGE_PORT, perception_port: int | None = PERCEPTION_PORT,
                 flow_port: int | None = FLOW_PORT, log_path: Path | None = None,
                 allowed_peers: set[str] | None = None,
                 hbwd_addr: tuple[str, int] | None = None,
                 video_addr: tuple[str, int] | None = None,
                 stats_addr: tuple[str, int] | None = None):
        self.v = vehicle
        self.safety_cfg = safety or SafetyConfig()
        self.wd = watchdog or WatchdogConfig()
        self.host, self.port = host, port
        self.perception_port, self.flow_port = perception_port, flow_port
        self.flog = FlightLog(log_path)
        self.allowed_peers = allowed_peers  # review A9: None = anyone (dev); set = only these
        self.exec = Executor(vehicle, SafetyGate(self.safety_cfg), self._emit, log=self.flog)
        self._writer: asyncio.StreamWriter | None = None
        self._last_hb: float | None = None
        self._perception_t = 0.0
        self._obstacle_t = 0.0
        self._palm_latched = False
        self._pos_bad_since: float | None = None
        self._batt_low_since: float | None = None
        self._server: asyncio.base_events.Server | None = None
        self._tasks: list[asyncio.Task] = []
        self.loop_stats: list[float] = []  # control-loop period samples (s), for the jitter graph
        self._tick_fail_since: float | None = None
        self._tick_fail_landed = False
        # edge/hbwd (C, SCHED_FIFO) is petted every safety tick; if the pets stop it drops the FC
        # heartbeat and commands LAND (only when armed in our modes)
        self.hbwd_addr = hbwd_addr
        self._hbwd_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) if hbwd_addr else None
        self._hbwd_seq = 0
        # separate video process (python -m edge.video): stream the pose to it every tick
        self.video_addr = video_addr
        self._video_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) if video_addr else None
        # opt-in QNX demo stats (dashboard): 1 Hz loop jitter + person/palm stop latency as JSON
        # datagrams to the station (python -m station listens on :7010). Never on the flight path.
        self.stats_addr = stats_addr
        self._stats_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) if stats_addr else None
        self._stats_from = 0  # index into loop_stats of the next unreported sample

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
        if self._stats_sock:
            self._tasks.append(asyncio.create_task(self._stats_loop()))
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
        if self.allowed_peers is not None and peer[0] not in self.allowed_peers:
            log.warning("refusing station %s (not in --station-ip)", peer)
            self.flog({"t": time.time(), "kind": "refused_peer", "peer": str(peer)})
            writer.close()
            return
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
        deadline = last + period
        while True:
            # absolute deadlines: sleeping a fixed period *after* each tick ran the loop below
            # 50 Hz and counted the tick's own work as scheduler lateness in the jitter graph
            await asyncio.sleep(max(0.0, deadline - time.monotonic()))
            now = time.monotonic()
            deadline += period
            if now - deadline > period:  # fell more than a tick behind: don't burst to catch up
                deadline = now + period
            self.loop_stats.append(now - last)
            if len(self.loop_stats) > 5000:
                del self.loop_stats[:1000]
                self._stats_from = max(0, self._stats_from - 1000)
            last = now
            # One exception must not end this loop: it carries the watchdog, the battery and
            # position failsafes and the FC heartbeat (review3 #6). The hbwd pet is sent only
            # at the end of a complete tick, so a tick that keeps failing stops the pets and the
            # C watchdog lands the drone; without hbwd, we try to land after 1 s of failures.
            try:
                self.safety_tick(now)
                self._tick_fail_since = None
                self._tick_fail_landed = False  # recovered: a later failure streak lands again
            except Exception:
                if self._tick_fail_since is None:
                    self._tick_fail_since = now
                    log.exception("safety tick failed")
                elif now - self._tick_fail_since > 1.0 and not self._tick_fail_landed:
                    try:
                        mode = self.v.state().mode
                    except Exception:
                        mode = "UNKNOWN"
                    if mode not in ("GUIDED", "LAND", "UNKNOWN"):
                        # pilot has AltHold/Stabilize: never take it away (safety review); if
                        # they hand back to GUIDED while it's still failing, the next tick lands
                        continue
                    self._tick_fail_landed = True
                    log.error("safety tick failing for 1 s: landing")
                    try:
                        self.exec.trigger_failsafe("land", "watchdog",
                                                   "edge safety loop is failing: landing")
                    except Exception:
                        log.exception("failsafe land failed")

    def safety_tick(self, now: float) -> None:
        self.v.tick(now)  # FC heartbeat + LAND retries live in this loop (review A3, A2)
        ex, s = self.exec, self.v.state()
        self._safety_checks(now, ex, s)
        if self._hbwd_sock:  # last: only a tick that ran every check pets the C watchdog
            self._hbwd_seq += 1
            ours = int(s.mode in ("GUIDED", "LAND"))
            try:
                self._hbwd_sock.sendto(f"PET {self._hbwd_seq} {int(s.armed)} {ours}\n".encode(),
                                       self.hbwd_addr)
            except OSError:
                pass

    def _safety_checks(self, now: float, ex: Executor, s) -> None:
        if self._video_sock:
            try:
                self._video_sock.sendto(json.dumps(s.as_dict()).encode(), self.video_addr)
            except OSError:
                pass
        # pilot took another mode in flight -> stand down, forget targets (review2 #5)
        in_air = s.armed and not s.landed
        ours = s.mode == "GUIDED" or (s.mode == "LAND" and ex.state in ("LANDING", "FAILSAFE"))
        if in_air and not ours and s.mode != "UNKNOWN":
            if ex.pilot_mode != s.mode:
                ex.on_pilot_control(s.mode)
        elif ex.pilot_mode is not None and (s.mode == "GUIDED" or not s.armed):
            ex.pilot_mode = None
        # re-check the running move (review2 #4, #6)
        if (why := ex.revalidate(s, self.safety_cfg)) is not None:
            ex.stop(why)
            kind = "person_close" if "person" in why else "safety_reject"
            self._emit(P.event(kind, f"{why}: stopped and holding"))
        # Only act while we're in control. If the pilot switched to AltHold/Land/Stabilize,
        # the edge must not override them; ArduPilot's own failsafes still apply. A LAND the
        # pilot chose is the pilot's (it used to count as ours and fired a stale watchdog).
        flying = ex.airborne and ours
        if not s.armed:
            # a station gap from before this flight must never fire mid-flight (safety review:
            # "no station heartbeat for 722.6 s" right as the pilot flipped to Land)
            self._last_hb = None
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
        # position estimate lost in flight -> land (review A6)
        if flying and not s.pos_ok:
            if self._pos_bad_since is None:
                self._pos_bad_since = now
            elif now - self._pos_bad_since > 1.0:
                diag = getattr(self.v, "pos_diag", None)
                ex.trigger_failsafe("land", "position", "position estimate lost: landing"
                                    + (f" ({diag()})" if diag else ""))
        else:
            self._pos_bad_since = None
        # battery: 2 s below the threshold, so throttle sag doesn't end a flight (review2 #12)
        if flying and 0 < s.battery_v < self.safety_cfg.batt_land_v:
            if self._batt_low_since is None:
                self._batt_low_since = now
            elif now - self._batt_low_since > 2.0:
                ex.trigger_failsafe("land", "battery", f"battery {s.battery_v:.2f} V: landing")
        else:
            self._batt_low_since = None
        # stale perception: forget the person and mark the detector dead
        if now - self._perception_t > PERCEPTION_STALE_S:
            ex.perception.person_m = None
            ex.perception.detector_alive = False
        if now - self._obstacle_t > OBSTACLE_STALE_S:
            ex.perception.obstacle_m = None
            ex.perception.obstacle_alive = False

    # --- QNX demo stats (opt-in) ------------------------------------------------------------
    def _send_stats(self, msg: dict) -> None:
        if not self._stats_sock:
            return
        try:
            self._stats_sock.sendto(json.dumps({"src": "edge", "t": time.time(), **msg}).encode(),
                                    self.stats_addr)
        except OSError:
            pass

    def loop_summary(self, hz: float = 50.0) -> dict | None:
        """Lateness (ms) of the safety loop's wake-ups since the last call."""
        new = self.loop_stats[self._stats_from:]
        self._stats_from = len(self.loop_stats)
        if not new:
            return None
        late = sorted(max(0.0, (dt - 1.0 / hz) * 1000.0) for dt in new)
        pct = lambda p: round(late[min(len(late) - 1, int(p / 100 * len(late)))], 2)
        return {"kind": "loop", "hz": hz, "n": len(late), "p50_ms": pct(50), "p99_ms": pct(99),
                "max_ms": round(late[-1], 2)}

    async def _stats_loop(self) -> None:
        while True:
            await asyncio.sleep(1.0)
            if (m := self.loop_summary()) is not None:
                self._send_stats(m)

    def _stop_latency(self, what: str, msg: dict, t_rx: float) -> None:
        """Detector frame time ("t", wall clock on the Pi) -> stop sent. Without "t" we can only
        report the edge's own reaction (UDP in -> stop), which is the floor, not the story."""
        now = time.time()
        rec = {"kind": "stop_latency", "what": what, "edge_ms": round((now - t_rx) * 1000, 2)}
        if isinstance(msg.get("t"), (int, float)):
            rec["ms"] = round((now - float(msg["t"])) * 1000, 1)
        if isinstance(msg.get("infer_ms"), (int, float)):
            rec["infer_ms"] = round(float(msg["infer_ms"]), 1)
        self._send_stats(rec)

    # --- inputs from the AI / flow processes -------------------------------------------------
    def on_perception(self, msg: dict) -> None:
        """{"person_m": float|null, "palm": bool, optional "t" (frame time, time.time()),
        optional "infer_ms"}"""
        t_rx = time.time()
        self._perception_t = time.monotonic()
        ex = self.exec
        ex.perception.detector_alive = True
        ex.perception.person_t = self._perception_t
        pm = msg.get("person_m")
        ex.perception.person_m = float(pm) if isinstance(pm, (int, float)) else None
        # react now rather than on the next tick: every ms counts for the stop latency demo
        if (why := ex.revalidate(self.v.state(), self.safety_cfg)) is not None:
            ex.stop(why)
            self._emit(P.event("person_close", f"{why}: stopped and holding"))
            self._stop_latency("person", msg, t_rx)
        palm = bool(msg.get("palm"))
        if palm and not self._palm_latched and ex.airborne:
            ex.stop("palm stop")
            self._emit(P.event("palm_stop", "palm seen: stopped and holding"))
            self._stop_latency("palm", msg, t_rx)
        self._palm_latched = palm

    def on_obstacle(self, fwd_m: float) -> None:
        """Forward ultrasonic reading in metres (< 0 = no echo within range = clear)."""
        t_rx = time.time()
        self._obstacle_t = time.monotonic()
        ex = self.exec
        ex.perception.obstacle_alive = True
        ex.perception.obstacle_t = self._obstacle_t
        ex.perception.obstacle_m = None if fwd_m < 0 else fwd_m
        if (why := ex.revalidate(self.v.state(), self.safety_cfg)) is not None:
            ex.stop(why)
            self._emit(P.event("safety_reject", f"{why}: stopped and holding"))
            self._stop_latency("obstacle", {}, t_rx)

    def _on_perception(self, data: bytes) -> None:
        try:
            self.on_perception(json.loads(data))
        except (ValueError, TypeError):
            pass

    def _on_flow(self, data: bytes) -> None:
        try:
            m = json.loads(data)
            if "fwd_m" in m:  # tools/rangefinder.py: the forward HC-SR04 (obstacles)
                self.on_obstacle(float(m["fwd_m"]))
                return
            if "range_m" in m:  # tools/rangefinder.py (HC-SR04 via Arduino): height above floor
                if not m.get("direct"):  # direct: it already went to the FC (edge.fcdirect)
                    self.v.send_distance_sensor(float(m["range_m"]))
                return
            if m.get("direct"):  # the flow process sent OPTICAL_FLOW to the FC itself (review3
                self.v.note_flow_quality(int(m["q"]))  # #8); keep its quality for the gate
                return
            self.v.send_optical_flow(float(m["fx"]), float(m["fy"]), int(m["q"]),
                                     int(m["dt_us"]), m.get("ground_m"))
        except (ValueError, TypeError, KeyError):
            pass


class _Udp(asyncio.DatagramProtocol):
    def __init__(self, cb):
        self.cb = cb

    def datagram_received(self, data, addr):
        self.cb(data)
