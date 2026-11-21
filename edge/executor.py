"""Runs one primitive at a time. A new accepted command cancels the running one; stop always wins."""
from __future__ import annotations

import asyncio
import math
import time
from typing import Callable

from shared import protocol as P

from .config import ExecConfig
from .safety import Perception, SafetyGate
from .vehicle import Vehicle, ned_to_body, wrap360, yaw_err

Emit = Callable[[dict], None]


class Executor:
    TICK = 0.05
    FLOW_BAD_S = 0.5  # flow quality below the minimum this long stops a running move

    def __init__(self, vehicle: Vehicle, gate: SafetyGate, emit: Emit,
                 cfg: ExecConfig | None = None, log: Callable[[dict], None] | None = None):
        self.v = vehicle
        self.gate = gate
        self.emit = emit
        self.cfg = cfg or ExecConfig()
        self.log = log or (lambda rec: None)
        self.perception = Perception()
        self._task: asyncio.Task | None = None
        self._active: tuple[str, str] | None = None   # (id, op)
        self._abort_reason = ""
        self._phase = ""          # TAKING_OFF / LANDING while those run
        self.failsafe: str | None = None  # None | "hover" | "land"
        self.target: tuple[float, float, float, float] | None = None  # last commanded
        # set at the FIRST takeoff of this edge process and kept (review A12, review3 #2): the
        # fence is the room we cleared, not wherever the drone last landed
        self.home: tuple[float, float] | None = None
        self._active_fwd = False   # running command moves the drone forward (person checks)
        self._active_moves = False # running command changes position (flow checks)
        self.pilot_mode: str | None = None  # set while the pilot has another mode in flight
        self._flow_bad_since: float | None = None

    def _set(self, n: float, e: float, alt: float, yaw: float) -> None:
        self.target = (n, e, alt, wrap360(yaw))
        self.v.set_target(n, e, alt, yaw)

    def _hold(self) -> None:
        s = self.v.state()
        self.target = (s.n, s.e, s.alt, s.yaw)
        self.v.hold()

    # --- state -------------------------------------------------------------------------------
    @property
    def airborne(self) -> bool:
        s = self.v.state()
        return s.armed and not s.landed and self._phase != "LANDING"

    @property
    def state(self) -> str:
        s = self.v.state()
        if not s.connected:
            return "DISCONNECTED"
        if self.failsafe:
            return "FAILSAFE"
        if self._phase:
            return self._phase
        if self._task and not self._task.done():
            return "BUSY"
        return "HOVER" if s.armed and not s.landed else "IDLE"

    @property
    def active_cmd(self) -> str | None:
        return self._active[0] if self._active else None

    # --- command entry -----------------------------------------------------------------------
    def handle(self, cid: str, op: str, args: dict) -> bool:
        s = self.v.state()
        if (op in ("stop", "hover", "land") and s.armed and not s.landed
                and s.mode not in ("GUIDED", "LAND", "UNKNOWN")):
            # the pilot has flipped SwA (AltHold etc.): the pilot always wins (review3 #7). A
            # LAND from the station or a dashboard would take the drone back from them.
            reason = f"pilot has control (mode {s.mode}); not overriding the pilot"
            self.log({"t": time.time(), "kind": "cmd", "id": cid, "op": op, "args": args,
                      "ok": False, "reason": reason})
            self.emit(P.ack(cid, False, reason))
            self.emit(P.event("safety_reject", f"{op}: {reason}"))
            return False
        if self._phase == "LANDING" and op in ("stop", "hover", "land"):
            # landing beats stop: never pull a landing drone back into a hover
            self.emit(P.ack(cid, True, "landing continues"))
            self.emit(P.done(cid, P.OK, "landing continues"))
            return True
        if self.failsafe == "land" and op != "land":
            verdict_ok, reason = False, "failsafe landing in progress"
            verdict = None
        else:
            airborne = self.airborne
            if not airborne:
                self.target = None
            verdict = self.gate.check(op, args, s, airborne, self.perception,
                                      base=self.target, home=self.home)
            verdict_ok, reason = verdict.ok, verdict.reason
        self.log({"t": time.time(), "kind": "cmd", "id": cid, "op": op, "args": args,
                  "ok": verdict_ok, "reason": reason,
                  "pos": [round(s.n, 2), round(s.e, 2), round(s.alt, 2), round(s.yaw, 1)]})
        self.emit(P.ack(cid, verdict_ok, reason))
        if not verdict_ok:
            self.emit(P.event("safety_reject", f"{op}: {reason}"))
            return False
        if self.failsafe == "hover":
            self.failsafe = None  # a fresh accepted command from a live station clears it
        self._cancel(f"superseded by {op}")
        if op == "takeoff" and self.home is None:
            self.home = (s.n, s.e)
        self._active_moves = op in ("move_body", "goto", "return_home")
        self._active_fwd = False
        if self._active_moves and verdict.target:
            fwd, _ = ned_to_body(verdict.target[0] - s.n, verdict.target[1] - s.e, s.yaw)
            self._active_fwd = fwd > 0.2
        self._active = (cid, op)
        self._task = asyncio.create_task(self._run(cid, op, args, verdict))
        return True

    def revalidate(self, s, cfg) -> str | None:
        """Re-check the *running* command every safety tick (review2 #4, #6). Returns a reason
        to stop it, or None. Forward-only checks let the drone back away from a person."""
        if not (self._task and not self._task.done() and self._active_moves):
            return None
        p = self.perception
        if self._active_fwd:
            if p.person_m is not None and p.person_m < cfg.person_close_m:
                return f"person {p.person_m:.1f} m ahead"
            if cfg.require_person_detector and not p.detector_alive:
                return "person detector stopped"
        # one blurry frame shouldn't abort a move (and the whole plan): flow must stay bad for
        # FLOW_BAD_S before the move is stopped (review3, smaller items)
        if s.flow_quality < cfg.flow_quality_min:
            now = time.monotonic()
            if self._flow_bad_since is None:
                self._flow_bad_since = now
            elif now - self._flow_bad_since >= self.FLOW_BAD_S:
                return f"optical flow quality {s.flow_quality} too low"
        else:
            self._flow_bad_since = None
        return None

    def on_pilot_control(self, mode: str) -> None:
        """The pilot switched the FC out of our modes in flight: get out of the way and forget
        our targets, so the next relative move starts from where the pilot left it (review2 #5).
        """
        self.pilot_mode = mode
        self._cancel(f"pilot took control (mode {mode})")
        self.target = None
        self._phase = ""
        self.failsafe = None
        self.log({"t": time.time(), "kind": "pilot_control", "mode": mode})

    def stop(self, reason: str) -> None:
        """Internal stop (palm, person, watchdog): cancel and hold."""
        self._cancel(reason)
        if self.airborne:
            self._hold()

    def trigger_failsafe(self, action: str, kind: str, detail: str) -> None:
        if self.failsafe == "land" or (self.failsafe == action):
            return
        self.failsafe = action
        self.emit(P.event(kind, detail))
        self.log({"t": time.time(), "kind": "failsafe", "action": action, "detail": detail})
        self._cancel(f"failsafe: {detail}")
        if action == "land":
            if self.v.state().armed:
                self._phase = "LANDING"
                self.v.land()
                self._task = asyncio.create_task(self._watch_landing())
        elif self.airborne:
            self._hold()

    def clear_hover_failsafe(self) -> None:
        if self.failsafe == "hover":
            self.failsafe = None

    # --- internals ---------------------------------------------------------------------------
    def _cancel(self, reason: str) -> None:
        if self._task and not self._task.done():
            self._abort_reason = reason
            self._task.cancel()
        if self._phase == "TAKING_OFF":
            self.v.cancel_takeoff()  # review A1: don't let a background arm/takeoff continue
            self._phase = ""

    async def _watch_landing(self) -> None:
        while self.v.state().armed:
            await asyncio.sleep(self.TICK)
        self._phase = ""
        if self.failsafe == "land":
            self.failsafe = None

    async def _run(self, cid: str, op: str, args: dict, verdict) -> None:
        try:
            status, reason = await self._do(op, args, verdict)
        except asyncio.CancelledError:
            status, reason = P.ABORTED, self._abort_reason or "cancelled"
            if self._phase == "TAKING_OFF":
                self._phase = ""
        except Exception as e:  # never let a bug leave the drone without a hold
            status, reason = P.FAILED, f"internal error: {e!r}"
            if self.airborne:
                self._hold()
        finally:
            if self._active and self._active[0] == cid:
                self._active = None
        self.log({"t": time.time(), "kind": "done", "id": cid, "status": status,
                  "reason": reason})
        self.emit(P.done(cid, status, reason))

    async def _do(self, op: str, args: dict, verdict) -> tuple[str, str]:
        c = self.cfg
        sc = self.gate.cfg
        if op == "stop" or op == "hover":
            s = self.v.state()
            if s.armed and not self.airborne:
                # armed but still on the ground (a takeoff just armed it): "stop" must not let
                # it climb. Abort and disarm via LAND (review2 #3)
                self.v.cancel_takeoff()
                self._phase = "LANDING"
                self.v.land()
                ok = await self._wait(lambda s: not s.armed, timeout=15.0)
                self._phase = ""
                if not ok:
                    return P.FAILED, "still armed on the ground; pilot: disarm"
                return P.OK, "takeoff stopped; disarmed on the ground"
            if self.airborne:
                self._hold()
            return P.OK, ""
        if op == "land":
            self.v.cancel_takeoff()  # review A1: land before arming finished must stop it
            if not self.v.state().armed:
                return P.OK, "already landed"
            self._phase = "LANDING"
            self.v.land()
            ok = await self._wait(lambda s: not s.armed, timeout=30.0)
            self._phase = ""
            if not ok:  # review A2: never report a landing that didn't happen
                return P.FAILED, "still armed 30 s after LAND; pilot: take over"
            return P.OK, ""
        if op == "takeoff":
            alt = verdict.target[2]
            self._phase = "TAKING_OFF"
            self.v.takeoff(alt)
            tol = self.move_tolerance(alt)  # 0.8 m takeoff: done at 0.56, not 0.5 (flow SITL)
            ok = await self._wait(lambda s: s.alt >= alt - tol,
                                  timeout=alt / sc.speed_z + c.cmd_timeout_pad_s)
            self._phase = ""
            if not ok:
                why = getattr(self.v, "last_error", "")
                s = self.v.state()
                if s.armed and s.alt < 0.3:
                    # never leave it armed on the ground while telemetry says IDLE (review2 #11)
                    self.v.cancel_takeoff()
                    self._phase = "LANDING"
                    self.v.land()
                    await self._wait(lambda s: not s.armed, timeout=15.0)
                    self._phase = ""
                    why = (why + "; " if why else "") + "disarmed"
                else:
                    self._hold()
                return P.FAILED, "takeoff timed out" + (f" ({why})" if why else "")
            s = self.v.state()
            self._set(s.n, s.e, alt, s.yaw)
            return P.OK, ""
        if op == "scan":
            rate = verdict.extra["rate_dps"]
            start = self.v.state().yaw
            n, e, alt, _ = verdict.target
            t0 = time.monotonic()
            while True:
                swept = rate * (time.monotonic() - t0)
                if swept >= 360.0:
                    break
                self.v.set_target(n, e, alt, wrap360(start + swept + rate * 0.5))
                self.target = (n, e, alt, start)  # a cancelled scan returns to its start yaw
                await asyncio.sleep(self.TICK)
            self._set(n, e, alt, start)
            return P.OK, ""
        # positional / yaw moves
        n, e, alt, yaw = verdict.target
        s = self.v.state()
        dist = math.dist((n, e, alt), (s.n, s.e, s.alt))
        timeout = (dist / min(sc.speed_xy, sc.speed_z) + abs(yaw_err(yaw, s.yaw)) / sc.yaw_rate
                   + c.cmd_timeout_pad_s)
        tol = self.move_tolerance(dist)
        self._set(n, e, alt, yaw)
        ok = await self._wait(
            lambda s: math.dist((n, e, alt), (s.n, s.e, s.alt)) <= tol
            and abs(yaw_err(yaw, s.yaw)) <= c.yaw_tolerance,
            timeout=timeout)
        if not ok:
            self._hold()
            return P.FAILED, "did not reach target in time"
        return P.OK, ""

    def move_tolerance(self, dist: float) -> float:
        c = self.cfg
        return min(c.pos_tolerance, max(c.pos_tolerance_min, c.pos_tolerance_frac * dist))

    async def _wait(self, pred, timeout: float) -> bool:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if pred(self.v.state()):
                return True
            await asyncio.sleep(self.TICK)
        return False
