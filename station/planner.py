"""Planner: brain actions -> protocol v1 primitives, one plan at a time.

    planner = Planner(edge, camera, locate=..., ai=..., memory=..., say=...)
    planner.submit(result.actions)     # cancels whatever plan is running (D7)

- `stop` and `land` pre-empt: the running plan is cancelled and the primitive goes out at once.
- A safety-gate rejection or a failed primitive ends the plan and Jarvis says why.
- mission: look for the target in the current frame, then turn in `scan_step` steps until
  `locate()` finds it (one full turn at most), fly to `approach_m` short of it (clamped inside
  the fence), face it, do the task (photo / read / solve / describe), then stay, come back or
  land.
- recall: search visual memory, say where and when, optionally fly to that pose and face the
  way the camera was facing.

The planner is never trusted either: the edge gates every primitive it sends.
"""
from __future__ import annotations

import asyncio
import contextlib
import contextvars
import logging
import math
import re
import time
from dataclasses import dataclass
from typing import Awaitable, Callable

from edge.config import ExecConfig, SafetyConfig, flight_time
from shared.client import CommandRejected, EdgeClient

from . import vision
from .camera import EdgeCamera, Frame

log = logging.getLogger("planner")


def _quiet():
    """Best effort: a landing or cleanup call must never raise over the real error."""
    return contextlib.suppress(Exception)

Say = Callable[[str], Awaitable[None]]
Locate = Callable[..., Awaitable[dict | None]]
MemorySearch = Callable[[str], Awaitable[list[dict]]]
OnPhoto = Callable[[Frame, str], Awaitable[None]]


POINTING = re.compile(r"point")
# plans starting with one of these send a flight step first: the reply waits for its verdict
DEFER_REPLY = ("takeoff", "move", "home", "stop", "land", "compass", "height", "pattern",
               "retrace")
SPIN_DPS = 30.0  # deg/s for a full turn: what tools/auto_hover --test spin flew
# the running plan's {"start": (n, e, h), "moved": bool}: per asyncio task, because a stop or
# land plan runs beside the plan it cancels
_PLAN: contextvars.ContextVar[dict | None] = contextvars.ContextVar("plan", default=None)
MOVE_OPS = ("move_body", "goto", "return_home")


def pattern_points(shape: str, size: float, side: str) -> list[tuple[float, float]]:
    """The corners of a pattern as (fwd, right) offsets from the start, in the order flown;
    the last one is the start again. size = side, circle diameter, or out-and-back distance."""
    sg = -1.0 if side == "left" else 1.0
    s = size
    if shape == "square":
        return [(s, 0.0), (s, sg * s), (0.0, sg * s), (0.0, 0.0)]
    if shape == "triangle":
        return [(s, 0.0), (s / 2, sg * s * math.sqrt(3) / 2), (0.0, 0.0)]
    if shape == "circle":  # an octagon through the start, centred size/2 to the side
        r, n = s / 2, 8
        return [(round(r * math.sin(2 * math.pi * k / n), 3),
                 round(sg * r * (1 - math.cos(2 * math.pi * k / n)), 3)) for k in range(1, n)] + [
                (0.0, 0.0)]
    if shape == "out_and_back":
        return [(s, 0.0), (0.0, 0.0)]
    raise ValueError(f"unknown shape {shape}")


class PlanAborted(Exception):
    pass


@dataclass
class PlannerConfig:
    takeoff_alt: float = 1.0      # m: where it normally hovers (the edge's ceiling is 2 m)
    approach_m: float = 1.0        # stop this far short of a target
    scan_step: float = 40.0        # deg per look while searching (~45 deg HFOV, some overlap)
    fence_margin: float = 0.4      # stay this far inside the edge's geofence
    max_step: float | None = SafetyConfig.max_step  # None: one goto, however far
    refine_beyond: float = 2.0     # target further than this: fly half way, look again, correct
    refine_max: int = 2            # at most this many re-looks on the way in
    confirm_below: float = 0.7     # sightings less confident than this need a second look
    confirm_m: float = 0.6         # ... that lands within this distance of the first
    max_pose_age: float = 0.5      # s; older frame poses (X-Pose-Age) can't place a target
    # Rangefinder height (the real drone; tools/auto_hover.py --rng is the field-tested original).
    # The FC's own height estimate drifts up on the ground while the motors spool, so a plain
    # takeoff stops before lift-off. With rng on: take off toward rng_ceiling, hold the moment
    # the rangefinder reads the target, keep correcting from it, land if it's lost or too high.
    rng: bool = False
    rng_alt: float = 0.7           # m above the floor: hover target (it sags ~0.2-0.4 m while moving)
    rng_ceiling: float = 1.0       # takeoff target handed to the FC so it keeps climbing
    rng_max_alt: float = 1.2       # land if the rangefinder reads above this
    rng_no_echo: float = 1.9       # readings above this are "no echo" (the sensor reports ~2.0)
    rng_tol: float = 0.12          # correct the height when it's off by more than this
    max_leg: float = 2.0           # rng: longest single sideways move (the Pi's gate caps it)
    rng_max_side: float = 2.0      # rng: a sideways command flies at most this far (field: 5 m
                                   # in 2 m legs turned to face each leg and wandered off)
    rng_fix_max: float = 0.15      # m per height correction (0.3 knocked it from 0.8 m to the floor)
    rng_fix_total: float = 0.45    # m: most the corrections may add up to, either way. They move
                                   # the edge's *commanded* height, so a drone that can't follow
                                   # (tired pack) had its target pushed up a step a second
    rng_stop_lead: float = 0.5     # s: stop the climb this far ahead (rangefinder + link lag)
    rng_sink_floor: float = 0.10   # land if this close to the floor (m of clearance) ...
    rng_sink_s: float = 3.0        # ... for this long. Not "below target": it flies a 2 m leg
                                   # steadily 0.3-0.4 m under its target (field 02:06)
    rng_yaw_drift: float = 45.0    # deg: land if the heading drifts this far with no turn sent
    pattern_size: float = 2.0      # m: a square's side / a circle's diameter when none is said
    max_wait: float = 60.0         # s: longest "hover for N seconds"


def compass(dn: float, de: float) -> str:
    names = ["north", "north-east", "east", "south-east", "south", "south-west", "west",
             "north-west"]
    return names[int(((math.degrees(math.atan2(de, dn)) + 22.5) % 360) // 45)]


def ago(t: float, now: float | None = None) -> str:
    s = max(0, int((now or time.time()) - t))
    if s < 60:
        return "a few seconds ago" if s < 15 else f"{s} seconds ago"
    m = s // 60
    return "a minute ago" if m == 1 else f"{m} minutes ago" if m < 60 else f"{m // 60} hours ago"


class Planner:
    def __init__(self, edge: EdgeClient, camera: EdgeCamera | None, *, locate: Locate,
                 ai=None, memory: MemorySearch | None = None, say: Say | None = None,
                 on_photo: OnPhoto | None = None, cfg: PlannerConfig | None = None,
                 safety: SafetyConfig | None = None,
                 on_approved: Callable[[str], None] | None = None):
        self.edge, self.camera = edge, camera
        self.on_approved = on_approved  # the edge's safety gate accepted this primitive (dashboard)
        self.locate, self.ai, self.memory = locate, ai, memory
        self._say = say
        self.on_photo = on_photo
        self.cfg = cfg or PlannerConfig()
        self.safety = safety or SafetyConfig()
        self._task: asyncio.Task | None = None
        self._urgent: set[asyncio.Task] = set()  # running stop / land plans
        self._accepted = False  # the last primitive _prim sent was accepted by the edge
        self._reply: str | None = None  # brain reply held until the edge accepts a step
        self.log: list[str] = []  # human-readable trace, for the dashboard and tests
        self.home: tuple[float, float] | None = None  # first takeoff point (the edge's fence)
        # rng mode
        self._ground = 0.12       # rangefinder reading on the floor, learned before each takeoff
        self._target: float | None = None  # height to hold (m above the floor); None = don't
        self._guard: asyncio.Task | None = None
        self._busy_prims = 0      # primitives in flight: no height corrections meanwhile
        self._vertical_busy = False  # an up/down move is running: no "sinking" check
        self._our_land = False    # we sent LAND: the FC leaving GUIDED isn't the pilot
        self._fix_sum = 0.0       # net height corrections since the target was last set (m)
        self._fix_capped = False  # said once that the corrections hit rng_fix_total
        self._retrace_to: tuple[float, float, float | None] | None = None  # where the last plan that moved started

    # --- public ------------------------------------------------------------------------------
    def submit(self, actions: list[dict], reply: str | None = None) -> asyncio.Task | None:
        """Run `actions`; `reply` is the brain's line ("On it."). When the plan starts with a
        flight step, the reply waits for the edge's verdict on it: spoken once accepted, dropped
        if refused, so a refusal is one line ("I'd love to turn around, but...") instead of
        "On it." followed by "Safety says no: ...". Other plans say it at once."""
        if not actions:
            return None
        if reply and actions[0]["kind"] not in DEFER_REPLY:
            asyncio.ensure_future(self.say(reply))  # scheduled before the plan's own lines
            reply = None
        if actions[0]["kind"] in ("stop", "land"):
            self.cancel()
            # asyncio only keeps a weak reference to tasks: hold this one until it finishes, or a
            # garbage collection could drop a STOP mid-flight
            t = asyncio.create_task(self._run(actions, reply))
            self._urgent.add(t)
            t.add_done_callback(self._urgent.discard)
            return t
        self.cancel()
        self._task = asyncio.create_task(self._run(actions, reply))
        return self._task

    def cancel(self) -> None:
        if self._task and not self._task.done():
            self._task.cancel()
        self._task = None

    @property
    def busy(self) -> bool:
        return bool(self._task and not self._task.done())

    async def wait(self) -> None:
        if self._task:
            try:
                await self._task
            except asyncio.CancelledError:
                pass

    # --- plumbing ----------------------------------------------------------------------------
    async def say(self, text: str) -> None:
        self.log.append(f"say: {text}")
        if self._say:
            try:
                await self._say(text)
            except Exception as e:  # speech must never break a plan
                log.warning("say failed: %s", e)

    async def _flush_reply(self) -> None:
        if r := self._reply:
            self._reply = None
            await self.say(r)

    def _pose(self) -> tuple[float, float, float | None] | None:
        """(n, e, height) now; height is the rangefinder target in rng mode."""
        if not self.tel:
            return None
        h = self._target if self.cfg.rng else self.tel.get("alt")
        return self.tel.get("n", 0.0), self.tel.get("e", 0.0), h

    async def _run(self, actions: list[dict], reply: str | None = None) -> None:
        self._reply = reply
        plan = {"start": self._pose(), "moved": False}
        _PLAN.set(plan)
        try:
            await self._run_actions(actions)
        finally:
            if plan["moved"] and plan["start"] is not None:
                self._retrace_to = plan["start"]

    async def _run_actions(self, actions: list[dict]) -> None:
        try:
            for a in actions:
                await getattr(self, f"_do_{a['kind']}")(a)
            await self._flush_reply()
        except asyncio.CancelledError:
            self.log.append("plan cancelled")
            raise
        except CommandRejected as e:
            self._reply = None  # "On it." then a refusal reads as two answers: just the refusal
            reason = str(e).rstrip(".")
            if await self._demo_line(e):
                await self.say(f"{reason[:1].upper()}{reason[1:]}.")
            else:
                await self.say(f"Safety says no: {reason}.")
        except PlanAborted as e:
            self._reply = None
            await self.say(str(e))
        except Exception as e:
            log.exception("plan failed")
            self._reply = None
            await self.say(f"Something went wrong: {e}.")

    async def _demo_line(self, e: CommandRejected) -> bool:
        """The edge in demo mode cleared the step and refused it with a sarcastic line that
        already says the safety checks passed ("I'd love to turn around, and my safety checks
        agree, but..."): no "Safety says no:" in front. Its `safety_reject` event carries
        "(demo)"; the ack that raised `e` arrives just before it, so give it a moment."""
        if "safety check" not in str(e).lower():
            return False  # a real gate reason (demo mode or not): keep the prefix
        events = getattr(self.edge, "events", None)
        if events is None:
            return False
        for _ in range(10):
            for ev in reversed(events[-5:]):
                if (ev.get("kind") == "safety_reject" and "(demo)" in str(ev.get("detail", ""))
                        and str(e).rstrip(".") in str(ev.get("detail", ""))):
                    return True
            await asyncio.sleep(0.02)
        return False

    async def _send(self, op: str, **args) -> str:
        """Send one primitive and report the gate's verdict; returns its id (done not awaited)."""
        self.log.append(f"{op} {args}" if args else op)
        if not self.edge.connected:
            raise PlanAborted("I'm not connected to the drone.")
        try:
            cid, ack = await self.edge.send(op, **args)   # = edge.run(), split to report the verdict
        except (asyncio.TimeoutError, OSError) as e:
            raise PlanAborted("The drone didn't answer, so that may not have gone through. If it's "
                              "flying, it hovers and then lands on its own.") from e
        if not ack["ok"]:
            raise CommandRejected(ack["reason"])
        self._accepted = True
        if op in MOVE_OPS and (plan := _PLAN.get()) is not None:
            plan["moved"] = True
        if self._reply:  # the edge took the first step: now "On it." is true (don't block on TTS)
            asyncio.ensure_future(self._flush_reply())
        if self.on_approved:
            try:
                self.on_approved(op)
            except Exception as e:  # reporting must never break a plan
                log.warning("on_approved failed: %s", e)
        return cid

    async def _prim(self, op: str, **args) -> dict:
        self._busy_prims += 1
        try:
            cid = await self._send(op, **args)
            done = await self.edge.wait_done(cid, self._timeout(op, args))
        finally:
            self._busy_prims -= 1
        if done["status"] != "ok":
            raise PlanAborted(f"That {op.replace('_', ' ')} was {done['status']}"
                              + (f": {done['reason']}." if done.get("reason") else "."))
        return done

    def _timeout(self, op: str, args: dict) -> float:
        """How long to wait for `done`. Fixed 60 s used to cut off any move longer than ~25 m
        at 0.5 m/s; with no distance cap (outdoors) it has to scale with the distance. The edge
        has its own per-command timeout, so this only has to be generous."""
        sc, pad = self.safety, 30.0
        t = lambda dxy, dz: flight_time(dxy, dz, 180.0, sc) * 1.5 + pad
        if op == "takeoff":
            return t(0.0, args.get("alt", 1.0))
        if op == "land":
            return max(90.0, t(0.0, -self.tel.get("alt", 0.0)))
        if op == "move_body":
            return t(math.hypot(args.get("fwd", 0.0), args.get("right", 0.0)), args.get("up", 0.0))
        if op in ("goto", "return_home"):
            n, e = (args["n"], args["e"]) if op == "goto" else (self.home or (0.0, 0.0))
            return t(math.hypot(n - self.tel.get("n", 0.0), e - self.tel.get("e", 0.0)), 0.0)
        return 90.0  # scan, yaw, stop, hover

    @property
    def tel(self) -> dict:
        return self.edge.telemetry or {}

    def _airborne(self) -> bool:
        if not self.tel.get("armed"):
            return False
        if self.cfg.rng:  # the FC's height drifts on the ground: trust the rangefinder
            h = self.rng_height()
            return self._target is not None if h is None else h > 0.15
        return self.tel.get("alt", 0.0) > 0.2

    # --- rangefinder height (cfg.rng) --------------------------------------------------------
    def rng_height(self) -> float | None:
        """Height above the floor by the rangefinder; None = no reading / no echo."""
        r = self.tel.get("range_m")
        return None if r is None or r < 0 or r > self.cfg.rng_no_echo else r - self._ground

    def _fc_guided(self) -> bool:
        return bool(self.tel.get("armed")) and self.tel.get("mode") == "GUIDED"

    async def _rng_takeoff(self, alt: float) -> None:
        c = self.cfg
        self._our_land = False
        if self.tel.get("armed"):  # already flying (FC height said otherwise): just hold here
            self._set_target(self.rng_height() or alt)
            self._start_guard()
            return
        ground = self.tel.get("range_m")
        if ground is None or ground < 0 or ground > 0.5:
            raise PlanAborted(f"My height sensor reads {ground} metres on the ground, so it isn't "
                              "pointing at the floor. Not taking off.")
        self._ground = ground
        target = min(alt, c.rng_max_alt - 0.2)
        cid = await self._send("takeoff", alt=c.rng_ceiling)
        # the takeoff is pre-empted by "stop" below; collect its done so it doesn't linger
        asyncio.ensure_future(self._drain(cid))
        t0, lost, prev, rate = time.monotonic(), None, None, 0.0
        try:
            while True:  # climb until the rangefinder says we're there
                await asyncio.sleep(0.1)
                h = self.rng_height()
                now = time.monotonic()
                if h is None:
                    lost = lost or now
                    if now - lost > 1.0:
                        raise PlanAborted("I lost my height sensor while taking off, so I'm landing.")
                    continue
                lost = None
                if prev and now > prev[1]:  # climb rate, smoothed: stop early enough not to overshoot
                    rate = 0.6 * rate + 0.4 * (h - prev[0]) / (now - prev[1])
                prev = (h, now)
                if h + max(0.0, rate) * c.rng_stop_lead >= target - 0.05:
                    break
                if now - t0 > 3 and not self._fc_guided():
                    raise PlanAborted("The flight controller didn't take the takeoff (pilot, "
                                      "failsafe or arming), so I've stopped.")
                if now - t0 > 8:
                    raise PlanAborted(f"I'm still only {h:.1f} metres up after 8 seconds, so I'm "
                                      "not lifting properly. Landing.")
            await self._prim("stop")  # hold here
        except BaseException:
            if self._fc_guided():
                self._our_land = True
                with _quiet():
                    await self.edge.run("land", timeout=60)
            raise
        self._set_target(target)
        self._start_guard()

    def _set_target(self, target: float | None) -> None:
        """A new height to hold: the corrections' running total starts again."""
        self._target = target
        self._fix_sum, self._fix_capped = 0.0, False

    def _adopt_flight(self) -> None:
        """Already in the air with no guard running (the station restarted or reconnected
        mid-flight): hold the current height and guard the flight from here on."""
        if not self.cfg.rng or (self._guard is not None and not self._guard.done()):
            return
        if not self._fc_guided():
            return  # the pilot or a failsafe has it: nothing of ours to guard
        if self._target is None:
            self._set_target(self.rng_height() or self.cfg.rng_alt)
        self._our_land = False
        self._start_guard()

    async def _drain(self, cid: str) -> None:
        with _quiet():
            await self.edge.wait_done(cid, 120)

    def _start_guard(self) -> None:
        if self._guard is None or self._guard.done():
            self._guard = asyncio.create_task(self._rng_guard())

    def _yaw(self) -> float | None:
        return self.tel.get("yaw")

    async def _rng_guard(self) -> None:
        """Beside every rng flight, every 0.2 s (never blocked by a correction):
        - hold the target height from the rangefinder (corrections run in the background);
        - land if the rangefinder is lost for 1 s, reads above rng_max_alt, or the drone stays
          within rng_sink_floor of the floor for rng_sink_s (field 2026-11-21: it sank from 0.5 m
          to 5 cm while "holding"; but flying low is fine);
        - land if the heading drifts more than rng_yaw_drift deg with no turn commanded (the
          same flight then yawed on its own just above the floor);
        - let go if the pilot or a failsafe takes the FC out of GUIDED (the edge never fights
          the pilot); our own LAND isn't reported as the pilot."""
        c = self.cfg
        lost = low_since = yaw_ref = None
        last_fix, fix = time.monotonic(), None
        while True:
            await asyncio.sleep(0.2)
            now = time.monotonic()
            if not self.tel or not self.edge.connected:
                # No telemetry is not "landed". station.edge_loop swaps in a fresh client after
                # a link drop, and its telemetry is empty until the first message: reading that
                # as disarmed ended this guard for the rest of the flight. Wait for the link (the
                # edge hovers, then lands, on its own meanwhile).
                lost = low_since = yaw_ref = None
                continue
            if not self.tel.get("armed"):
                self._target = None
                return  # landed (or disarmed)
            if self.tel.get("mode") != "GUIDED":
                self._target = None
                if not self._our_land:
                    self.cancel()
                    await self.say("The pilot or a failsafe has the drone now. I've let go.")
                return
            h = self.rng_height()
            idle = self._busy_prims == 0 and not self._our_land
            why = None
            if h is None:
                lost = lost or now
                if now - lost > 1.0:
                    why = "I lost my height sensor, so I'm landing."
            else:
                lost = None
                if h > c.rng_max_alt:
                    why = f"I'm {h:.1f} metres up, above the {c.rng_max_alt} metre limit, so I'm landing."
                elif (self._target is not None and not self._vertical_busy and not self._our_land
                      and h < c.rng_sink_floor):
                    low_since = low_since or now
                    if now - low_since > c.rng_sink_s:
                        why = (f"I'm sinking: {h:.2f} metres off the ground, so I'm "
                               "landing.")
                else:
                    low_since = None
            y = self._yaw()
            if not idle or y is None:
                yaw_ref = None  # a primitive may turn the drone (scan, yaw, face the move)
            elif yaw_ref is None:
                yaw_ref = y
            elif abs((y - yaw_ref + 180) % 360 - 180) > c.rng_yaw_drift:
                why = (f"I'm turning on my own ({abs((y - yaw_ref + 180) % 360 - 180):.0f} "
                       "degrees), so I'm landing.")
            if why:
                self._target = None
                self.submit([{"kind": "land"}])
                await self.say(why)
                return
            if (h is not None and self._target is not None and idle and (fix is None or fix.done())
                    and now - last_fix >= 1.0 and abs(self._target - h) > c.rng_tol):
                last_fix = now
                up = round(max(-c.rng_fix_max, min(c.rng_fix_max, self._target - h)), 2)
                # anti-windup: each correction moves the edge's commanded height whether or not
                # the drone followed the last one. Stop at rng_fix_total in either direction;
                # if it still sinks, the "sinking" check above lands it.
                room = c.rng_fix_total - self._fix_sum if up > 0 else -c.rng_fix_total - self._fix_sum
                up = round(min(up, room) if up > 0 else max(up, room), 2)
                if abs(up) < 0.02:
                    if not self._fix_capped:
                        self._fix_capped = True
                        log.warning("height corrections reached %+.2f m and the drone is still "
                                    "%.2f m off its target: not pushing further", self._fix_sum,
                                    self._target - h)
                    continue
                # in the background: the checks above must keep running while it flies
                fix = asyncio.ensure_future(self._correct(up))

    async def _correct(self, up: float) -> None:
        try:  # direct: a correction isn't a plan step (no dashboard verdict, no reply)
            await self.edge.run("move_body", timeout=10, up=up)
            self._fix_sum += up  # only what the edge accepted and finished
        except Exception as e:
            log.info("height correction %+.2f m: %s", up, e)

    async def _takeoff(self, alt: float) -> None:
        if self.cfg.rng:
            here = (self.tel.get("n", 0.0), self.tel.get("e", 0.0)) if self.tel else None
            self._accepted = False
            try:
                await self._rng_takeoff(alt)
            finally:
                if self._accepted and self.home is None and here is not None:
                    self.home = here
            return
        # the edge centres its fence on the first takeoff point and keeps it there (review3
        # #2); remember the same point so our clamp agrees with the edge's fence
        # The edge fixes its fence when it *accepts* the first takeoff; a refused one sets
        # nothing there, so it mustn't here either (a later takeoff from elsewhere would leave
        # our clamp and the edge's fence disagreeing).
        here = (self.tel.get("n", 0.0), self.tel.get("e", 0.0)) if self.tel else None
        self._accepted = False  # set by _prim once the edge acks
        try:
            await self._prim("takeoff", alt=alt)
        finally:
            if self._accepted and self.home is None and here is not None:
                self.home = here

    async def _ensure_airborne(self) -> None:
        if not self._airborne():
            await self._takeoff(self.cfg.takeoff_alt)
        else:
            self._adopt_flight()

    def _clamp(self, n: float, e: float) -> tuple[float, float]:
        if self.safety.fence_half is None:  # no geofence (outdoors): nothing to clamp to
            return n, e
        lim = self.safety.fence_half - self.cfg.fence_margin
        hn, he = self.home or (0.0, 0.0)
        return hn + max(-lim, min(lim, n - hn)), he + max(-lim, min(lim, e - he))

    async def _goto(self, n: float, e: float) -> None:
        """goto in legs of at most max_step (the gate refuses longer single moves)."""
        n, e = self._clamp(n, e)
        while True:
            cn, ce = self.tel.get("n", 0.0), self.tel.get("e", 0.0)
            d = math.hypot(n - cn, e - ce)
            if d < 0.15:
                return
            f = 1.0 if self.cfg.max_step is None else min(1.0, (self.cfg.max_step - 0.1) / d)
            await self._prim("goto", n=round(cn + f * (n - cn), 3), e=round(ce + f * (e - ce), 3))
            if f >= 1.0:
                return

    async def _face(self, bearing: float) -> None:
        await self._prim("yaw", deg=round(bearing % 360, 1), relative=False)

    def _check_pose(self, fr: Frame) -> None:
        """The separate video process stamps frames with the core's last pose; if the core
        went quiet, that pose is stale and any goal computed from it would be wrong."""
        if fr.pose.age > self.cfg.max_pose_age:
            raise PlanAborted(f"My position feed is {fr.pose.age:.0f} seconds old, so I can't "
                              "tell where things are. Holding here.")

    async def _frame(self) -> Frame:
        if not self.camera:
            raise PlanAborted("I have no camera feed.")
        try:
            return await self.camera.frame()
        except Exception as e:
            raise PlanAborted(f"I can't see anything: the camera feed is down ({e}).") from e

    # --- actions -----------------------------------------------------------------------------
    async def _do_takeoff(self, a: dict) -> None:
        if self._airborne():
            self._adopt_flight()
            return
        if self.cfg.rng:  # always the field-tested hover height; "go up" from there if wanted
            await self._takeoff(self.cfg.rng_alt)
            return
        await self._takeoff(a.get("alt") or self.cfg.takeoff_alt)

    async def _do_land(self, a: dict) -> None:
        self._target = None  # stop height corrections: they'd fight the descent
        self._our_land = True
        await self._prim("land")

    async def _do_stop(self, a: dict) -> None:
        await self._prim("stop")

    async def _do_home(self, a: dict) -> None:
        await self._prim("return_home")

    async def _do_move(self, a: dict) -> None:
        if self.cfg.rng:
            await self._ensure_airborne()
            await self._rng_move(a)
            return
        await self._ensure_airborne()
        fwd, right, up = a.get("fwd", 0.0), a.get("right", 0.0), a.get("up", 0.0)
        if fwd or right or up:
            await self._prim("move_body", fwd=fwd, right=right, up=up)
        if a.get("yaw_deg"):
            await self._turn(a["yaw_deg"])

    async def _rng_move(self, a: dict) -> None:
        """rng: height changes move the held target (within the rangefinder's limit); sideways
        moves go in legs of at most max_leg."""
        c = self.cfg
        fwd, right, up = a.get("fwd", 0.0), a.get("right", 0.0), a.get("up", 0.0)
        if up:
            now = self._target if self._target is not None else (self.rng_height() or c.rng_alt)
            want = now + up
            top = c.rng_max_alt - 0.2
            if want > top + 1e-6:
                raise PlanAborted(f"That would take me to {want:.1f} metres. My height sensor "
                                  f"only lets me fly up to {top:.1f}, so I'm staying at {now:.1f}.")
            if want < 0.2:
                raise PlanAborted("That's too close to the floor. Say land if you want me down.")
            self._vertical_busy = True
            try:
                await self._prim("move_body", up=round(want - (self.rng_height() or now), 2))
            finally:
                self._vertical_busy = False
            self._set_target(want)
        dist = math.hypot(fwd, right)
        if dist > c.rng_max_side:
            f = c.rng_max_side / dist
            fwd, right = fwd * f, right * f
            await self.say(f"I'll keep it to {c.rng_max_side:.0f} metres at a time out here.")
            dist = c.rng_max_side
        while dist > 0.01:
            f = min(1.0, c.max_leg / dist)
            await self._prim("move_body", fwd=round(fwd * f, 3), right=round(right * f, 3))
            fwd, right = fwd * (1 - f), right * (1 - f)
            dist = math.hypot(fwd, right)
        if a.get("yaw_deg"):
            await self._turn(a["yaw_deg"])

    async def _do_compass(self, a: dict) -> None:
        """North / east metres (world frame), then face a compass heading if asked. An absolute
        goto from where it has stopped: a body move would need the heading, and telemetry's lags
        the edge's target right after a turn (27 deg off in the sim)."""
        await self._ensure_airborne()
        north, east, up = a.get("north", 0.0), a.get("east", 0.0), a.get("up", 0.0)
        dist = math.hypot(north, east)
        if dist >= 0.05:
            cap = self.cfg.rng_max_side if self.cfg.rng else None
            if cap is not None and dist > cap:
                north, east = north * cap / dist, east * cap / dist
                await self.say(f"I'll keep it to {cap:.0f} metres at a time out here.")
            await self._settle()
            await self._goto(round(self.tel.get("n", 0.0) + north, 3),
                             round(self.tel.get("e", 0.0) + east, 3))
        if up:
            await self._do_move({"up": up})
        if a.get("face") is not None:
            await self._face(a["face"])

    async def _do_height(self, a: dict) -> None:
        """To a height above the ground: the rangefinder target in rng mode, else the FC's."""
        await self._ensure_airborne()
        if self.cfg.rng:
            now = self._target if self._target is not None else (self.rng_height() or self.cfg.rng_alt)
        else:
            # the edge calls a climb done within tolerance and relative moves build on its
            # target: read the height once it has stopped, not while it's still coasting up
            await self._settle()
            now = self.tel.get("alt", 0.0)
        up = round(a["alt"] - now, 2)
        if abs(up) >= 0.05:
            await self._do_move({"up": up})

    async def _settle(self, timeout: float = 3.0) -> None:
        """Wait (at most `timeout`) until telemetry says the drone has stopped moving."""
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            t = self.tel
            if max(abs(t.get(k) or 0.0) for k in ("vn", "ve", "vd")) < 0.1:
                return
            await asyncio.sleep(0.1)

    async def _do_wait(self, a: dict) -> None:
        """Hover in place: stop whatever the edge is doing, then hold for the time asked."""
        if self._airborne():
            await self._prim("hover")
        await asyncio.sleep(max(0.0, min(float(a.get("seconds", 0.0)), self.cfg.max_wait)))

    async def _do_pattern(self, a: dict) -> None:
        """A square / triangle / circle / out-and-back from here, ending back here. Sent as
        corner-to-corner body moves: the edge builds each on its last *target* and heading (not
        the measured position), so the shape closes even when a leg stops a little short."""
        await self._ensure_airborne()
        shape, side = a["shape"], a.get("side") or "right"
        size = a.get("size") or self.cfg.pattern_size
        legs = self._legs(pattern_points(shape, size, side))
        longest = max(math.hypot(*leg) for leg in legs)
        caps = [c for c in (self.cfg.rng_max_side if self.cfg.rng else None, self.cfg.max_step)
                if c]
        # The edge builds each leg on its last *target* but measures the step against the cap
        # from where the drone *is*, up to pos_tolerance short of it: a 2 m square's second
        # side came out as 2.3 m and was refused (tools/nav_check, 2026-11-23). Keep that spare.
        cap = min(caps) - ExecConfig.pos_tolerance if caps else None
        if cap is not None and longest > cap:
            # out here a command flies at most rng_max_side: shrink the shape, don't distort it
            size *= cap / longest
            legs = self._legs(pattern_points(shape, size, side))
            if a.get("size"):  # they asked for a size: say it's smaller
                await self.say(f"I'll keep the {shape.replace('_', ' ')} to "
                               f"{size:.1f} metres out here.")
        for fwd, right in legs:
            await self._do_move({"fwd": round(fwd, 3), "right": round(right, 3)})

    @staticmethod
    def _legs(pts: list[tuple[float, float]]) -> list[tuple[float, float]]:
        prev, legs = (0.0, 0.0), []
        for p in pts:
            legs.append((p[0] - prev[0], p[1] - prev[1]))
            prev = p
        return legs

    async def _do_retrace(self, a: dict) -> None:
        """Back to where this plan started if it has moved ("forward 2 m and come back"), else
        to where the last plan that moved started ("undo that")."""
        plan = _PLAN.get()
        to = plan["start"] if plan and plan["moved"] else self._retrace_to
        if to is None:
            raise PlanAborted("I haven't been anywhere yet, so there's nowhere to go back to.")
        if not self._airborne():
            raise PlanAborted("I'm on the ground. Take off first and I'll go back.")
        n, e, h = to
        dist = math.hypot(n - self.tel.get("n", 0.0), e - self.tel.get("e", 0.0))
        if self.cfg.rng and dist > self.cfg.rng_max_side:
            raise PlanAborted(f"That's {dist:.1f} metres back, and out here I only fly "
                              f"{self.cfg.rng_max_side:.0f} at a time. Say which way and how far.")
        if dist >= 0.15:  # absolute goto: a relative move would add the last leg's shortfall
            await self._goto(n, e)
        if h is not None:  # rng: the rangefinder target; else the FC's height
            await self._do_height({"alt": h})

    async def _turn(self, deg: float) -> None:
        """A full turn ("do a 360") is the edge's scan primitive (tools/auto_hover --test spin):
        a relative yaw of 360 would end where it started and do nothing."""
        if abs(deg) >= 359:
            await self._prim("scan", rate_dps=SPIN_DPS)
        else:
            await self._prim("yaw", deg=deg, relative=True)

    async def look_for(self, target: str, K: dict) -> tuple[dict, Frame] | None:
        """One look. A shaky sighting (conf < confirm_below) must be seen again, in about the
        same place, in a second frame; the two fixes are averaged."""
        fr = await self._frame()
        self._check_pose(fr)
        hit = await self.locate(self.ai, target, fr.jpeg, fr.pose, K)
        if hit and hit.get("conf", 1.0) < self.cfg.confirm_below:
            fr2 = await self._frame()
            hit2 = await self.locate(self.ai, target, fr2.jpeg, fr2.pose, K)
            if not hit2 or math.hypot(hit2["n"] - hit["n"], hit2["e"] - hit["e"]) > self.cfg.confirm_m:
                self.log.append("unconfirmed sighting")
                return None
            hit = {**hit, "n": (hit["n"] + hit2["n"]) / 2, "e": (hit["e"] + hit2["e"]) / 2,
                   "conf": max(hit["conf"], hit2["conf"])}
        return (hit, fr) if hit else None

    async def find(self, target: str) -> tuple[dict, Frame] | None:
        """Look ahead, then turn step by step for at most one full turn."""
        K = await self.camera.K() if self.camera else {}
        steps = int(round(360 / self.cfg.scan_step))
        for i in range(steps):
            found = await self.look_for(target, K)
            self.log.append(f"look {i}: {'found' if found else 'nothing'}")
            if found:
                return found
            if i < steps - 1:
                await self._prim("yaw", deg=self.cfg.scan_step, relative=True)
        return None

    async def _do_mission(self, a: dict) -> None:
        target = re.sub(r"^(the|a|an|my|that) ", "", a["target"].strip().lower())
        await self._ensure_airborne()
        if POINTING.search(target):
            target = await self._pointed_at()
        found = await self.find(target)
        if not found:
            raise PlanAborted(f"I looked all the way round and couldn't find the {target}.")
        hit, _ = found
        if not hit.get("upper_bound"):
            hit = await self._refine(target, hit)
        cn, ce = self.tel.get("n", 0.0), self.tel.get("e", 0.0)
        d = math.hypot(hit["n"] - cn, hit["e"] - ce)
        if hit.get("upper_bound"):
            # it fills the frame: it is closer than any range we could compute, so flying
            # "1 m short" of a guess would fly into it (review3 #3). Just face it.
            self.log.append("target fills the frame: not approaching")
            await self.say(f"The {target} is right in front of me.")
        elif d > self.cfg.approach_m + 0.2:
            f = (d - self.cfg.approach_m) / d
            await self._goto(cn + f * (hit["n"] - cn), ce + f * (hit["e"] - ce))
        cn, ce = self.tel.get("n", 0.0), self.tel.get("e", 0.0)
        await self._face(math.degrees(math.atan2(hit["e"] - ce, hit["n"] - cn)))
        if a.get("action", "none") != "none":
            await self._task_at(a["action"], a.get("question", ""), target)
        else:
            await self.say(f"At the {target}.")
        after = a.get("after", "stay")
        if after == "come_back":
            await self._prim("return_home")
        elif after == "land":
            await self._prim("land")

    def _stop_point(self, hit: dict) -> tuple[float, float, float]:
        """(n, e, distance to target) of the point approach_m short of the target."""
        cn, ce = self.tel.get("n", 0.0), self.tel.get("e", 0.0)
        d = math.hypot(hit["n"] - cn, hit["e"] - ce)
        f = max(0.0, (d - self.cfg.approach_m) / d) if d > 1e-6 else 0.0
        return cn + f * (hit["n"] - cn), ce + f * (hit["e"] - ce), d

    async def _refine(self, target: str, hit: dict) -> dict:
        """Closed-loop approach. Range from box size is the weakest number in locate()
        (+-30 % or worse); bearing is good. So while the target is far: fly half way to the
        stop point, face it, look again and take the new fix. Each re-look shrinks the range
        error with the distance."""
        K = await self.camera.K() if self.camera else {}
        for i in range(self.cfg.refine_max):
            sn, se, d = self._stop_point(hit)
            if d <= self.cfg.refine_beyond:
                break
            cn, ce = self.tel.get("n", 0.0), self.tel.get("e", 0.0)
            await self._goto((cn + sn) / 2, (ce + se) / 2)
            cn, ce = self.tel.get("n", 0.0), self.tel.get("e", 0.0)
            await self._face(math.degrees(math.atan2(hit["e"] - ce, hit["n"] - cn)))
            again = await self.look_for(target, K)
            self.log.append(f"refine {i}: {'new fix' if again else 'lost it, keeping the old fix'}")
            if not again:
                break
            hit = again[0]
            if hit.get("upper_bound"):  # now filling the frame: close enough, stop refining
                break
        return hit

    async def _pointed_at(self) -> str:
        """"Fly to where I'm pointing": the vision model names what the person points at."""
        if not self.ai:
            raise PlanAborted("I need my vision model to see where you're pointing.")
        fr = await self._frame()
        try:
            name = await vision.ask(self.ai, "pointing", fr.jpeg)
        except Exception as e:
            raise PlanAborted(f"My vision model didn't answer ({e}).") from e
        name = re.sub(r"^(the|a|an) ", "", name.strip(" .").lower())
        if not name or "none" in name or len(name.split()) > 4:
            raise PlanAborted("I can't tell what you're pointing at. Point at it in front of me.")
        await self.say(f"You're pointing at the {name}.")
        return name

    async def _task_at(self, action: str, question: str, target: str = "") -> None:
        fr = await self._frame()
        if action == "photo":
            # the photo goes out first (the dashboard flashes it at once); the caption is only for
            # what Jarvis says, and must never hold the photo back
            if self.on_photo:
                await self.on_photo(fr, target)
            caption = ""
            if self.ai:
                try:
                    caption = await asyncio.wait_for(vision.ask(self.ai, "describe", fr.jpeg), 8.0)
                except Exception as e:
                    log.warning("caption failed: %s", e)
            await self.say("Got the shot." + (f" {caption}" if caption else ""))
            return
        if not self.ai:
            raise PlanAborted("My vision model is offline, so I can't do that.")
        try:
            answer = await vision.ask(self.ai, action, fr.jpeg, question)
        except Exception as e:
            raise PlanAborted(f"My vision model didn't answer ({e}).") from e
        self.log.append(f"vision {action}: {answer}")
        await self.say(answer or "I can't make anything out.")

    async def _do_look(self, a: dict) -> None:
        await self._task_at(a.get("action", "describe"), a.get("question", ""))

    async def _do_recall(self, a: dict) -> None:
        q = a["query"]
        if not self.memory:
            raise PlanAborted("My memory is offline.")
        try:
            hits = await self.memory(q)
        except Exception as e:
            raise PlanAborted(f"I can't reach my memory ({e}).") from e
        if not hits:
            raise PlanAborted(f"I don't remember seeing any {q}.")
        h = hits[0]
        dist = math.hypot(h["n"], h["e"])
        where = ("right by where we took off" if dist < 0.5 else
                 f"about {dist:.0f} metre{'s' if round(dist) != 1 else ''} "
                 f"{compass(h['n'], h['e'])} of the takeoff spot")
        await self.say(f"I saw {q} {where}, {ago(h['t'])}.")
        if a.get("fly_there"):
            await self._ensure_airborne()
            await self._goto(h["n"], h["e"])
            await self._face(h.get("yaw", 0.0))
            await self.say("This is where I saw it.")
