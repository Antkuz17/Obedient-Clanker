"""Safety gate: every primitive from the station passes through check() before anything flies.

The gate also computes the concrete target, so what gets checked is exactly what gets flown.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

from .config import SafetyConfig
from .vehicle import VehicleState, body_to_ned, ned_to_body, wrap360, yaw_err


@dataclass
class Perception:
    """Latest on-board AI outputs (fed by the vision process). None = unknown / stale."""
    person_m: float | None = None   # nearest person ahead; None = nobody seen
    detector_alive: bool = False     # vision process reported within the stale window
    obstacle_m: float | None = None  # forward ultrasonic echo; None = nothing within its range
    obstacle_alive: bool = False     # forward sensor reported within the stale window
    person_t: float = 0.0            # time.monotonic() of the last reading of each
    obstacle_t: float = 0.0


@dataclass
class Verdict:
    ok: bool
    reason: str = ""
    target: tuple[float, float, float, float] | None = None  # n, e, alt, yaw
    extra: dict = field(default_factory=dict)


def _reject(reason: str) -> Verdict:
    return Verdict(False, reason)


def person_room(p: Perception, c: SafetyConfig, since: float = 0.0) -> float | None:
    """Metres to the nearest person, read cautiously (estimates can be person_range_error
    too far). None = nobody seen, or only seen before `since` (the drone has turned since, so
    that reading is about another direction)."""
    if p.person_m is None or p.person_t < since:
        return None
    return p.person_m / (1 + c.person_range_error)


def obstacle_ahead(p: Perception, since: float = 0.0) -> float | None:
    return None if p.obstacle_m is None or p.obstacle_t < since else p.obstacle_m


def path_blocked(p: Perception, ahead: float, c: SafetyConfig,
                 since: float = 0.0) -> str | None:
    """Would flying `ahead` metres forward end inside a clearance? Reason, or None. Readings
    older than `since` are ignored."""
    if (room := person_room(p, c, since)) is not None and room - ahead < c.person_close_m:
        return f"person {p.person_m:.1f} m ahead"
    if (ob := obstacle_ahead(p, since)) is not None and ob - ahead < c.obstacle_close_m:
        return f"something {p.obstacle_m:.1f} m ahead"
    return None


class SafetyGate:
    def __init__(self, cfg: SafetyConfig | None = None):
        self.cfg = cfg or SafetyConfig()

    def check(self, op: str, args: dict, s: VehicleState, airborne: bool,
              perception: Perception | None = None,
              base: tuple[float, float, float, float] | None = None,
              home: tuple[float, float] | None = None) -> Verdict:
        """base = last commanded (n, e, alt, yaw). Relative moves build on it, not on the
        measured position, so small tracking errors don't compound move after move."""
        c = self.cfg
        p = perception or Perception()
        bn, be, balt, byaw = base if base is not None else (s.n, s.e, s.alt, s.yaw)
        hn, he = home if home is not None else (0.0, 0.0)

        if op in ("stop", "hover"):
            return Verdict(True, target=(s.n, s.e, s.alt, s.yaw))
        if op == "land":
            return Verdict(True)

        if not s.connected:
            return _reject("no flight controller link")

        if op == "takeoff":
            alt = args["alt"]
            if airborne or s.armed:
                return _reject("already flying")
            if alt < c.alt_min:
                return _reject(f"takeoff altitude {alt:.1f} m below {c.alt_min} m")
            if c.max_takeoff_alt is not None and alt > c.max_takeoff_alt:
                return _reject(f"takeoff altitude {alt:.1f} m above {c.max_takeoff_alt} m")
            if s.battery_v < c.batt_reject_v:
                return _reject(f"battery {s.battery_v:.2f} V below {c.batt_reject_v} V")
            if s.flow_quality < c.flow_quality_min:
                return _reject(f"optical flow quality {s.flow_quality} too low to hold position")
            if not s.pos_ok:
                return _reject("flight controller has no position estimate (EKF)")
            if not s.limits_ok:
                return _reject("flight controller speed limits not confirmed")
            # the fence stays where the first takeoff put it (review3 #2): taking off again
            # from outside it would let the next moves start beyond the walls we planned for
            if (c.fence_half is not None and home is not None
                    and (abs(s.n - hn) > c.fence_half or abs(s.e - he) > c.fence_half)):
                return _reject(f"drone is ({s.n - hn:.1f}, {s.e - he:.1f}) m from the first "
                               f"takeoff point, outside the +-{c.fence_half} m flying area; "
                               "carry it back, or restart the edge to reset the area")
            return Verdict(True, target=(s.n, s.e, alt, s.yaw))

        # everything below needs to be in the air
        if not airborne:
            return _reject("not flying; take off first")
        if s.mode != "GUIDED":
            return _reject(f"pilot has control (mode {s.mode}); not sending moves")
        if not s.pos_ok:
            return _reject("flight controller lost its position estimate")
        if s.battery_v < c.batt_reject_v:
            return _reject(f"battery {s.battery_v:.2f} V below {c.batt_reject_v} V; land now")

        if op == "yaw":
            yaw = args["deg"] + (byaw if args.get("relative") else 0.0)
            return Verdict(True, target=(bn, be, balt, wrap360(yaw)))

        if op == "scan":
            rate = args.get("rate_dps", 20.0)
            if rate <= 0 or (c.scan_rate_max is not None and rate > c.scan_rate_max):
                return _reject(f"scan rate {rate} deg/s not allowed")
            return Verdict(True, target=(bn, be, balt, byaw), extra={"rate_dps": rate})

        if op == "move_body":
            dn, de = body_to_ned(args.get("fwd", 0.0), args.get("right", 0.0), byaw)
            n, e, alt = bn + dn, be + de, balt + args.get("up", 0.0)
        elif op == "goto":
            n, e, alt = args["n"], args["e"], args.get("alt", balt)
        elif op == "return_home":
            n, e, alt = hn, he, balt
        else:
            return _reject(f"unknown op {op}")

        if s.flow_quality < c.flow_quality_min:
            return _reject(f"optical flow quality {s.flow_quality} too low to move")
        step = math.dist((n, e, alt), (s.n, s.e, s.alt))
        if c.max_step is not None and step > c.max_step:
            return _reject(f"move of {step:.1f} m exceeds the {c.max_step} m limit")
        if alt < c.alt_min:
            return _reject(f"target altitude {alt:.1f} m below {c.alt_min} m")
        if c.alt_max is not None and alt > c.alt_max:
            return _reject(f"target altitude {alt:.1f} m above {c.alt_max} m")
        if c.fence_half is not None and (abs(n - hn) > c.fence_half or abs(e - he) > c.fence_half):
            return _reject(f"target ({n - hn:.1f}, {e - he:.1f}) from home is outside the "
                           f"+-{c.fence_half} m geofence")
        # The person detector and the obstacle sensor only look forward. A horizontal move
        # longer than face_travel_min_m turns to face the way it flies first (the executor
        # does that), so the sensors watch the path; shorter moves keep their heading.
        fwd, _ = ned_to_body(n - s.n, e - s.e, s.yaw)
        dxy = math.hypot(n - s.n, e - s.e)
        extra: dict = {}
        if dxy > c.face_travel_min_m:
            bearing = wrap360(math.degrees(math.atan2(e - s.e, n - s.n)))
            extra["face_travel"] = bearing
            # the sensors only describe the path now if we already face it
            seen, ahead = abs(yaw_err(bearing, s.yaw)) <= 30.0, dxy
        else:
            seen, ahead = fwd > c.forward_check_min_m, fwd
        if seen:
            if (why := path_blocked(p, ahead, c)) is not None:
                return _reject(f"{why}; not moving there")
        if seen or extra:
            if c.require_person_detector and not p.detector_alive:
                return _reject("person detector is not running; not moving forward")
            if c.require_obstacle_sensor and not p.obstacle_alive:
                return _reject("forward obstacle sensor is not reporting; not moving forward")
        return Verdict(True, target=(n, e, alt, byaw), extra=extra)
