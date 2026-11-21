"""Safety gate: every primitive from the station passes through check() before anything flies.

The gate also computes the concrete target, so what gets checked is exactly what gets flown.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

from .config import SafetyConfig
from .vehicle import VehicleState, body_to_ned, ned_to_body, wrap360


@dataclass
class Perception:
    """Latest on-board AI outputs (fed by the vision process). None = unknown / stale."""
    person_m: float | None = None


@dataclass
class Verdict:
    ok: bool
    reason: str = ""
    target: tuple[float, float, float, float] | None = None  # n, e, alt, yaw
    extra: dict = field(default_factory=dict)


def _reject(reason: str) -> Verdict:
    return Verdict(False, reason)


class SafetyGate:
    def __init__(self, cfg: SafetyConfig | None = None):
        self.cfg = cfg or SafetyConfig()

    def check(self, op: str, args: dict, s: VehicleState, airborne: bool,
              perception: Perception | None = None) -> Verdict:
        c = self.cfg
        p = perception or Perception()

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
            if not c.alt_min <= alt <= c.max_takeoff_alt:
                return _reject(f"takeoff altitude {alt:.1f} m outside "
                               f"{c.alt_min}-{c.max_takeoff_alt} m")
            if s.battery_v < c.batt_reject_v:
                return _reject(f"battery {s.battery_v:.2f} V below {c.batt_reject_v} V")
            if s.flow_quality < c.flow_quality_min:
                return _reject(f"optical flow quality {s.flow_quality} too low to hold position")
            return Verdict(True, target=(s.n, s.e, alt, s.yaw))

        # everything below needs to be in the air
        if not airborne:
            return _reject("not flying; take off first")
        if s.battery_v < c.batt_reject_v:
            return _reject(f"battery {s.battery_v:.2f} V below {c.batt_reject_v} V; land now")

        if op == "yaw":
            yaw = args["deg"] + (s.yaw if args.get("relative") else 0.0)
            return Verdict(True, target=(s.n, s.e, s.alt, wrap360(yaw)))

        if op == "scan":
            rate = args.get("rate_dps", 20.0)
            if not 0 < rate <= c.scan_rate_max:
                return _reject(f"scan rate {rate} deg/s outside 0-{c.scan_rate_max}")
            return Verdict(True, target=(s.n, s.e, s.alt, s.yaw), extra={"rate_dps": rate})

        if op == "move_body":
            dn, de = body_to_ned(args.get("fwd", 0.0), args.get("right", 0.0), s.yaw)
            n, e, alt = s.n + dn, s.e + de, s.alt + args.get("up", 0.0)
        elif op == "goto":
            n, e, alt = args["n"], args["e"], args.get("alt", s.alt)
        elif op == "return_home":
            n, e, alt = 0.0, 0.0, s.alt
        else:
            return _reject(f"unknown op {op}")

        if s.flow_quality < c.flow_quality_min:
            return _reject(f"optical flow quality {s.flow_quality} too low to move")
        step = math.dist((n, e, alt), (s.n, s.e, s.alt))
        if step > c.max_step:
            return _reject(f"move of {step:.1f} m exceeds the {c.max_step} m limit")
        if not c.alt_min <= alt <= c.alt_max:
            return _reject(f"target altitude {alt:.1f} m outside {c.alt_min}-{c.alt_max} m")
        if abs(n) > c.fence_half or abs(e) > c.fence_half:
            return _reject(f"target ({n:.1f}, {e:.1f}) is outside the "
                           f"+-{c.fence_half} m geofence")
        fwd, _ = ned_to_body(n - s.n, e - s.e, s.yaw)
        if fwd > 0.2 and p.person_m is not None and p.person_m < c.person_close_m:
            return _reject(f"person {p.person_m:.1f} m ahead; not moving forward")
        return Verdict(True, target=(n, e, alt, s.yaw))
