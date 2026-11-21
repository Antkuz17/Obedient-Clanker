"""Edge limits. Tune at the venue; every number here is a safety decision."""
from __future__ import annotations

from dataclasses import dataclass


@dataclass
class SafetyConfig:
    alt_min: float = 0.3          # m; moves can't go below this (only takeoff/land): ground floor
    # Flying outdoors (2026-11-21): no ceiling, no geofence, no per-move cap. None = no limit;
    # set a number to bring one back (the gate and the planner both honour it).
    alt_max: float | None = None          # m above the takeoff ground
    fence_half: float | None = None       # m; box +-fence_half N/E around the first takeoff
    max_step: float | None = None         # m; largest single move
    max_takeoff_alt: float | None = None  # m
    # Speeds the edge writes to the FC on every connect (WP_SPD / WP_SPD_UP / WP_SPD_DN). No
    # slowing on purpose: horizontal and climb are ArduPilot's parameter maxima, so the real top
    # speed is whatever the airframe and the FC's lean-angle limit (ANGLE_MAX) give. Descent stays
    # at ArduPilot's default: a quad sinking much faster into its own downwash can lose lift
    # (vortex ring state). That's aerodynamics, not a limit we picked.
    speed_xy: float = 20.0        # m/s
    speed_z: float = 10.0         # m/s, climb
    speed_dn: float = 1.5         # m/s, descent
    yaw_rate: float = 90.0        # deg/s; only used to size timeouts (the FC turns at its own rate)
    scan_rate_max: float | None = None  # deg/s; None = no cap on the scan op's turn rate
    batt_reject_v: float = 10.6   # 3S: refuse new moves below this
    batt_land_v: float = 10.2     # 3S: auto-land below this
    flow_quality_min: int = 50    # refuse moves when flow is this bad (0-255)
    person_close_m: float = 1.0   # refuse forward moves that would end closer to a person
    # forward HC-SR04 (tools/rangefinder.py "fwd_m"): walls, furniture, a torso too close for
    # the camera detector. Refuse forward moves that would end closer than this to an echo.
    obstacle_close_m: float = 0.7
    # with no live forward sensor, refuse forward moves (python -m edge --require-obstacle-sensor
    # once it's mounted; off by default so the drone flies before the hardware is on)
    require_obstacle_sensor: bool = False
    # Fail-safe (docs/flow.md): with no live person detector, refuse forward moves. Off by
    # default so sim/SITL work without a vision process; `python -m edge --mav usb` turns it on.
    require_person_detector: bool = False


@dataclass
class WatchdogConfig:
    hover_after_s: float = 1.0
    land_after_s: float = 5.0


@dataclass
class ExecConfig:
    pos_tolerance: float = 0.3    # m; optical flow can't do better (review A13)
    # short moves: done within frac x distance, floored at tol_min, capped at pos_tolerance.
    # fwd=0.5 used to finish at +0.24 m (flow-only SITL); now it must get within 0.15 m
    pos_tolerance_frac: float = 0.3
    pos_tolerance_min: float = 0.1
    yaw_tolerance: float = 5.0    # deg
    cmd_timeout_pad_s: float = 8.0
    # Timeouts assume at most these speeds (acceleration, wind, short moves never reach the top
    # speed). They only make timeouts longer: never a speed limit.
    timeout_speed_xy: float = 2.0  # m/s
    timeout_speed_z: float = 1.0   # m/s


def flight_time(dxy: float, dz: float, dyaw: float, safety: "SafetyConfig",
                ex: "ExecConfig | None" = None) -> float:
    """Generous time a move should take (no padding): horizontal, vertical, turn."""
    ex = ex or ExecConfig()
    vz = safety.speed_z if dz > 0 else safety.speed_dn
    return (dxy / min(safety.speed_xy, ex.timeout_speed_xy)
            + abs(dz) / min(vz, ex.timeout_speed_z)
            + abs(dyaw) / safety.yaw_rate)
