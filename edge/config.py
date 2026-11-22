"""Edge limits. Tune at the venue; every number here is a safety decision."""
from __future__ import annotations

from dataclasses import dataclass


@dataclass
class SafetyConfig:
    alt_min: float = 0.3          # m; moves can't go below this (only takeoff/land): ground floor
    # Outdoors (2026-11-21). None = no limit; set a number to bring one back.
    alt_max: float | None = 5.0           # m above the takeoff ground: the team's ceiling
    fence_half: float | None = None       # m; box +-fence_half N/E around the first takeoff
    max_step: float | None = None         # m; largest single move
    max_takeoff_alt: float | None = 5.0   # m
    # Speeds the edge writes to the FC on every connect (WP_SPD / WP_SPD_UP / WP_SPD_DN).
    # Horizontal: the fastest speed at which the person / obstacle checks can still stop the
    # drone in time (max_checked_speed() below, from the braking measured in SITL). The edge
    # clamps anything faster. Climb: no sensor looks up or down, so ArduPilot's maximum; the
    # 5 m ceiling is a position target the FC decelerates into. Descent: ArduPilot's default; a
    # quad sinking much faster into its own downwash can lose lift (vortex ring state).
    speed_xy: float = 1.5         # m/s
    speed_z: float = 10.0         # m/s, climb
    speed_dn: float = 1.5         # m/s, descent
    yaw_rate: float = 90.0        # deg/s; only used to size timeouts (the FC turns at its own rate)
    # How hard the FC brakes (and speeds up), written with the speeds (WP_ACC / WP_JERK). With
    # ArduPilot's defaults (2.5 m/s2, jerk 1.0 m/s3) SITL took ~1.3 s to even start slowing
    # after a stop and 4-9 m to stand still. With these: 0.70 / 1.12 / 1.57 / 2.68 m from
    # 1 / 1.5 / 2 / 3 m/s, braking starting 0.4-0.5 s after the stop (tools/sitl_brake.py,
    # 2026-11-21). 5 m/s2 is a ~27 deg lean, inside ArduPilot's default 30 deg lean limit.
    accel_xy: float = 5.0         # m/s2
    jerk_xy: float = 10.0         # m/s3
    # Stopping model, an upper bound on those SITL runs: d <= v * (latency + stop_react_s)
    # + v^2 / (2 * brake_decel). Re-measure on the real drone and update.
    stop_react_s: float = 0.45    # stop sent -> FC braking
    brake_decel: float = 2.0      # m/s2, average once braking
    # The sensors that have to stop it. Every forward check uses these, and max_checked_speed()
    # makes sure speed_xy lets each one stop the drone inside what it can see.
    face_travel_min_m: float = 0.5   # longer horizontal moves turn to face the way they fly first
    obstacle_range_m: float = 2.0    # forward HC-SR04: reliable echoes up to here
    obstacle_latency_s: float = 0.07 # 20 Hz pings + Arduino serial + UDP
    person_range_m: float = 4.0      # person detector: distance estimate usable up to here
    person_latency_s: float = 0.3    # camera frame + inference (10 Hz) + UDP: measure on the Pi
    person_range_error: float = 0.3  # a person may be up to 30 % closer than the estimate says
    scan_rate_max: float | None = None  # deg/s; None = no cap on the scan op's turn rate
    # 3S, read under load. Was 10.6 / 10.2: the field pack (12.05 V at rest) sagged to 10.4 V
    # within 5 s of hovering at ~80% throttle, so every move was refused and it auto-landed
    # (2026-11-22, cloud lead's call; 9.6 V loaded = 3.2 V/cell)
    batt_reject_v: float = 10.0   # 3S: refuse new moves below this
    batt_land_v: float = 9.6      # 3S: auto-land below this (for 2 s)
    flow_quality_min: int = 50    # refuse moves when flow is this bad (0-255)
    person_close_m: float = 1.0   # clearance: never end or stop closer than this to a person
    # forward HC-SR04 (tools/rangefinder.py "fwd_m"): walls, furniture, a torso too close for
    # the camera detector. Clearance to keep from an echo.
    obstacle_close_m: float = 0.6
    # with no live forward sensor, refuse forward moves (python -m edge --require-obstacle-sensor
    # once it's mounted; off by default so the drone flies before the hardware is on)
    require_obstacle_sensor: bool = False
    # Fail-safe (docs/flow.md): with no live person detector, refuse forward moves. Off by
    # default so sim/SITL work without a vision process; `python -m edge --mav usb` turns it on.
    require_person_detector: bool = False


def stopping_distance(v: float, latency_s: float, c: SafetyConfig) -> float:
    """Metres travelled from a sensor seeing something at speed v (m/s) to standing still."""
    v = max(0.0, v)
    return v * (latency_s + c.stop_react_s) + v * v / (2 * c.brake_decel)


def max_checked_speed(c: SafetyConfig) -> float:
    """Fastest horizontal speed at which both forward checks still work: something first seen at
    the edge of a sensor's range can be stopped for with the clearance kept."""
    budgets = [(c.obstacle_range_m - c.obstacle_close_m, c.obstacle_latency_s),
               (c.person_range_m / (1 + c.person_range_error) - c.person_close_m,
                c.person_latency_s)]
    lo, hi = 0.0, 50.0
    for _ in range(60):  # bisection: stopping_distance grows with v
        mid = (lo + hi) / 2
        if all(stopping_distance(mid, lat, c) <= room for room, lat in budgets):
            lo = mid
        else:
            hi = mid
    return lo


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
