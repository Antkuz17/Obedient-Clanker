"""Edge limits. Tune at the venue; every number here is a safety decision."""
from __future__ import annotations

from dataclasses import dataclass


@dataclass
class SafetyConfig:
    alt_min: float = 0.3          # m; below this only takeoff/land
    alt_max: float = 1.8          # m; HC-SR04 tops out ~2 m
    fence_half: float = 3.0       # m; box +-fence_half N/E around home
    max_step: float = 3.0         # m; largest single move
    max_takeoff_alt: float = 1.5  # m
    speed_xy: float = 0.5         # m/s
    speed_z: float = 0.3          # m/s
    yaw_rate: float = 45.0        # deg/s
    scan_rate_max: float = 30.0   # deg/s
    batt_reject_v: float = 10.6   # 3S: refuse new moves below this
    batt_land_v: float = 10.2     # 3S: auto-land below this
    flow_quality_min: int = 50    # refuse moves when flow is this bad (0-255)
    person_close_m: float = 1.0   # refuse forward moves if a person is closer


@dataclass
class WatchdogConfig:
    hover_after_s: float = 1.0
    land_after_s: float = 5.0


@dataclass
class ExecConfig:
    pos_tolerance: float = 0.15   # m
    yaw_tolerance: float = 5.0    # deg
    cmd_timeout_pad_s: float = 8.0
