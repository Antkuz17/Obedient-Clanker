"""Vehicle interface. Implementations: SimVehicle (edge/sim.py), MavVehicle (edge/mav.py)."""
from __future__ import annotations

import math
from dataclasses import dataclass, asdict


@dataclass
class VehicleState:
    connected: bool = False
    armed: bool = False
    mode: str = "UNKNOWN"
    n: float = 0.0
    e: float = 0.0
    alt: float = 0.0        # m above takeoff point (positive up)
    yaw: float = 0.0        # deg, 0 = north, clockwise, [0, 360)
    vn: float = 0.0
    ve: float = 0.0
    vd: float = 0.0
    battery_v: float = 0.0
    flow_quality: int = 0
    range_m: float | None = None
    landed: bool = True

    def as_dict(self) -> dict:
        return asdict(self)


class Vehicle:
    """Minimal flight primitives. All targets are local NED positions + yaw (deg)."""

    def state(self) -> VehicleState:
        raise NotImplementedError

    async def start(self) -> None:
        pass

    async def close(self) -> None:
        pass

    def takeoff(self, alt: float) -> None:
        """Switch to GUIDED, arm, climb to alt."""
        raise NotImplementedError

    def set_target(self, n: float, e: float, alt: float, yaw: float) -> None:
        raise NotImplementedError

    def land(self) -> None:
        raise NotImplementedError

    def hold(self) -> None:
        s = self.state()
        self.set_target(s.n, s.e, s.alt, s.yaw)

    def send_optical_flow(self, flow_x: float, flow_y: float, quality: int, dt_us: int,
                          ground_m: float | None) -> None:
        """Forward a software optical-flow sample to the FC (no-op in sim)."""


def wrap360(deg: float) -> float:
    return deg % 360.0


def yaw_err(target: float, current: float) -> float:
    """Signed shortest difference target - current, in (-180, 180]."""
    d = (target - current + 180.0) % 360.0 - 180.0
    return 180.0 if d == -180.0 else d


def body_to_ned(fwd: float, right: float, yaw_deg: float) -> tuple[float, float]:
    y = math.radians(yaw_deg)
    return fwd * math.cos(y) - right * math.sin(y), fwd * math.sin(y) + right * math.cos(y)


def ned_to_body(dn: float, de: float, yaw_deg: float) -> tuple[float, float]:
    y = math.radians(yaw_deg)
    return dn * math.cos(y) + de * math.sin(y), -dn * math.sin(y) + de * math.cos(y)
