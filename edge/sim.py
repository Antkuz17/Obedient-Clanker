"""Kinematic sim vehicle. Good enough to develop the station against; not a physics sim.

Use ArduPilot SITL (tools/sitl.md) for anything flight-dynamics related.
"""
from __future__ import annotations

import asyncio
import math
import time

from .config import SafetyConfig
from .vehicle import Vehicle, VehicleState, wrap360, yaw_err


class SimVehicle(Vehicle):
    def __init__(self, cfg: SafetyConfig | None = None, rate_hz: float = 50.0,
                 speedup: float = 1.0):
        self.cfg = cfg or SafetyConfig()
        self.dt = 1.0 / rate_hz
        self.speedup = speedup
        self.s = VehicleState(connected=True, mode="GUIDED", battery_v=12.5,
                              flow_quality=200, range_m=0.0)
        self._target: tuple[float, float, float, float] | None = None
        self._landing = False
        self._task: asyncio.Task | None = None
        # fault injection for tests/demos
        self.drain_v_per_s = 0.002

    def state(self) -> VehicleState:
        return VehicleState(**self.s.as_dict())

    def send_distance_sensor(self, range_m: float) -> None:
        self.s.range_m = None if range_m < 0 else range_m

    async def start(self) -> None:
        self._task = asyncio.create_task(self._run())

    async def close(self) -> None:
        if self._task:
            self._task.cancel()

    async def _run(self) -> None:
        while True:
            self.step(self.dt * self.speedup)
            await asyncio.sleep(self.dt)

    # --- primitives --------------------------------------------------------------------------
    def takeoff(self, alt: float) -> None:
        self.s.armed = True
        self.s.landed = False
        self._landing = False
        self._target = (self.s.n, self.s.e, alt, self.s.yaw)

    def set_target(self, n: float, e: float, alt: float, yaw: float) -> None:
        if not self.s.armed or self._landing:
            return
        self._target = (n, e, alt, wrap360(yaw))

    def land(self) -> None:
        if self.s.armed:
            self._landing = True
            self._target = (self.s.n, self.s.e, 0.0, self.s.yaw)

    # --- physics -----------------------------------------------------------------------------
    def step(self, dt: float) -> None:
        s = self.s
        if s.armed:
            s.battery_v = max(9.0, s.battery_v - self.drain_v_per_s * dt)
        if not s.armed or self._target is None:
            s.vn = s.ve = s.vd = 0.0
            return
        tn, te, talt, tyaw = self._target
        dn, de, dz = tn - s.n, te - s.e, talt - s.alt
        dxy = math.hypot(dn, de)
        vxy = min(self.cfg.speed_xy, dxy / dt) if dxy > 1e-6 else 0.0
        vz_max = self.cfg.speed_z * (1.5 if self._landing else 1.0)
        vz = max(-vz_max, min(vz_max, dz / dt))
        s.vn = vxy * dn / dxy if dxy > 1e-6 else 0.0
        s.ve = vxy * de / dxy if dxy > 1e-6 else 0.0
        s.vd = -vz
        s.n += s.vn * dt
        s.e += s.ve * dt
        s.alt = max(0.0, s.alt + vz * dt)
        dy = yaw_err(tyaw, s.yaw)
        step = max(-self.cfg.yaw_rate * dt, min(self.cfg.yaw_rate * dt, dy))
        s.yaw = wrap360(s.yaw + step)
        s.range_m = s.alt
        if self._landing and s.alt <= 0.02:
            s.alt = 0.0
            s.armed = False
            s.landed = True
            self._landing = False
            self._target = None
            s.vn = s.ve = s.vd = 0.0


def sim_clock() -> float:
    return time.monotonic()
