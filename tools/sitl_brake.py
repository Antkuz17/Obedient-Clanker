"""Measure how far the drone travels after a `stop` at several speeds (ArduPilot SITL, through
the full edge stack). This is the number that decides whether the person / obstacle checks can
actually protect anything: the drone must stop inside what its sensors can see.

  docker run --rm -d --name hope-sitl-brake -p 5780:5760 hope-sitl
  .venv/bin/python -m tools.sitl_brake tcp:127.0.0.1:5780 1.0 1.5 2.0 3.0 [--acc=5 --jerk=10]

For each speed: set WP_SPD, fly forward, once at speed send `stop` (what a person / obstacle
detection does: hold the current position), then report reaction time (stop sent -> slowing
down), stopping distance (stop sent -> standstill) and the average deceleration.
"""
from __future__ import annotations

import asyncio
import math
import sys
import time

from edge.config import SafetyConfig, WatchdogConfig
from edge.mav import MavVehicle
from edge.server import EdgeServer
from shared import protocol as P
from shared.client import CommandRejected, EdgeClient


def hspeed(v) -> float:
    s = v.state()
    return math.hypot(s.vn, s.ve)


async def set_speed(v: MavVehicle, sp: float) -> None:
    """WP_SPD and the guided speed override (takeoff's DO_CHANGE_SPEED) both, else the FC
    flies at whichever is faster."""
    from edge.mav import M
    v.speed_xy = sp
    v.limit_param_sets[0]["WP_SPD"] = sp
    v.limit_param_sets[1]["WPNAV_SPEED"] = sp * 100
    v.limit_params = {}
    await asyncio.get_running_loop().run_in_executor(None, v._apply_limits)
    v._cmd(M.MAV_CMD_DO_CHANGE_SPEED, 1, sp, -1)


async def main(device: str, speeds: list[float]) -> int:
    cfg = SafetyConfig(flow_quality_min=0)
    acc = float(next((a.split("=")[1] for a in sys.argv if a.startswith("--acc=")), cfg.accel_xy))
    jerk = float(next((a.split("=")[1] for a in sys.argv if a.startswith("--jerk=")), cfg.jerk_xy))
    print(f"braking settings: WP_ACC={acc} m/s2, WP_JERK={jerk} m/s3", flush=True)
    v = MavVehicle(device, gps=True, accel_xy=acc, jerk_xy=jerk)
    srv = EdgeServer(v, cfg, WatchdogConfig(), host="127.0.0.1", port=0,
                     perception_port=None, flow_port=None)
    await srv.start()
    edge = await EdgeClient("127.0.0.1", srv.port).connect()
    t0 = time.monotonic()
    while not v.state().connected and time.monotonic() - t0 < 30:
        await asyncio.sleep(0.5)
    for attempt in range(15):  # SITL's EKF needs ~20 s and a GPS fix before it will arm
        try:
            d = await edge.run("takeoff", alt=2.0, timeout=40)
            if d["status"] == P.OK:
                break
            print("   takeoff:", d["status"], d["reason"], flush=True)
        except CommandRejected as e:
            print("   takeoff rejected:", e, flush=True)
        await asyncio.sleep(5)
    else:
        print("could not take off")
        return 1
    await asyncio.sleep(2)
    print(f"{'speed':>7} {'at stop':>8} {'react s':>8} {'stop m':>7} {'decel m/s2':>10}", flush=True)
    results = []
    for sp in speeds:
        await set_speed(v, sp)
        cid, ack = await edge.send("move_body", fwd=40.0)
        if not ack["ok"]:
            print("move refused:", ack["reason"])
            return 1
        t_start = time.monotonic()
        while hspeed(v) < 0.95 * sp and time.monotonic() - t_start < 30:
            await asyncio.sleep(0.02)
        await asyncio.sleep(0.5)  # steady
        s0 = v.state()
        v0 = hspeed(v)
        ux, uy = s0.vn / v0, s0.ve / v0           # direction of travel
        t_stop = time.monotonic()
        await edge.send("stop")
        t_react, dist, t_peak = None, 0.0, 0.0
        while time.monotonic() - t_stop < 8.0:   # sample the whole stop, overshoot included
            s = v.state()
            h = math.hypot(s.vn, s.ve)
            t = time.monotonic() - t_stop
            if t_react is None and h < 0.9 * v0:
                t_react = t
            along = (s.n - s0.n) * ux + (s.e - s0.e) * uy
            if along > dist:
                dist, t_peak = along, t
            await asyncio.sleep(0.02)
        decel = v0 ** 2 / (2 * dist) if dist > 0.01 else float("inf")
        print(f"{sp:7.1f} {v0:8.2f} {t_react or float('nan'):8.2f} {dist:7.2f} {decel:10.2f}"
              f"   (furthest point {t_peak:.1f} s after stop)", flush=True)
        results.append((sp, v0, t_react, dist))
        await asyncio.sleep(1.0)
    await edge.run("land", timeout=60)
    await edge.close()
    await srv.close()
    return 0


if __name__ == "__main__":
    dev = sys.argv[1] if len(sys.argv) > 1 else "tcp:127.0.0.1:5780"
    sp = [float(x) for x in sys.argv[2:] if not x.startswith("--")] or [1.0, 1.5, 2.0, 3.0]
    sys.exit(asyncio.run(main(dev, sp)))
