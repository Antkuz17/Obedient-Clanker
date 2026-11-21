"""Scripted flight against ArduPilot SITL through the full edge stack (MavVehicle + server).

  docker build -t hope-sitl tools/sitl && docker run --rm -d --name hope-sitl -p 5760:5760 hope-sitl
  .venv/bin/python -m tools.sitl_flight            # exits 0 if every step passed
"""
from __future__ import annotations

import asyncio
import logging
import math
import sys
import time

from edge.config import SafetyConfig, WatchdogConfig
from edge.mav import MavVehicle
from edge.server import EdgeServer
from shared import protocol as P
from shared.client import CommandRejected, EdgeClient

ok_all = True


def check(name: str, cond: bool, info: str = "") -> None:
    global ok_all
    ok_all &= bool(cond)
    print(f"[{'PASS' if cond else 'FAIL'}] {name} {info}", flush=True)


ALT_TOL = 0.15
POS_TOL = 0.35  # executor reports done within pos_tolerance (0.3 m)


def alt_ok(v) -> bool:
    return abs(v.state().alt - 1.0) < ALT_TOL


def pos(v):
    s = v.state()
    return f"n={s.n:.2f} e={s.e:.2f} alt={s.alt:.2f} yaw={s.yaw:.0f} mode={s.mode}"


async def main(device: str) -> int:
    cfg = SafetyConfig(flow_quality_min=0)
    v = MavVehicle(device, speed_xy=cfg.speed_xy)
    srv = EdgeServer(v, cfg, WatchdogConfig(), host="127.0.0.1", port=0,
                     perception_port=None, flow_port=None)
    await srv.start()
    edge = await EdgeClient("127.0.0.1", srv.port,
                            on_event=lambda e: print("   event:", e["kind"], e["detail"])
                            ).connect()

    t0 = time.monotonic()
    while not v.state().connected and time.monotonic() - t0 < 30:
        await asyncio.sleep(0.5)
    check("FC heartbeat", v.state().connected)

    # the EKF needs GPS lock in SITL before arming; retry takeoff until it goes
    took_off = False
    for attempt in range(12):
        try:
            d = await edge.run("takeoff", alt=1.0, timeout=30)
        except CommandRejected as e:
            print("   takeoff rejected:", e)
            await asyncio.sleep(5)
            continue
        if d["status"] == P.OK:
            took_off = True
            break
        print(f"   takeoff attempt {attempt + 1}: {d['status']} {d['reason']} ({pos(v)})")
        await asyncio.sleep(5)
    await asyncio.sleep(2)  # settle on the commanded altitude
    check("takeoff to 1.0 m", took_off and alt_ok(v), pos(v))
    if not took_off:
        return 1

    n0, e0 = v.state().n, v.state().e
    d = await edge.run("move_body", fwd=1.0, timeout=40)
    s = v.state()
    check("move_body fwd 1 m", d["status"] == P.OK and abs(math.dist((s.n, s.e), (n0, e0)) - 1)
          < POS_TOL and alt_ok(v), f"{d['status']} {pos(v)}")

    d = await edge.run("yaw", deg=90.0, timeout=30)
    check("yaw to 90", d["status"] == P.OK and abs(((v.state().yaw - 90 + 180) % 360) - 180)
          < 8 and alt_ok(v), f"{d['status']} {pos(v)}")

    n1, e1 = v.state().n, v.state().e
    d = await edge.run("move_body", right=1.0, timeout=40)   # facing east: right = south
    s = v.state()
    check("move_body right 1 m while facing east", d["status"] == P.OK
          and abs(s.n - (n1 - 1.0)) < POS_TOL and abs(s.e - e1) < POS_TOL and alt_ok(v),
          f"{d['status']} {pos(v)}")

    try:
        await edge.run("move_body", up=5.0)
        check("gate rejects up 5 m", False)
    except CommandRejected as e:
        check("gate rejects up 5 m", True, str(e))

    d = await edge.run("return_home", timeout=40)
    s = v.state()
    home = srv.exec.home
    check("return_home (to the takeoff point)", d["status"] == P.OK
          and math.hypot(s.n - home[0], s.e - home[1]) < POS_TOL and alt_ok(v),
          f"{d['status']} {pos(v)}")

    # watchdog: station goes silent -> hover at 1 s -> land at 5 s
    edge.heartbeat = False
    t0 = time.monotonic()
    while v.state().armed and time.monotonic() - t0 < 30:
        await asyncio.sleep(0.2)
    kinds = [e["detail"] for e in edge.events if e["kind"] == "watchdog"]
    check("watchdog hovers then lands + disarms", not v.state().armed
          and any("holding" in k for k in kinds) and any("landing" in k for k in kinds),
          f"after {time.monotonic() - t0:.1f} s, {pos(v)}")

    # A3: a hung edge must make the FC land on its own (FS_GCS_ENABLE=5 Land)
    for name, val in ((b"FS_GCS_ENABLE", 5), (b"FS_GCS_TIMEOUT", 2)):
        v._send(v.m.mav.param_set_send, v.m.target_system, v.m.target_component, name, val,
                9)  # MAV_PARAM_TYPE_REAL32 is what ArduPilot expects over the wire
    edge.heartbeat = True
    await asyncio.sleep(1.0)
    d = await edge.run("takeoff", alt=1.0, timeout=40)
    check("re-takeoff after watchdog landing", d["status"] == P.OK, f"{d['status']} {d['reason']} {pos(v)}")
    await asyncio.sleep(1.0)
    print("   freezing the edge event loop for 6 s (simulated hung core)...", flush=True)
    time.sleep(6.0)  # blocks the event loop: no safety_tick, so no GCS heartbeat to the FC
    st = v.state()
    check("FC's own GCS failsafe took over (mode LAND) while the edge was hung",
          st.mode == "LAND", pos(v))
    t0 = time.monotonic()
    while v.state().armed and time.monotonic() - t0 < 30:
        await asyncio.sleep(0.2)
    check("...and landed + disarmed", not v.state().armed, pos(v))

    await edge.close()
    await srv.close()
    print("ALL PASS" if ok_all else "SOME FAILED")
    return 0 if ok_all else 1


if __name__ == "__main__":
    logging.basicConfig(level=logging.WARNING)
    sys.exit(asyncio.run(main(sys.argv[1] if len(sys.argv) > 1 else "tcp:127.0.0.1:5760")))
