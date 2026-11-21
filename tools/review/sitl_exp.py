"""Review 2 repro (docs/review2.md, bugs 1-3) against a PRIVATE ArduPilot SITL on port 5770.

  docker run --rm -d --name hope-sitl-review -p 5770:5760 hope-sitl
  .venv/bin/python -m tools.review.sitl_exp
  docker rm -f hope-sitl-review
"""
import asyncio, math, sys, time, logging
from edge.config import SafetyConfig, WatchdogConfig
from edge.mav import MavVehicle
from edge.server import EdgeServer
from shared import protocol as P
from shared.client import CommandRejected, EdgeClient

DEV = "tcp:127.0.0.1:5770"


async def sample(v, secs, hz=20):
    out = []
    t0 = time.monotonic()
    while time.monotonic() - t0 < secs:
        s = v.state()
        out.append((time.monotonic() - t0, s.n, s.e, s.alt, s.vn, s.ve, s.vd, s.mode, s.armed))
        await asyncio.sleep(1 / hz)
    return out


async def main():
    cfg = SafetyConfig(flow_quality_min=0)
    v = MavVehicle(DEV, speed_xy=cfg.speed_xy)
    srv = EdgeServer(v, cfg, WatchdogConfig(), host="127.0.0.1", port=0,
                     perception_port=None, flow_port=None)
    await srv.start()
    edge = await EdgeClient("127.0.0.1", srv.port,
                            on_event=lambda e: print("   event:", e["kind"], e["detail"])).connect()
    t0 = time.monotonic()
    while not v.state().connected and time.monotonic() - t0 < 60:
        await asyncio.sleep(0.5)
    print("connected", v.state().connected)

    # ---- E3: stop right after arming (before lift-off) ---------------------------------
    print("\n== E3: 'stop' sent just after the drone arms, before lift-off")
    for attempt in range(20):
        cid, ack = await edge.send("takeoff", alt=1.0)
        if not ack["ok"]:
            print("   takeoff rejected:", ack["reason"]); await asyncio.sleep(5); continue
        t0 = time.monotonic()
        while not v.state().armed and time.monotonic() - t0 < 8:
            await asyncio.sleep(0.02)
        if v.state().armed:
            break
        d = await edge.wait_done(cid, 30)
        print("   attempt", attempt + 1, d["status"], d["reason"]); await asyncio.sleep(5)
    await asyncio.sleep(0.4)   # the takeoff thread has now sent NAV_TAKEOFF
    s = v.state()
    print(f"   at stop: armed={s.armed} alt={s.alt:.2f} landed={s.landed} state={srv.exec.state}")
    cid2, ack2 = await edge.send("stop")
    d1 = await edge.wait_done(cid, 10)
    d2 = await edge.wait_done(cid2, 10)
    print("   takeoff done:", d1["status"], d1["reason"], "| stop:", ack2["ok"], d2["status"])
    tr = await sample(v, 10)
    print(f"   10 s after stop: alt max={max(x[3] for x in tr):.2f} final={tr[-1][3]:.2f} "
          f"armed={tr[-1][8]} mode={tr[-1][7]} edge state={srv.exec.state}")
    print(f"   max climb rate after stop: {max(-x[6] for x in tr):.2f} m/s")

    if not v.state().armed or v.state().alt < 0.5:
        print("   (drone not up; taking off normally for the next experiments)")
        if v.state().armed:
            await edge.run("land", timeout=40)
        task = asyncio.create_task(sample(v, 12))
        d = await edge.run("takeoff", alt=1.0, timeout=40)
        print("   takeoff:", d)
        tr = await task
        print(f"   normal takeoff max climb rate: {max(-x[6] for x in tr):.2f} m/s, "
              f"max alt {max(x[3] for x in tr):.2f}")
    await asyncio.sleep(3)

    # ---- E2: is the 0.5 m/s speed limit real? ------------------------------------------
    print("\n== E2: horizontal speed during move_body fwd=2.5 (limit in config: 0.5 m/s)")
    task = asyncio.create_task(sample(v, 14))
    try:
        d = await edge.run("move_body", fwd=2.5, timeout=40)
        print("   done:", d["status"], d["reason"])
    except CommandRejected as e:
        print("   rejected:", e)
    tr = await task
    sp = [math.hypot(x[4], x[5]) for x in tr]
    print(f"   max horizontal speed {max(sp):.2f} m/s; alt range {min(x[3] for x in tr):.2f}"
          f"-{max(x[3] for x in tr):.2f}")
    task = asyncio.create_task(sample(v, 8))
    try:
        d = await edge.run("move_body", up=0.4, timeout=30)
        print("   up 0.4 done:", d["status"], d["reason"])
    except CommandRejected as e:
        print("   up rejected:", e)
    tr = await task
    print(f"   max climb rate on move up: {max(-x[6] for x in tr):.2f} m/s (limit 0.3)")

    # ---- E1: after 'land', can the pilot take the drone back? --------------------------
    print("\n== E1: land, then the 'pilot' switches to ALT_HOLD (mode 2)")
    # hold throttle mid-stick like a pilot would, so ALT_HOLD holds height
    def rc():
        v._send(v.m.mav.rc_channels_override_send, v.m.target_system, v.m.target_component,
                1500, 1500, 1500, 1500, 0, 0, 0, 0)
    cid, ack = await edge.send("land")
    t0 = time.monotonic()
    while v.state().mode != "LAND" and time.monotonic() - t0 < 5:
        await asyncio.sleep(0.05)
    print("   mode after land cmd:", v.state().mode, f"alt={v.state().alt:.2f}")
    rc(); v._set_mode(2)
    seq = []
    t0 = time.monotonic()
    while time.monotonic() - t0 < 4:
        rc()
        m = v.state().mode
        if not seq or seq[-1][1] != m:
            seq.append((round(time.monotonic() - t0, 2), m))
        await asyncio.sleep(0.05)
    print("   mode timeline after the pilot's switch:", seq)
    print("   edge _land_req still set:", v._land_req)
    t0 = time.monotonic()
    while v.state().armed and time.monotonic() - t0 < 30:
        await asyncio.sleep(0.2)
    print("   disarmed:", not v.state().armed)
    await edge.close(); await srv.close()


logging.basicConfig(level=logging.WARNING)
asyncio.run(main())
