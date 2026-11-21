"""Review 2 repro (docs/review2.md, bugs 4-7) against the kinematic sim. No hardware, no Docker.

  .venv/bin/python -m tools.review.sim_exp
"""
import asyncio, time
from edge.config import SafetyConfig, WatchdogConfig
from edge.server import EdgeServer
from edge.sim import SimVehicle
from shared.client import CommandRejected, EdgeClient

async def rig(**kw):
    cfg = SafetyConfig(**kw)
    sim = SimVehicle(cfg, speedup=1.0)
    srv = EdgeServer(sim, cfg, WatchdogConfig(), host="127.0.0.1", port=0, perception_port=None, flow_port=None)
    await srv.start()
    edge = await EdgeClient("127.0.0.1", srv.port).connect()
    return cfg, sim, srv, edge

async def feed(srv, stop, person=None):
    while not stop.is_set():
        srv.on_perception({"person_m": person, "palm": False}); await asyncio.sleep(0.1)

async def main():
    print("== E6: vision process dies in the middle of a forward move (detector required)")
    cfg, sim, srv, edge = await rig(require_person_detector=True)
    stop = asyncio.Event(); t = asyncio.create_task(feed(srv, stop))
    await edge.run("takeoff", alt=1.0)
    cid, ack = await edge.send("move_body", fwd=2.5)
    await asyncio.sleep(0.5); n_at_death = sim.s.n
    stop.set(); await t                       # vision dies
    d = await edge.wait_done(cid, 30)
    print(f"   vision died at n={n_at_death:.2f}; move finished '{d['status']}' at n={sim.s.n:.2f} "
          f"(detector_alive={srv.exec.perception.detector_alive})")
    try:
        await edge.run("move_body", fwd=0.3)
    except CommandRejected as e:
        print("   next forward move:", e)
    await edge.close(); await srv.close()

    print("== E7: pilot takes over, moves the drone, hands back; then 'move forward 0.5'")
    cfg, sim, srv, edge = await rig()
    await edge.run("takeoff", alt=1.0)
    await edge.run("move_body", fwd=0.5)
    await asyncio.sleep(0.5)
    sim.s.mode = "ALT_HOLD"; sim._target = None
    sim.s.n, sim.s.e = -1.5, 1.0             # pilot flew it somewhere else
    await asyncio.sleep(0.3)
    sim.s.mode = "GUIDED"
    print(f"   handed back at n={sim.s.n:.2f} e={sim.s.e:.2f}; edge's remembered target={srv.exec.target}")
    d = await edge.run("move_body", fwd=0.5); await asyncio.sleep(0.5)
    print(f"   after 'fwd 0.5': n={sim.s.n:.2f} e={sim.s.e:.2f}  (expected n=-1.00 e=1.00)")
    await edge.close(); await srv.close()

    print("== E8: person close ahead; try to back away")
    cfg, sim, srv, edge = await rig()
    stop = asyncio.Event(); t = asyncio.create_task(feed(srv, stop, person=0.6))
    await edge.run("takeoff", alt=1.0)
    d = await edge.run("move_body", fwd=-1.0); await asyncio.sleep(0.3)
    print(f"   back 1 m with a person at 0.6 m: {d['status']} ({d['reason']}); moved to n={sim.s.n:.2f}")
    stop.set(); await t
    await edge.close(); await srv.close()

    print("== E9: station planner hangs forever, heartbeat thread keeps going")
    cfg, sim, srv, edge = await rig()
    await edge.run("takeoff", alt=1.0)
    time.sleep(7)                              # the station's event loop is frozen for 7 s
    await asyncio.sleep(0.2)
    print(f"   after a 7 s frozen station: failsafe={srv.exec.failsafe} armed={sim.s.armed} state={srv.exec.state}")
    await edge.close(); await srv.close()

asyncio.run(main())
