"""edge --demo: commands are judged by the real gate and refused; the vehicle never moves."""
import asyncio

import pytest

from edge import demo
from edge.config import SafetyConfig, WatchdogConfig
from edge.server import EdgeServer
from edge.sim import SimVehicle
from shared import protocol as P
from shared.client import CommandRejected, EdgeClient


@pytest.fixture
async def demo_rig():
    cfg = SafetyConfig()
    cfg.max_step = 2.0  # the real drone's slow-and-low cap
    sim = SimVehicle(cfg, speedup=8.0)
    srv = EdgeServer(sim, cfg, WatchdogConfig(), host="127.0.0.1", port=0,
                     perception_port=None, flow_port=None)
    srv.exec.demo = True
    await srv.start()
    edge = await EdgeClient("127.0.0.1", srv.port).connect()
    await asyncio.sleep(0.3)  # first telemetry
    yield srv, sim, edge
    await edge.close()
    await srv.close()


async def test_demo_refuses_flight_commands_and_never_moves(demo_rig):
    srv, sim, edge = demo_rig
    for op, args, words in (("takeoff", {"alt": 1.0}, "take off to 1 metre"),
                            ("move_body", {"fwd": -1.0}, "fly back 1 metre"),
                            ("yaw", {"deg": 180.0, "relative": True}, "turn around"),
                            ("scan", {"rate_dps": 30.0}, "full circle")):
        with pytest.raises(CommandRejected, match=words):
            await edge.run(op, **args)
    await asyncio.sleep(0.2)
    assert not sim.s.armed and sim.s.alt == 0.0 and (sim.s.n, sim.s.e) == (0.0, 0.0)


async def test_demo_real_gate_reasons_win(demo_rig):
    srv, sim, edge = demo_rig
    with pytest.raises(CommandRejected, match="exceeds the 2.0 m limit"):
        await edge.run("move_body", fwd=-3.0)
    sim.s.battery_v = 10.0
    with pytest.raises(CommandRejected, match="battery"):
        await edge.run("takeoff", alt=1.0)


async def test_demo_telemetry_shows_a_pretend_hover_and_land_still_works(demo_rig):
    srv, sim, edge = demo_rig
    assert not edge.telemetry["armed"]                      # real state until a takeoff
    with pytest.raises(CommandRejected, match="take off"):
        await edge.run("takeoff", alt=1.0)
    await asyncio.sleep(0.3)
    t = edge.telemetry
    assert t["armed"] and t["alt"] == pytest.approx(demo.DEMO_ALT_M) and t["mode"] == "GUIDED-DEMO"
    assert t["state"] == "HOVER"
    assert (await edge.run("stop"))["status"] == P.OK
    assert (await edge.run("land"))["status"] == P.OK      # on the ground: "already landed"
    await asyncio.sleep(0.3)
    assert not edge.telemetry["armed"] and not sim.s.armed  # pretend hover over


async def test_demo_takeoff_refused_for_a_real_reason_stays_down(demo_rig):
    srv, sim, edge = demo_rig
    sim.s.battery_v = 10.0
    with pytest.raises(CommandRejected, match="battery"):
        await edge.run("takeoff", alt=1.0)
    await asyncio.sleep(0.3)
    assert not edge.telemetry["armed"]


def test_describe():
    assert demo.describe("move_body", {"fwd": 2, "right": -1}) == "fly forward 2 metres and left 1 metre"
    assert demo.describe("yaw", {"deg": -90, "relative": True}) == "turn left 90 degrees"
    assert demo.describe("return_home", {}) == "fly back to where I took off"
