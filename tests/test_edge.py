import asyncio
import json
import socket
import time

import pytest

from edge.config import SafetyConfig, WatchdogConfig
from edge.safety import Perception, SafetyGate
from edge.server import EdgeServer
from edge.sim import SimVehicle
from edge.vehicle import VehicleState, body_to_ned, ned_to_body, yaw_err
from shared import protocol as P
from shared.client import CommandRejected, EdgeClient


# --- protocol -------------------------------------------------------------------------------
def test_validate_cmd_normalizes_and_rejects():
    assert P.validate_cmd(P.cmd("1", "move_body", fwd=1)) == ("1", "move_body", {"fwd": 1.0})
    for bad in [P.cmd("1", "flip"), P.cmd("1", "takeoff"), P.cmd("1", "move_body", up="x"),
                P.cmd("1", "move_body", sideways=1), P.cmd("", "hover"),
                P.cmd("1", "goto", n=float("nan"), e=0)]:
        with pytest.raises(P.ProtocolError):
            P.validate_cmd(bad)


def test_frames():
    assert body_to_ned(1, 0, 90) == pytest.approx((0, 1))
    assert ned_to_body(*body_to_ned(1.3, -0.4, 37), 37) == pytest.approx((1.3, -0.4))
    assert yaw_err(10, 350) == pytest.approx(20)
    assert yaw_err(350, 10) == pytest.approx(-20)


# --- safety gate ----------------------------------------------------------------------------
FLYING = VehicleState(connected=True, armed=True, mode="GUIDED", alt=1.0, battery_v=12.0,
                      flow_quality=200, landed=False)


@pytest.mark.parametrize("op,args,why", [
    ("move_body", {"up": 10.0}, "exceeds"),
    ("move_body", {"up": 1.5}, "altitude"),
    ("goto", {"n": 3.5, "e": 0.0}, "exceeds"),
    ("move_body", {"fwd": -3.5}, "exceeds"),
    ("yaw", {"deg": 90.0}, None),
    ("scan", {"rate_dps": 100.0}, "scan rate"),
])
def test_gate_limits(op, args, why):
    v = SafetyGate().check(op, args, FLYING, airborne=True)
    assert v.ok == (why is None)
    if why:
        assert why in v.reason


def test_gate_fence_battery_flow_person():
    g = SafetyGate()
    s = VehicleState(**{**FLYING.as_dict(), "n": 2.5})
    assert "geofence" in g.check("move_body", {"fwd": 1.0}, s, True).reason
    low = VehicleState(**{**FLYING.as_dict(), "battery_v": 10.4})
    assert "battery" in g.check("move_body", {"fwd": 0.5}, low, True).reason
    blind = VehicleState(**{**FLYING.as_dict(), "flow_quality": 10})
    assert "flow" in g.check("move_body", {"fwd": 0.5}, blind, True).reason
    near = Perception(person_m=0.6)
    assert "person" in g.check("move_body", {"fwd": 0.5}, FLYING, True, near).reason
    assert g.check("move_body", {"fwd": -0.5}, FLYING, True, near).ok  # backing off is fine
    assert g.check("land", {}, low, True).ok and g.check("stop", {}, blind, True).ok


def test_gate_relative_moves_build_on_commanded_target_not_measured():
    sagging = VehicleState(**{**FLYING.as_dict(), "alt": 0.9, "n": 0.1, "yaw": 2.0})
    v = SafetyGate().check("move_body", {"fwd": 1.0}, sagging, True, base=(0.0, 0.0, 1.0, 0.0))
    assert v.target == pytest.approx((1.0, 0.0, 1.0, 0.0))
    v = SafetyGate().check("yaw", {"deg": 90.0, "relative": True}, sagging, True,
                           base=(0.0, 0.0, 1.0, 0.0))
    assert v.target == pytest.approx((0.0, 0.0, 1.0, 90.0))


def test_gate_requires_live_detector_when_configured():
    g = SafetyGate(SafetyConfig(require_person_detector=True))
    assert "detector" in g.check("move_body", {"fwd": 0.5}, FLYING, True, Perception()).reason
    assert g.check("move_body", {"fwd": -0.5}, FLYING, True, Perception()).ok
    assert g.check("move_body", {"fwd": 0.5}, FLYING, True,
                   Perception(detector_alive=True)).ok
    assert SafetyGate().check("move_body", {"fwd": 0.5}, FLYING, True).ok  # default: off


def test_gate_needs_guided_and_position():
    g = SafetyGate()
    althold = VehicleState(**{**FLYING.as_dict(), "mode": "ALT_HOLD"})
    assert "pilot has control" in g.check("move_body", {"fwd": 0.5}, althold, True).reason
    lost = VehicleState(**{**FLYING.as_dict(), "pos_ok": False})
    assert "position" in g.check("move_body", {"fwd": 0.5}, lost, True).reason
    assert g.check("land", {}, lost, True).ok and g.check("stop", {}, althold, True).ok
    ground = VehicleState(connected=True, battery_v=12.4, flow_quality=200, pos_ok=False)
    assert "EKF" in g.check("takeoff", {"alt": 1.0}, ground, False).reason


def test_gate_fence_and_home_follow_takeoff_point():
    g = SafetyGate()
    s = VehicleState(**{**FLYING.as_dict(), "n": 5.0, "e": 5.0})
    assert g.check("move_body", {"fwd": 1.0}, s, True, home=(5.0, 5.0)).ok
    assert "geofence" in g.check("move_body", {"fwd": 1.0}, s, True).reason   # home (0,0)
    v = g.check("return_home", {}, VehicleState(**{**s.as_dict(), "n": 6.0}), True,
                home=(5.0, 5.0))
    assert v.target[:2] == (5.0, 5.0)


def test_gate_takeoff_rules():
    g = SafetyGate()
    ground = VehicleState(connected=True, battery_v=12.4, flow_quality=200)
    assert g.check("takeoff", {"alt": 1.0}, ground, False).ok
    assert not g.check("takeoff", {"alt": 3.0}, ground, False).ok
    assert "take off first" in g.check("move_body", {"fwd": 1.0}, ground, False).reason
    assert not g.check("takeoff", {"alt": 1.0}, FLYING, True).ok


# --- end to end over TCP against the sim ----------------------------------------------------
def free_port() -> int:
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


@pytest.fixture
async def rig():
    cfg = SafetyConfig()
    sim = SimVehicle(cfg, speedup=8.0)
    srv = EdgeServer(sim, cfg, WatchdogConfig(hover_after_s=0.4, land_after_s=1.2),
                     host="127.0.0.1", port=0, perception_port=free_port(), flow_port=None)
    await srv.start()
    edge = await EdgeClient("127.0.0.1", srv.port).connect()
    yield srv, sim, edge
    await edge.close()
    await srv.close()


async def test_takeoff_move_yaw_return_land(rig):
    srv, sim, edge = rig
    assert (await edge.run("takeoff", alt=1.0))["status"] == P.OK
    assert sim.s.alt == pytest.approx(1.0, abs=0.3)    # done fires within pos_tolerance
    await edge.run("yaw", deg=90.0)
    await edge.run("move_body", fwd=1.0)              # facing east -> +e
    assert (sim.s.n, sim.s.e) == pytest.approx((0, 1.0), abs=0.3)
    await asyncio.sleep(0.5)                          # ...and then settles on the exact target
    assert (sim.s.n, sim.s.e, sim.s.alt) == pytest.approx((0, 1.0, 1.0), abs=0.03)
    await edge.run("return_home")
    await asyncio.sleep(0.5)
    assert (sim.s.n, sim.s.e) == pytest.approx((0, 0), abs=0.03)
    assert edge.telemetry["state"] == "HOVER"
    await edge.run("land")
    assert not sim.s.armed


async def test_rejection_is_reported_with_reason(rig):
    srv, sim, edge = rig
    await edge.run("takeoff", alt=1.0)
    with pytest.raises(CommandRejected, match="exceeds"):
        await edge.run("move_body", up=10.0)
    await asyncio.sleep(0.05)
    assert any(e["kind"] == "safety_reject" for e in edge.events)


async def test_new_command_supersedes_and_stop_wins(rig):
    srv, sim, edge = rig
    await edge.run("takeoff", alt=1.0)
    c1, _ = await edge.send("move_body", fwd=2.5)
    await asyncio.sleep(0.2)
    c2, ack = await edge.send("stop")
    assert ack["ok"]
    d1 = await edge.wait_done(c1)
    assert d1["status"] == P.ABORTED and "stop" in d1["reason"]
    assert (await edge.wait_done(c2))["status"] == P.OK
    n = sim.s.n
    await asyncio.sleep(0.3)
    assert sim.s.n == pytest.approx(n, abs=0.05)      # holding, not drifting to the old target
    assert 0.05 < n < 2.5


async def test_watchdog_hovers_then_lands(rig):
    srv, sim, edge = rig
    await edge.run("takeoff", alt=1.0)
    c1, _ = await edge.send("move_body", fwd=2.5)
    edge.heartbeat = False                            # station "dies"
    d1 = await edge.wait_done(c1)
    assert d1["status"] == P.ABORTED and "failsafe" in d1["reason"]
    await asyncio.sleep(1.5)
    kinds = [e["detail"] for e in edge.events if e["kind"] == "watchdog"]
    assert any("holding" in k for k in kinds) and any("landing" in k for k in kinds)
    for _ in range(100):
        if not sim.s.armed:
            break
        await asyncio.sleep(0.05)
    assert not sim.s.armed and sim.s.alt == 0.0


async def test_watchdog_recovers_if_station_returns_before_land(rig):
    srv, sim, edge = rig
    await edge.run("takeoff", alt=1.0)
    edge.heartbeat = False
    await asyncio.sleep(0.7)
    assert srv.exec.failsafe == "hover"
    edge.heartbeat = True
    await asyncio.sleep(0.3)
    assert srv.exec.failsafe is None and sim.s.armed
    assert (await edge.run("move_body", fwd=0.5))["status"] == P.OK


async def test_palm_and_person_stop(rig):
    srv, sim, edge = rig
    await edge.run("takeoff", alt=1.0)
    c1, _ = await edge.send("move_body", fwd=2.5)
    await asyncio.sleep(0.1)
    srv.on_perception({"person_m": None, "palm": True})
    assert (await edge.wait_done(c1))["status"] == P.ABORTED
    srv.on_perception({"person_m": None, "palm": False})
    c2, _ = await edge.send("move_body", fwd=-1.0)
    await asyncio.sleep(0.1)
    srv.on_perception({"person_m": 0.5, "palm": False})
    assert (await edge.wait_done(c2))["status"] == P.ABORTED
    with pytest.raises(CommandRejected, match="person"):
        await edge.run("move_body", fwd=0.5)
    await asyncio.sleep(0.1)
    kinds = {e["kind"] for e in edge.events}
    assert {"palm_stop", "person_close"} <= kinds


async def test_perception_over_udp(rig):
    srv, sim, edge = rig
    loop = asyncio.get_running_loop()
    tr, _ = await loop.create_datagram_endpoint(
        asyncio.DatagramProtocol, remote_addr=("127.0.0.1", srv.perception_port))
    tr.sendto(json.dumps({"person_m": 0.4, "palm": False}).encode())
    await asyncio.sleep(0.1)
    tr.close()
    assert srv.exec.perception.person_m == pytest.approx(0.4)


async def test_low_battery_auto_lands(rig):
    srv, sim, edge = rig
    await edge.run("takeoff", alt=1.0)
    sim.s.battery_v = 10.0
    for _ in range(200):
        if not sim.s.armed:
            break
        await asyncio.sleep(0.05)
    assert not sim.s.armed
    assert any(e["kind"] == "battery" for e in edge.events)


async def test_scan_turns_full_circle(rig):
    srv, sim, edge = rig
    await edge.run("takeoff", alt=1.0)
    yaws = []
    cid, ack = await edge.send("scan", rate_dps=30.0)
    assert ack["ok"], ack
    while not edge.is_done(cid):
        yaws.append(sim.s.yaw)
        await asyncio.sleep(0.1)
    assert (await edge.wait_done(cid))["status"] == P.OK
    assert max(yaws) > 300 and min(yaws) < 60


async def test_land_during_takeoff_cancels_it(rig):
    srv, sim, edge = rig
    calls = []
    sim.cancel_takeoff = lambda: calls.append("cancel")
    c1, _ = await edge.send("takeoff", alt=1.0)
    c2, ack = await edge.send("land")
    assert ack["ok"] and calls                       # review A1: the takeoff is told to stop
    assert (await edge.wait_done(c1))["status"] == P.ABORTED


async def test_land_that_never_disarms_is_reported_failed(rig, monkeypatch):
    srv, sim, edge = rig
    await edge.run("takeoff", alt=1.0)
    sim.land = lambda: None                           # FC ignores LAND
    monkeypatch.setattr(srv.exec, "_wait", _instant_false)
    d = await edge.run("land")
    assert d["status"] == P.FAILED and "still armed" in d["reason"]   # review A2


async def _instant_false(pred, timeout):
    return False


async def test_watchdog_does_not_override_pilot_mode(rig):
    srv, sim, edge = rig
    await edge.run("takeoff", alt=1.0)
    sim.s.mode = "ALT_HOLD"                           # pilot flipped the switch
    edge.heartbeat = False
    await asyncio.sleep(1.6)
    assert sim.s.armed and srv.exec.failsafe is None  # edge stays out of the pilot's way


async def test_position_loss_in_flight_lands(rig):
    srv, sim, edge = rig
    await edge.run("takeoff", alt=1.0)
    sim.s.pos_ok = False
    for _ in range(200):
        if not sim.s.armed:
            break
        await asyncio.sleep(0.05)
    assert not sim.s.armed
    assert any(e["kind"] == "position" for e in edge.events)


async def test_client_heartbeat_survives_a_blocked_event_loop():
    """Review A10: a slow LLM/STT call that blocks the station's event loop must not starve
    the heartbeat (1 s gap = edge hovers, 5 s = lands)."""
    import socket as so
    import threading
    srv = so.socket()
    srv.bind(("127.0.0.1", 0))
    srv.listen(1)
    beats = []

    def accept():
        c, _ = srv.accept()
        for line in c.makefile("rb"):
            if b"heartbeat" in line:
                beats.append(time.monotonic())
    threading.Thread(target=accept, daemon=True).start()
    edge = await EdgeClient("127.0.0.1", srv.getsockname()[1]).connect()
    await asyncio.sleep(0.3)
    t0 = time.monotonic()
    time.sleep(1.5)                                   # block the event loop
    during = [b for b in beats if t0 < b < t0 + 1.5]
    await edge.close()
    srv.close()
    assert len(during) >= 6                           # ~5 Hz kept going
    assert max(b - a for a, b in zip(during, during[1:])) < 0.5


async def test_station_allowlist_refuses_other_ips():
    cfg = SafetyConfig()
    srv = EdgeServer(SimVehicle(cfg), cfg, host="127.0.0.1", port=0, perception_port=None,
                     flow_port=None, allowed_peers={"10.9.9.9"})
    await srv.start()
    edge = await EdgeClient("127.0.0.1", srv.port).connect()
    await asyncio.sleep(0.3)
    assert not edge.connected and srv._writer is None
    await edge.close()
    await srv.close()


async def test_bad_message_gets_error_ack(rig):
    srv, sim, edge = rig
    edge._send({"type": "cmd", "id": "x1", "op": "barrel_roll", "args": {}})
    fut = asyncio.get_running_loop().create_future()
    edge._acks["x1"] = fut
    ack = await asyncio.wait_for(fut, 2)
    assert not ack["ok"] and "unknown op" in ack["reason"]
