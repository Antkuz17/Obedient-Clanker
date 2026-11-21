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


async def test_ack_arriving_before_send_returns(rig):
    """The ack can be dispatched while send() is still in the executor (SITL KeyError 'c4')."""
    srv, sim, edge = rig
    real = edge._send
    def slow_send(msg):
        real(msg)
        time.sleep(0.2)  # the loop is free meanwhile: the ack arrives and _dispatch pops it
    edge._send = slow_send
    assert (await edge.run("takeoff", alt=1.0))["status"] == P.OK


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
    # a person walks in during a forward move -> it stops
    c2, _ = await edge.send("move_body", fwd=1.0)
    await asyncio.sleep(0.1)
    srv.on_perception({"person_m": 0.5, "palm": False})
    assert (await edge.wait_done(c2))["status"] == P.ABORTED
    with pytest.raises(CommandRejected, match="person"):
        await edge.run("move_body", fwd=0.5)
    # ...but backing away from them is allowed and is NOT aborted (review2 #6)
    n0 = sim.s.n
    c3, ack = await edge.send("move_body", fwd=-1.0)
    assert ack["ok"]
    for _ in range(3):
        srv.on_perception({"person_m": 0.5, "palm": False})
        await asyncio.sleep(0.1)
    assert (await edge.wait_done(c3))["status"] == P.OK
    assert sim.s.n == pytest.approx(n0 - 1.0, abs=0.3)
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


async def test_hung_planner_stops_the_heartbeat():
    """review2 #7: a 1 s block is fine, but a planner hung for good must stop heartbeating."""
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
    edge = await EdgeClient("127.0.0.1", srv.getsockname()[1], liveness_s=1.0).connect()
    await asyncio.sleep(0.3)
    t0 = time.monotonic()
    time.sleep(3.0)                                   # hung for 3 s
    late = [b for b in beats if b > t0 + 1.5]
    await edge.close()
    srv.close()
    assert not late                                   # beats stopped once liveness expired


def test_edge_refuses_real_drone_without_station_ip():
    import subprocess
    import sys
    r = subprocess.run([sys.executable, "-m", "edge", "--mav", "udpin:127.0.0.1:14599"],
                       capture_output=True, text=True, timeout=30)
    assert r.returncode != 0 and "--station-ip" in r.stderr


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


async def test_dead_detector_stops_a_running_forward_move():
    """review2 #4: the detector dying mid-move must stop a forward move, not just new ones."""
    cfg = SafetyConfig(require_person_detector=True)
    sim = SimVehicle(cfg, speedup=1.0)  # real time: the 1 s stale window must elapse mid-move
    srv = EdgeServer(sim, cfg, host="127.0.0.1", port=0, perception_port=None, flow_port=None)
    await srv.start()
    edge = await EdgeClient("127.0.0.1", srv.port).connect()
    alive = True

    async def vision():
        while alive:
            srv.on_perception({"person_m": None, "palm": False})
            await asyncio.sleep(0.1)
    t = asyncio.create_task(vision())
    await edge.run("takeoff", alt=1.0)
    cid, ack = await edge.send("move_body", fwd=2.5)
    assert ack["ok"]
    await asyncio.sleep(0.3)
    alive = False
    await t
    d = await edge.wait_done(cid, 10)
    assert d["status"] == P.ABORTED and "detector" in d["reason"]
    assert sim.s.n < 1.5
    await edge.close()
    await srv.close()


async def test_flow_loss_stops_a_running_move(rig):
    srv, sim, edge = rig
    srv.exec.FLOW_BAD_S = 0.1  # the sim runs 8x fast; the move would finish within 0.5 s
    await edge.run("takeoff", alt=1.0)
    cid, _ = await edge.send("move_body", fwd=-2.5)
    await asyncio.sleep(0.1)
    sim.s.flow_quality = 0
    d = await edge.wait_done(cid, 10)
    assert d["status"] == P.ABORTED and "flow" in d["reason"]


async def test_brief_flow_dip_does_not_abort_a_move(rig):
    """review3: one blurry frame (flow quality 0 for < FLOW_BAD_S) must not end the move."""
    srv, sim, edge = rig
    await edge.run("takeoff", alt=1.0)
    cid, _ = await edge.send("move_body", fwd=-2.5)
    await asyncio.sleep(0.1)
    sim.s.flow_quality = 0
    await asyncio.sleep(0.15)
    sim.s.flow_quality = 200
    d = await edge.wait_done(cid, 10)
    assert d["status"] == P.OK


async def test_pilot_takeover_resets_target_and_cancels(rig):
    """review2 #5: after the pilot flies it elsewhere and hands back, relative moves start
    from where the drone actually is."""
    srv, sim, edge = rig
    await edge.run("takeoff", alt=1.0)
    await edge.run("move_body", fwd=0.5)
    sim.s.mode = "ALT_HOLD"
    sim._target = None
    sim.s.n, sim.s.e = -1.5, 1.0
    await asyncio.sleep(0.2)
    assert srv.exec.target is None and srv.exec.pilot_mode == "ALT_HOLD"
    with pytest.raises(CommandRejected, match="pilot has control"):
        await edge.run("move_body", fwd=0.5)
    sim.s.mode = "GUIDED"
    await asyncio.sleep(0.1)
    await edge.run("move_body", fwd=0.5)
    await asyncio.sleep(0.3)
    assert (sim.s.n, sim.s.e) == pytest.approx((-1.0, 1.0), abs=0.05)


async def test_stop_while_armed_on_ground_disarms(rig):
    """review2 #3: 'stop' right after arming must not let the takeoff continue."""
    srv, sim, edge = rig
    sim.takeoff = lambda alt: setattr(sim.s, "armed", True)   # armed, not yet climbing
    sim.s.landed = True
    c1, _ = await edge.send("takeoff", alt=1.0)
    await asyncio.sleep(0.1)
    c2, ack = await edge.send("stop")
    assert ack["ok"]
    d = await edge.wait_done(c2, 10)
    assert d["status"] == P.OK and "disarmed" in d["reason"] and not sim.s.armed


async def test_battery_sag_is_debounced(rig):
    srv, sim, edge = rig
    await edge.run("takeoff", alt=1.0)
    sim.s.battery_v = 10.0
    await asyncio.sleep(0.5)
    sim.s.battery_v = 11.5                            # throttle sag recovered
    await asyncio.sleep(2.5)
    assert sim.s.armed and srv.exec.failsafe is None  # review2 #12


async def test_bad_message_gets_error_ack(rig):
    srv, sim, edge = rig
    edge._send({"type": "cmd", "id": "x1", "op": "barrel_roll", "args": {}})
    fut = asyncio.get_running_loop().create_future()
    edge._acks["x1"] = fut
    ack = await asyncio.wait_for(fut, 2)
    assert not ack["ok"] and "unknown op" in ack["reason"]


def test_move_tolerance_scales_for_short_moves():
    from edge.config import ExecConfig
    from edge.executor import Executor
    ex = Executor.__new__(Executor)
    ex.cfg = ExecConfig()
    assert ex.move_tolerance(0.5) == pytest.approx(0.15)   # was 0.3: done at +0.24 m
    assert ex.move_tolerance(0.1) == pytest.approx(0.1)    # floor
    assert ex.move_tolerance(1.0) == pytest.approx(0.3)    # >= 1 m: unchanged
    assert ex.move_tolerance(3.0) == pytest.approx(0.3)


async def test_short_move_goes_most_of_the_way(rig):
    srv, sim, edge = rig
    await edge.run("takeoff", alt=1.0)
    await asyncio.sleep(0.5)
    n0 = sim.s.n
    assert (await edge.run("move_body", fwd=0.5))["status"] == P.OK
    assert sim.s.n - n0 >= 0.35


# --- review3 ---------------------------------------------------------------------------------
async def test_fence_stays_at_the_first_takeoff_point(rig):
    """review3 #2: a second takeoff used to re-centre the fence wherever the drone landed."""
    srv, sim, edge = rig
    await edge.run("takeoff", alt=1.0)
    await edge.run("move_body", fwd=2.5)
    await edge.run("land")
    await edge.run("takeoff", alt=1.0)                  # 2.5 m from home: inside, allowed
    assert srv.exec.home == pytest.approx((0.0, 0.0), abs=0.05)
    with pytest.raises(CommandRejected, match="geofence"):
        await edge.run("move_body", fwd=1.0)           # 3.5 m from the first takeoff point
    await edge.run("land")
    sim.s.n = 4.0                                      # carried outside the area
    with pytest.raises(CommandRejected, match="flying area"):
        await edge.run("takeoff", alt=1.0)


async def test_station_land_does_not_override_the_pilot(rig):
    """review3 #7: with the pilot in AltHold, a station/dashboard LAND used to take over."""
    srv, sim, edge = rig
    await edge.run("takeoff", alt=1.0)
    sim.s.mode = "ALT_HOLD"
    await asyncio.sleep(0.1)
    for op in ("land", "stop"):
        with pytest.raises(CommandRejected, match="pilot has control"):
            await edge.run(op)
    assert sim.s.armed and not sim._landing
    sim.s.mode = "GUIDED"                              # pilot hands back: land works again
    await asyncio.sleep(0.1)
    assert (await edge.run("land"))["status"] == P.OK


async def test_safety_loop_survives_an_exception(rig, monkeypatch):
    """review3 #6: one exception used to end the 50 Hz loop and, with it, the watchdog."""
    srv, sim, edge = rig
    await edge.run("takeoff", alt=1.0)
    orig, n = sim.tick, {"calls": 0}

    def flaky(now):
        n["calls"] += 1
        if n["calls"] == 1:
            raise OSError("simulated USB write failure")
        return orig(now)
    monkeypatch.setattr(sim, "tick", flaky)
    await asyncio.sleep(0.2)
    assert not srv._tasks[1].done() and n["calls"] > 3
    edge.heartbeat = False
    for _ in range(100):
        if not sim.s.armed:
            break
        await asyncio.sleep(0.05)
    assert not sim.s.armed                             # the watchdog still landed it


async def test_persistently_failing_safety_tick_lands(rig, monkeypatch):
    srv, sim, edge = rig
    await edge.run("takeoff", alt=1.0)

    def broken(now, ex, s):
        raise RuntimeError("bug in a safety check")
    monkeypatch.setattr(srv, "_safety_checks", broken)
    for _ in range(100):
        if not sim.s.armed:
            break
        await asyncio.sleep(0.05)
    assert not sim.s.armed


def test_pets_only_after_a_complete_safety_tick(monkeypatch):
    """hbwd is petted last: a tick that dies part-way must not tell the C watchdog all is well."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("127.0.0.1", 0))
    sock.settimeout(0.2)
    srv = EdgeServer(SimVehicle(), perception_port=None, flow_port=None,
                     hbwd_addr=sock.getsockname())
    srv.safety_tick(time.monotonic())
    assert sock.recv(64).startswith(b"PET 1 ")

    def broken(now, ex, s):
        raise RuntimeError("bug")
    monkeypatch.setattr(srv, "_safety_checks", broken)
    with pytest.raises(RuntimeError):
        srv.safety_tick(time.monotonic())
    with pytest.raises(socket.timeout):
        sock.recv(64)


def test_direct_flow_is_not_forwarded_again():
    """review3 #8: with --fc the flow process already sent OPTICAL_FLOW to the FC itself."""
    class V(SimVehicle):
        sent, noted = [], []
        def send_optical_flow(self, *a):
            self.sent.append(a)
        def note_flow_quality(self, q):
            self.noted.append(q)
        def send_distance_sensor(self, r):
            self.sent.append(r)
    v = V()
    srv = EdgeServer(v, perception_port=None, flow_port=None)
    srv._on_flow(json.dumps({"fx": 0.1, "fy": 0.0, "q": 180, "dt_us": 33000}).encode())
    srv._on_flow(json.dumps({"fx": 0.1, "fy": 0.0, "q": 170, "dt_us": 33000,
                             "direct": True}).encode())
    srv._on_flow(json.dumps({"range_m": 0.5, "direct": True}).encode())
    assert len(v.sent) == 1 and v.noted == [170]


def test_fcdirect_frames_decode_as_mavlink2():
    from pymavlink.dialects.v20 import ardupilotmega as mav2
    from edge.fcdirect import FcDirect
    rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rx.bind(("127.0.0.1", 0))
    rx.settimeout(1.0)
    fc = FcDirect(f"127.0.0.1:{rx.getsockname()[1]}", component=193)
    parser = mav2.MAVLink(None)
    fc.optical_flow(0.25, -0.5, 200)
    m = parser.parse_char(rx.recv(300))
    assert m.get_type() == "OPTICAL_FLOW" and m.quality == 200
    assert m.flow_rate_x == pytest.approx(0.25) and m.flow_rate_y == pytest.approx(-0.5)
    assert m.get_srcSystem() == 255 and m.get_srcComponent() == 193
    fc.distance(0.42)
    d = parser.parse_char(rx.recv(300))
    assert d.get_type() == "DISTANCE_SENSOR" and d.current_distance == 42
    fc.distance(-1)
    assert parser.parse_char(rx.recv(300)).current_distance == 201  # no echo = out of range


def _mav_vehicle():
    """A MavVehicle with no link, for checking the height reference logic."""
    from edge.mav import MavVehicle

    class Rec:
        sent = []
        def set_position_target_local_ned_send(self, *a):
            self.sent.append(a)
    v = MavVehicle("udpin:127.0.0.1:0")
    v.m = type("M", (), {"mav": Rec(), "target_system": 1, "target_component": 1})()
    return v


class _Msg:
    def __init__(self, t, **kw):
        self._t = t
        self.__dict__.update(kw)

    def get_type(self):
        return self._t


def test_height_is_measured_from_the_ground_not_the_drifted_baro():
    """review3 #1: the baro drifts while the drone sits on the ground. Height and position
    targets must be relative to the ground at arming, as NAV_TAKEOFF is."""
    from pymavlink import mavutil
    M = mavutil.mavlink
    v = _mav_vehicle()
    lpn = lambda z: _Msg("LOCAL_POSITION_NED", x=0.0, y=0.0, z=z, vx=0.0, vy=0.0, vz=0.0)
    v._handle(lpn(-0.96))                       # disarmed, baro has drifted to +0.96 m
    assert v.state().alt == pytest.approx(0.0)
    v.s.armed = True                            # armed; the land detector still says ON_GROUND
    v._handle(_Msg("EXTENDED_SYS_STATE", landed_state=M.MAV_LANDED_STATE_ON_GROUND))
    v._handle(lpn(-0.97))
    assert v.state().alt == pytest.approx(0.0)
    v._handle(_Msg("EXTENDED_SYS_STATE", landed_state=M.MAV_LANDED_STATE_IN_AIR))
    v._handle(lpn(-1.77))                       # climbed 0.8 m: frozen reference
    assert v.state().alt == pytest.approx(0.8)
    v.set_target(0.5, 0.0, 0.8, 0.0)            # hold 0.8 m above the ground ...
    z = v.m.mav.sent[-1][7]  # (time, sys, comp, frame, mask, x, y, z, ...)
    assert z == pytest.approx(-1.77)            # ... = 1.77 m above the EKF origin
