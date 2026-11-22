"""tools/voice_flights: test 4 ("fly forward" / "test four") must never send anything to the drone."""
import argparse
import asyncio
import time

import pytest

from edge.config import SafetyConfig, WatchdogConfig
from edge.server import EdgeServer
from edge.sim import SimVehicle
from tools.voice_flights import classify, create_app


def test_test4_wins_over_every_other_word():
    for text in ("Test four.", "Running test 4: rise, go left and turn around", "test for",
                 "fly forward", "go straight ahead"):
        assert classify(text) == "forward"
        assert classify(text, flying=True) == "forward"
    assert classify("Rise, go left, then right, then come down") == "leftright"


@pytest.fixture
async def rig():
    """A drone in NORMAL flying mode (not demo) with a fresh preflight GO: the worst case."""
    cfg = SafetyConfig()
    sim = SimVehicle(cfg, speedup=4.0)
    srv = EdgeServer(sim, cfg, WatchdogConfig(), host="127.0.0.1", port=0,
                     perception_port=None, flow_port=None)
    await srv.start()
    sent = []
    real = srv.exec.handle
    srv.exec.handle = lambda cid, op, args: (sent.append(op), real(cid, op, args))[1]
    app = create_app(argparse.Namespace(edge=f"127.0.0.1:{srv.port}", port=0, gps=False,
                                        min_batt=11.2, demo=False))
    hub = app.state.hub
    hub.ai.speak = _silent
    task = asyncio.ensure_future(hub.keep_edge())
    for _ in range(50):
        if hub.edge is not None and hub.edge.connected:
            break
        await asyncio.sleep(0.1)
    assert hub.edge is not None and hub.edge.connected and hub.edge.telemetry  # really linked
    hub.preflight.update(ok=True, t=time.time())
    yield srv, sim, hub, sent
    task.cancel()
    await srv.close()


async def _silent(text):
    return b""


async def test_test4_sends_nothing_even_in_flying_mode(rig):
    srv, sim, hub, sent = rig
    for text in ("Test four. Hey drone, fly forward.", "fly forward", "test 4"):
        r = await hub.command(text)
        assert r["preset"] == "forward" and r["started"] is False
    await asyncio.sleep(0.5)
    assert sent == []                                   # not one command reached the drone
    assert hub.edge.connected                           # ...although the link was up
    assert not sim.s.armed and (sim.s.n, sim.s.e, sim.s.alt) == (0.0, 0.0, 0.0)
    replies = [e["text"] for e in hub.events if e["kind"] == "reply"]
    assert len(replies) == 3 and all("not flying forward" in t and "person" in t for t in replies)
    assert any(e["kind"] == "alert" for e in hub.events)
    await hub.command("rise and hover")                 # control: a real preset DOES send
    await asyncio.sleep(0.5)
    assert "takeoff" in sent
    hub.flight.cancel()


async def test_test4_quotes_a_real_detection(rig):
    srv, sim, hub, sent = rig

    async def detector():  # 10 Hz, like edge/vision
        while True:
            srv.on_perception({"person_m": 2.3, "palm": False,
                               "boxes": [[0.4, 0.2, 0.6, 0.9, 0.9, 2.3]]})
            await asyncio.sleep(0.1)
    feed = asyncio.ensure_future(detector())
    await asyncio.sleep(0.6)  # into the telemetry
    r = await hub.command("test four")
    feed.cancel()
    assert r["person_m"] == pytest.approx(2.3)
    assert "2.3 metres ahead" in [e["text"] for e in hub.events if e["kind"] == "reply"][-1]
    assert sent == []
