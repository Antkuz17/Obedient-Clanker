"""Station + cloud: brain, localization, planner (against the sim edge), memory, cloud relay.

No network and no keys: the AI is a fake with the same methods as station.ai.AI.
"""
import asyncio
import json
import math
import socket
import zlib

import httpx
import numpy as np
import pytest

from edge.config import SafetyConfig, WatchdogConfig
from edge.server import EdgeServer
from edge.sim import SimVehicle
from edge.vehicle import ned_to_body
from edge.video import SIM_WORLD, intrinsics
from shared.client import EdgeClient
from station import localize_stub
from station.ai import AI
from station.brain import Brain, fast_parse, rule_parse
from station.camera import Frame, Pose
from station.config import MissingKey, Settings
from station.localize import box_to_goal, locate, parse_box
from station.planner import Planner, PlannerConfig, compass


def free_port() -> int:
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind(("127.0.0.1", 0))
    p = s.getsockname()[1]
    s.close()
    return p


class FakeAI:
    """Stands in for station.ai.AI. Embeddings: hashed bag of words, so similar text is close."""

    def __init__(self, chat_reply: dict | None = None, vision_reply: str = "It says HELLO."):
        self.chat_reply = chat_reply
        self.vision_reply = vision_reply
        self.last_latency = {"chat": 0.5}
        self.calls: list[str] = []

    async def chat(self, messages, tools=None, **kw):
        self.calls.append("chat")
        if self.chat_reply is None:
            raise httpx.ConnectError("down")
        return self.chat_reply

    async def vision(self, prompt, jpeg, system="", max_tokens=400, think=None):
        self.calls.append("vision")
        return self.vision_reply

    async def embed(self, texts):
        out = []
        for t in texts:
            v = np.zeros(64, np.float32)
            for w in t.lower().replace(",", " ").split():
                v[zlib.crc32(w.rstrip("s").encode()) % 64] += 1
            out.append(v.tolist())
        return out

    async def speak(self, text):
        return b"mp3:" + text.encode()

    async def transcribe(self, audio, filename="clip.webm"):
        return audio.decode()

    async def aclose(self):
        pass


def tool_call(name: str, **args) -> dict:
    return {"function": {"name": name, "arguments": json.dumps(args)}}


# --- brain ----------------------------------------------------------------------------------
@pytest.mark.parametrize("text,kinds", [
    ("take off", ["takeoff"]),
    ("move back 1 metre", ["move"]),
    ("turn around", ["move"]),
    ("land", ["land"]),
    ("stop!", ["stop"]),
    ("come back", ["home"]),
    ("fly to the door and take a photo, then come back", ["mission"]),
    ("where did I leave my keys?", ["recall"]),
    ("what's written here?", ["look"]),
    ("crash into the wall", []),
])
def test_rule_parse(text, kinds):
    r = rule_parse(text, {"armed": True, "alt": 1.0})
    assert [a["kind"] for a in r.actions] == kinds
    assert r.reply


def test_rule_parse_details():
    mv = rule_parse("move back 1 metre", {"armed": True}).actions[0]
    assert mv["fwd"] == -1.0 and mv["yaw_deg"] == 0
    assert rule_parse("turn around", {"armed": True}).actions[0]["yaw_deg"] == 360  # a full turn (filming preset)
    assert rule_parse("go up half a metre", {"armed": True}).actions[0]["up"] == 0.5
    m = rule_parse("fly to the door and take a photo, then come back", {"armed": True}).actions[0]
    assert (m["target"], m["action"], m["after"]) == ("door", "photo", "come_back")
    # landed: a move gets a takeoff first
    assert [a["kind"] for a in rule_parse("go forward 1 metre", {"armed": False}).actions] == [
        "takeoff", "move"]


async def test_brain_uses_tool_calls():
    ai = FakeAI({"content": "Up we go.", "tool_calls": [
        tool_call("takeoff", alt=1.0), tool_call("move", up=0.5, yaw_deg=180),
        tool_call("mission", target="red poster", action="read", after="come_back"),
        tool_call("flip"), tool_call("move")]})  # junk calls are dropped
    r = await Brain(ai).think("take off, go up a bit and turn around, then read the red poster")
    assert r.source == "llm" and r.reply == "Up we go."
    assert [a["kind"] for a in r.actions] == ["takeoff", "move", "mission"]
    assert r.actions[1] == {"kind": "move", "fwd": 0.0, "right": 0.0, "up": 0.5, "yaw_deg": 180.0}
    assert r.actions[2]["after"] == "come_back"


async def test_brain_refusal_and_fallback():
    r = await Brain(FakeAI({"content": "I'd rather not become modern art.", "tool_calls": []})
                    ).think("crash into the wall")
    assert r.actions == [] and "modern art" in r.reply
    r = await Brain(FakeAI(None)).think("land")  # LLM down -> rules
    assert r.source == "rules" and r.actions == [{"kind": "land"}]
    r = await Brain(None).think("take off")      # no key -> rules
    assert r.actions[0]["kind"] == "takeoff"


@pytest.mark.parametrize("text, kind", [
    ("Stop!", "stop"), ("stop stop stop", "stop"), ("Jarvis, stop now", "stop"), ("freeze", "stop"),
    ("hold on", "stop"), ("Land.", "land"), ("land now please", "land"), ("come down", "land")])
async def test_urgent_commands_skip_the_llm(text, kind):
    ai = FakeAI({"content": "should not be asked", "tool_calls": []})
    r = await Brain(ai).think(text)
    assert ai.calls == [] and r.actions == [{"kind": kind}]


async def test_long_moves_take_the_fast_path_now():
    """Outdoors: no distance or height cap in the fast path either."""
    ai = FakeAI({"content": "should not be asked", "tool_calls": []})
    r = await Brain(ai).think("take off to 3 metres and fly forward 20 metres")
    assert ai.calls == []
    assert r.actions[0] == {"kind": "takeoff", "alt": 3.0}
    assert r.actions[1]["fwd"] == 20.0


@pytest.mark.parametrize("text", ["land on the table after the photo", "don't stop at the door",
                                  "fly to the bus stop sign", "take off and find the door",
                                  "go left", "move forward a bit", "what's up", "land and go forward 1 metre"])
async def test_sentences_still_go_to_the_llm(text):
    ai = FakeAI({"content": "Sure.", "tool_calls": []})
    await Brain(ai).think(text)
    assert ai.calls == ["chat"]


@pytest.mark.parametrize("text, want", [
    ("take off", [("takeoff", 1.0)]),     # normal hover height (slow-and-low profile)
    ("Take off to 1.2 metres.", [("takeoff", 1.2)]),
    ("Move forward one meter and turn left 90 degrees.", [("fwd", 1.0), ("yaw_deg", -90.0)]),
    ("Jarvis, go back 50 centimetres please.", [("fwd", -0.5)]),
    ("Go up half a metre and turn around.", [("up", 0.5), ("yaw_deg", 360.0)]),
    ("1 metre left, then come back", [("right", -1.0), ("home", None)]),
    ("turn right 45°", [("yaw_deg", 45.0)]),
    ("forward 1 metre then land", [("fwd", 1.0), ("land", None)])])
async def test_simple_commands_skip_the_llm(text, want):
    ai = FakeAI({"content": "should not be asked", "tool_calls": []})
    r = await Brain(ai).think(text, {"armed": True})
    assert ai.calls == [] and r.source == "fast"
    got = []
    for a in r.actions:
        if a["kind"] == "move":
            got += [(k, a[k]) for k in ("fwd", "right", "up", "yaw_deg") if a[k]]
        else:
            got.append((a["kind"], a.get("alt")))
    assert got == want


async def test_fast_path_takes_off_first_when_landed():
    r = await Brain(FakeAI(None)).think("forward 1 metre", {"armed": False})
    assert [a["kind"] for a in r.actions] == ["takeoff", "move"]


async def test_ai_missing_key_and_requests():
    s = Settings(vultr_key="", eleven_key="", eleven_voice="")
    ai = AI(s)
    with pytest.raises(MissingKey) as e:
        await ai.chat([{"role": "user", "content": "hi"}])
    assert e.value.var == "VULTR_INFERENCE_KEY"
    assert await ai.speak("hello") == b""  # no TTS key -> text only, no crash
    await ai.aclose()

    seen = []

    def handler(req: httpx.Request) -> httpx.Response:
        seen.append((req.url.host, req.url.path, req.headers.get("authorization"),
                     req.headers.get("xi-api-key")))
        if req.url.path.endswith("/chat/completions"):
            return httpx.Response(200, json={"choices": [{"message": {"content": "ok"}}]})
        if req.url.path.endswith("/embeddings"):
            return httpx.Response(200, json={"data": [{"index": 0, "embedding": [1, 0]}]})
        if req.url.host == "api.elevenlabs.io":
            return httpx.Response(200, content=b"MP3")
        return httpx.Response(404)

    s = Settings(vultr_key="vk", eleven_key="ek", eleven_voice="jarvis")
    ai = AI(s, transport=httpx.MockTransport(handler))
    assert (await ai.chat([{"role": "user", "content": "hi"}]))["content"] == "ok"
    assert await ai.vision("read", b"\xff\xd8jpeg") == "ok"
    assert await ai.embed(["x"]) == [[1, 0]]
    assert await ai.speak("hi") == b"MP3"
    await ai.aclose()
    assert seen[0][2] == "Bearer vk" and seen[-1] == ("api.elevenlabs.io",
                                                      "/v1/text-to-speech/jarvis", None, "ek")


# --- localization ---------------------------------------------------------------------------
def sim_box(name: str, pose: Pose, K: dict) -> list[float]:
    """The same projection edge.video.SimCamera draws."""
    _, n, e, z0, h, w, _ = next(o for o in SIM_WORLD if o[0] == name)
    fwd, right = ned_to_body(n - pose.n, e - pose.e, pose.yaw)
    u = K["cx"] + K["fx"] * right / fwd
    half = K["fx"] * (w / 2) / fwd
    return [u - half, K["cy"] - K["fx"] * (z0 + h - pose.alt) / fwd,
            u + half, K["cy"] - K["fx"] * (z0 - pose.alt) / fwd]


@pytest.mark.parametrize("name,pose", [
    ("door", Pose(0, 0, 1.0, 0, 0)),
    ("door", Pose(0.5, -0.8, 1.0, 15, 0)),
    ("chair", Pose(0, 0, 1.0, 60, 0)),
])
def test_box_to_goal_recovers_sim_positions(name, pose):
    K = intrinsics()
    hit = box_to_goal(name, sim_box(name, pose, K), pose, K)
    _, n, e, *_ = next(o for o in SIM_WORLD if o[0] == name)
    assert hit["n"] == pytest.approx(n, abs=0.15) and hit["e"] == pytest.approx(e, abs=0.15)


def test_parse_box_formats():
    K = intrinsics()
    box, conf = parse_box('```json\n{"found": true, "box": [250, 100, 750, 900], "conf": 0.9}\n```',
                          K["width"], K["height"])
    assert box == pytest.approx([160, 48, 480, 432]) and conf == 0.9
    assert parse_box('{"found": false}', 640, 480) is None
    assert parse_box("no idea", 640, 480) is None


async def test_locate_with_vlm_box():
    K = intrinsics()
    pose = Pose(-1.0, 0, 1.0, 0, 0)  # 4 m back: the whole door fits in a ~45 deg frame
    x0, y0, x1, y1 = sim_box("door", pose, K)
    norm = [x0 / 640 * 1000, y0 / 480 * 1000, x1 / 640 * 1000, y1 / 480 * 1000]
    ai = FakeAI(vision_reply=json.dumps({"found": True, "box": norm, "conf": 0.8}))
    hit = await locate(ai, "door", b"jpg", pose, K)
    assert hit["n"] == pytest.approx(3.0, abs=0.15) and hit["conf"] == 0.8
    assert not hit["upper_bound"]
    ai.vision_reply = '{"found": true, "box": [1, 1, 900, 900], "conf": 0.1}'
    assert await locate(ai, "door", b"jpg", pose, K) is None  # low confidence


@pytest.mark.parametrize("dist", [1.2, 1.6, 2.0])
def test_close_door_filling_the_frame_is_never_placed_further_away(dist):
    """review3 #3: a door cut off by the frame read as 2.3-2.9 m away at any closer range,
    and the planner flew towards it. Now: range from a cut-off box is never beyond the truth,
    and with the bottom cut off it is flagged as an upper bound (the planner won't approach)."""
    K = intrinsics()
    pose = Pose(3.0 - dist, 0, 1.0, 0, 0)
    x0, y0, x1, y1 = sim_box("door", pose, K)
    clipped = [max(0, x0), max(0, y0), min(640, x1), min(480, y1)]   # what a VLM can return
    hit = box_to_goal("door", clipped, pose, K)
    assert hit is not None and hit["upper_bound"]                      # bottom off-frame too


def test_door_with_top_cut_off_uses_the_floor_range():
    """Top cut off, bottom (on the floor) visible: the floor-contact range is used, not the
    too-far size range."""
    K = intrinsics()
    pose = Pose(0.5, 0, 0.6, 0, 0)  # 2.5 m from the door, camera low: top cut, bottom in view
    x0, y0, x1, y1 = sim_box("door", pose, K)
    assert y0 < 0 < y1 < 479
    hit = box_to_goal("door", [x0, 0, x1, y1], pose, K)
    assert not hit["upper_bound"] and hit["dist"] == pytest.approx(2.5, abs=0.1)


# --- planner against the sim edge -----------------------------------------------------------
class SimCam:
    def __init__(self, sim: SimVehicle):
        self.sim = sim

    async def K(self):
        return intrinsics()

    async def frame(self):
        s = self.sim.state()
        return Frame(b"\xff\xd8sim", Pose(s.n, s.e, s.alt, s.yaw, 0.0))


@pytest.fixture
async def rig():
    cfg = SafetyConfig()
    sim = SimVehicle(cfg, speedup=10.0)
    srv = EdgeServer(sim, cfg, WatchdogConfig(hover_after_s=0.6, land_after_s=2.0),
                     host="127.0.0.1", port=0, perception_port=free_port(), flow_port=None)
    await srv.start()
    edge = await EdgeClient("127.0.0.1", srv.port).connect()
    await asyncio.sleep(0.3)
    yield sim, edge
    await edge.close()
    await srv.close()


def make_planner(sim, edge, ai=None, memory=None):
    said, photos = [], []

    async def say(t):
        said.append(t)

    async def on_photo(fr, caption):
        photos.append((fr, caption))

    p = Planner(edge, SimCam(sim), locate=localize_stub.locate, ai=ai, memory=memory, say=say,
                on_photo=on_photo)
    return p, said, photos


async def run_plan(p, actions, timeout=40):
    p.submit(actions)
    await asyncio.wait_for(p.wait(), timeout)


async def test_mission_door_photo_come_back(rig):
    sim, edge = rig
    p, said, photos = make_planner(sim, edge, ai=FakeAI(vision_reply="A brown door."))
    await run_plan(p, [{"kind": "mission", "target": "door", "action": "photo",
                        "question": "", "after": "stay"}])
    s = sim.state()
    assert s.armed and s.n == pytest.approx(2.0, abs=0.35) and abs(s.e) < 0.35  # 1 m short
    assert photos and "door" in photos[0][1].lower() and said[-1].startswith("Got the shot")
    await run_plan(p, [{"kind": "home"}])
    assert math.hypot(sim.state().n, sim.state().e) < 0.35


async def test_planner_reports_gate_verdicts(rig):
    """The dashboard's "Approved" comes from these callbacks; refusals still raise."""
    sim, edge = rig
    p, said, _ = make_planner(sim, edge)
    approved = []
    p.on_approved = approved.append
    await run_plan(p, [{"kind": "takeoff", "alt": 1.0}])
    assert approved == ["takeoff"]
    await run_plan(p, [{"kind": "move", "fwd": 0.0, "right": 0.0, "up": -5.0, "yaw_deg": 0.0}])
    assert said[-1].startswith("Safety says no") and approved == ["takeoff"]  # into the ground


async def test_reply_waits_for_the_gate(rig):
    """A refused first step is one line (the refusal), not "On it." + "Safety says no: ..."."""
    sim, edge = rig
    p, said, _ = make_planner(sim, edge)
    p.submit([{"kind": "takeoff", "alt": 1.0}], "Lifting off.")
    await asyncio.wait_for(p.wait(), 40)
    assert said == ["Lifting off."]
    said.clear()
    p.submit([{"kind": "move", "fwd": 0.0, "right": 0.0, "up": -5.0, "yaw_deg": 0.0}], "On it.")
    await asyncio.wait_for(p.wait(), 40)
    assert len(said) == 1 and said[0].startswith("Safety says no")
    said.clear()
    p.submit([{"kind": "recall", "query": "keys", "fly_there": False}], "Let me think back.")
    await asyncio.wait_for(p.wait(), 40)
    assert said[0] == "Let me think back."  # no flight step first: said at once


async def test_mission_scans_until_found(rig):
    sim, edge = rig
    p, said, _ = make_planner(sim, edge)
    await run_plan(p, [{"kind": "takeoff", "alt": 1.0},
                       {"kind": "mission", "target": "the chair", "action": "none",
                        "question": "", "after": "stay"}], timeout=60)
    assert any(l.startswith("yaw") for l in p.log)  # had to turn to find it
    s = sim.state()
    d = math.hypot(-1.0 - s.n, 2.0 - s.e)
    assert d == pytest.approx(1.0, abs=0.4)
    assert said[-1] == "At the chair."


async def test_mission_not_found_and_gate_rejection(rig):
    sim, edge = rig
    p, said, _ = make_planner(sim, edge)
    await run_plan(p, [{"kind": "mission", "target": "unicorn", "action": "none",
                        "question": "", "after": "stay"}], timeout=60)
    assert "couldn't find the unicorn" in said[-1]
    await run_plan(p, [{"kind": "move", "fwd": 0, "right": 0, "up": -10, "yaw_deg": 0}])
    assert said[-1].startswith("Safety says no")


async def test_look_read_and_stop_preempts(rig):
    sim, edge = rig
    p, said, _ = make_planner(sim, edge, ai=FakeAI(vision_reply="It says EXIT."))
    await run_plan(p, [{"kind": "takeoff", "alt": 1.0},
                       {"kind": "look", "action": "read", "question": ""}])
    assert said[-1] == "It says EXIT."
    p.submit([{"kind": "move", "fwd": 2.5, "right": 0, "up": 0, "yaw_deg": 0}])
    await asyncio.sleep(0.3)
    stop = p.submit([{"kind": "stop"}])
    await asyncio.wait_for(stop, 10)
    assert "plan cancelled" in p.log and sim.state().n < 2.3


async def test_recall_says_where_and_flies_there(rig):
    sim, edge = rig

    async def memory(q):
        return [{"caption": "silver keys on a chair", "n": -2.0, "e": -1.0, "alt": 1.0,
                 "yaw": 200.0, "t": __import__("time").time() - 125}]

    p, said, _ = make_planner(sim, edge, memory=memory)
    await run_plan(p, [{"kind": "recall", "query": "my keys", "fly_there": True}], timeout=60)
    assert "south-west" in said[0] and "2 minutes ago" in said[0]
    s = sim.state()
    assert s.n == pytest.approx(-2.0, abs=0.35) and s.e == pytest.approx(-1.0, abs=0.35)
    assert said[-1] == "This is where I saw it."


def test_compass():
    assert compass(1, 0) == "north" and compass(0, 1) == "east" and compass(-1, -1) == "south-west"


# --- memory ---------------------------------------------------------------------------------
async def test_memory_ingest_and_search(tmp_path):
    from cloud.memory import Memory
    mem = Memory(tmp_path, FakeAI())
    await mem.ingest(b"j1", {"n": 0, "e": 3, "yaw": 90, "t": 100}, "white whiteboard centre")
    await mem.ingest(b"j2", {"n": -2, "e": -1.5, "yaw": 200, "t": 200},
                     "silver keys on a grey table, left")
    await mem.ingest(b"j3", {"n": 2, "e": -2.5, "yaw": 300, "t": 300}, "green plant in a pot")
    hits = await mem.search("where did I leave my keys")
    assert hits[0]["n"] == -2 and "keys" in hits[0]["caption"]
    assert mem.jpeg_path(hits[0]["id"]).read_bytes() == b"j2"
    assert mem.count() == 3 and len(mem.recent()) == 3


# --- cloud ----------------------------------------------------------------------------------
def test_cloud_relays_commands_to_station(tmp_path):
    from fastapi.testclient import TestClient
    from cloud.app import create_app

    s = Settings(vultr_key="", station_token="tok", data_dir=tmp_path)
    app = create_app(s, ai=FakeAI())
    with TestClient(app) as c:
        assert c.get("/health").json()["station"] is False
        with pytest.raises(Exception):
            with c.websocket_connect("/ws/station?token=wrong") as ws:
                ws.receive_json()
        with c.websocket_connect("/ws/station?token=tok") as st, \
                c.websocket_connect("/ws/ui?client=judge") as ui:
            assert ui.receive_json()["type"] == "hello"
            st.send_json({"type": "telemetry", "state": "IDLE", "armed": False, "alt": 0.0})
            r = c.post("/text", json={"text": "take off", "client": "judge"}).json()
            assert r["ok"] and r["actions"][0]["kind"] == "takeoff"
            m = st.receive_json()
            assert m["type"] == "actions" and m["actions"][0]["kind"] == "takeoff"
            assert m["reply"] == "Lifting off."  # the station says it once the edge accepts
            st.send_json({"type": "say", "text": "Lifting off."})
            assert st.receive_json() == {"type": "audio", "mp3": "bXAzOkxpZnRpbmcgb2ZmLg==",
                                         "text": "Lifting off."}
            # another viewer can't steal the controls, but can always hit STOP
            r = c.post("/text", json={"text": "land", "client": "someone"}).json()
            assert not r["ok"] and "controls" in r["reply"]
            ui.send_json({"type": "op", "op": "stop"})
            for _ in range(10):
                m = st.receive_json()
                if m["type"] == "actions":
                    break
            assert m["actions"] == [{"kind": "stop"}]
            # memory search round trip from the station
            st.send_json({"type": "memory_search", "q": "keys", "req": 7})
            while (m := st.receive_json())["type"] != "memory_result":
                pass
            assert m == {"type": "memory_result", "req": 7, "hits": []}
        # voice: FakeAI.transcribe echoes the bytes
        r = c.post("/voice", files={"audio": ("clip.webm", b"land")},
                   data={"client": "judge"}).json()
        assert r["text"] == "land" and r["ok"] is False  # station gone


# --- station: edge reconnect, cloud-lost landing --------------------------------------------
async def _edge_server(port: int = 0):
    cfg = SafetyConfig()
    sim = SimVehicle(cfg, speedup=10.0)
    srv = EdgeServer(sim, cfg, WatchdogConfig(hover_after_s=0.6, land_after_s=2.0),
                     host="127.0.0.1", port=port, perception_port=free_port(), flow_port=None)
    await srv.start()
    return sim, srv


async def _until(cond, timeout=15.0):
    t0 = asyncio.get_running_loop().time()
    while not cond():
        assert asyncio.get_running_loop().time() - t0 < timeout, "timed out"
        await asyncio.sleep(0.05)


async def test_station_reconnects_to_edge(tmp_path, monkeypatch):
    import station.station as st_mod
    monkeypatch.setattr(st_mod, "EDGE_RETRY_S", (0.1, 0.3))
    sim, srv = await _edge_server()
    port = srv.port
    st = st_mod.Station(Settings(vultr_key="", edge_port=port, data_dir=tmp_path),
                        stub_locate=True, speaker=False, ai=FakeAI())
    said = []
    st.say = lambda t: said.append(t) or asyncio.sleep(0)
    st.planner._say = st.say
    loop_task = asyncio.create_task(st.edge_loop())
    await _until(lambda: st.edge.connected)
    first = st.edge
    await srv.close()                       # the Pi loses power: no FIN, telemetry just stops
    await _until(lambda: any("lost the link" in s for s in said))
    sim2, srv2 = await _edge_server(port)   # ... and comes back
    await _until(lambda: st.edge.connected and st.edge is not first)
    assert st.planner.edge is st.edge
    st.planner.submit([{"kind": "takeoff", "alt": 1.0}])
    await asyncio.wait_for(st.planner.wait(), 30)
    assert sim2.state().armed
    loop_task.cancel()
    await st.edge.close()
    await srv2.close()


async def test_station_lands_when_cloud_lost(rig, tmp_path, monkeypatch):
    import station.station as st_mod
    sim, edge = rig
    st = st_mod.Station(Settings(vultr_key="", data_dir=tmp_path), stub_locate=True,
                        speaker=False, ai=FakeAI())
    st.edge = st.planner.edge = edge
    said = []
    st.say = lambda t: said.append(t) or asyncio.sleep(0)
    st.planner._say = st.say
    await edge.run("takeoff", alt=1.0, timeout=30)
    st.cloud_lost_land_s = 0.5
    st._cloud_down_since = __import__("time").monotonic()
    wd = asyncio.create_task(st.cloud_watchdog())
    await _until(lambda: not sim.state().armed, 30)
    assert said == ["I've lost the cloud, so I'm landing."]
    wd.cancel()


# --- off-drone perception: closed-loop approach, confirmation, pointing, rerank --------------
def biased_locate(scale: float):
    """Ground truth with the range from the capture pose scaled: size-prior error."""
    async def loc(ai, name, jpeg, pose, K):
        hit = await localize_stub.locate(ai, name, jpeg, pose, K)
        if hit:
            hit = {**hit, "n": pose.n + scale * (hit["n"] - pose.n),
                   "e": pose.e + scale * (hit["e"] - pose.e)}
        return hit
    return loc


async def test_closed_loop_approach_beats_one_shot(rig):
    sim, edge = rig
    chair = (-1.0, 2.0)
    errs = {}
    for refine_max in (0, 2):
        p, said, _ = make_planner(sim, edge)
        p.locate = biased_locate(1.4)  # range 40 % too long
        p.cfg.refine_beyond, p.cfg.refine_max = 1.5, refine_max
        await run_plan(p, [{"kind": "takeoff", "alt": 1.0}, {"kind": "home"}])
        await run_plan(p, [{"kind": "move", "yaw_deg": -sim.state().yaw}])
        await run_plan(p, [{"kind": "mission", "target": "chair", "action": "none",
                            "question": "", "after": "stay"}], timeout=60)
        s = sim.state()
        errs[refine_max] = abs(math.hypot(chair[0] - s.n, chair[1] - s.e) - 1.0)
        if refine_max:
            assert any(l.startswith("refine 0: new fix") for l in p.log)
    print("approach error (m):", errs)
    assert errs[2] < 0.6 * errs[0], errs


async def test_shaky_sighting_needs_a_second_look(rig):
    sim, edge = rig
    calls = []

    async def flaky(ai, name, jpeg, pose, K):
        calls.append(1)
        return {"n": 3.0, "e": 0.0, "dist": 3.0, "bearing": 0.0, "conf": 0.4} if len(calls) == 1 else None

    p, said, _ = make_planner(sim, edge)
    p.locate = flaky
    p.cfg.scan_step = 180  # two looks only
    await run_plan(p, [{"kind": "takeoff", "alt": 1.0},
                       {"kind": "mission", "target": "door", "action": "none", "question": "",
                        "after": "stay"}], timeout=40)
    assert "unconfirmed sighting" in p.log and "couldn't find the door" in said[-1]


async def test_fly_to_where_im_pointing(rig):
    sim, edge = rig
    r = rule_parse("fly to where I'm pointing", {"armed": True})
    assert r.actions[0]["target"] == "where I'm pointing"
    p, said, _ = make_planner(sim, edge, ai=FakeAI(vision_reply="The door."))
    await run_plan(p, r.actions, timeout=60)
    assert said[0] == "You're pointing at the door." and said[-1] == "At the door."
    assert sim.state().n == pytest.approx(2.0, abs=0.4)
    p, said, _ = make_planner(sim, edge, ai=FakeAI(vision_reply="NONE"))
    await run_plan(p, r.actions)
    assert "can't tell what you're pointing at" in said[-1]


async def test_memory_rerank_and_fallback(tmp_path):
    from cloud.memory import Memory

    class RerankAI(FakeAI):
        async def rerank(self, q, docs):
            return [1.0 if "drawer" in d else 0.0 for d in docs]

    mem = Memory(tmp_path / "a", RerankAI(), min_score=-1)
    await mem.ingest(b"1", {"n": 0, "e": 0, "t": 1}, "keys keys keys on a table")
    await mem.ingest(b"2", {"n": 1, "e": 1, "t": 2}, "keys inside an open drawer")
    assert (await mem.search("keys"))[0]["caption"] == "keys inside an open drawer"

    class BrokenRerank(FakeAI):
        async def rerank(self, q, docs):
            raise httpx.HTTPStatusError("404", request=None, response=None)

    mem = Memory(tmp_path / "b", BrokenRerank())
    await mem.ingest(b"1", {"n": 0, "e": 0, "t": 1}, "silver keys on a table")
    assert (await mem.search("keys"))[0]["caption"] == "silver keys on a table"


async def test_stale_pose_blocks_localization(rig):
    sim, edge = rig
    assert Pose.parse("1,2,0.9,90,5", "3.5").age == 3.5 and Pose.parse("1,2,0.9,90,5").age == 0

    class StaleCam(SimCam):
        async def frame(self):
            fr = await super().frame()
            fr.pose.age = 4.0
            return fr

    p, said, _ = make_planner(sim, edge)
    p.camera = StaleCam(sim)
    await run_plan(p, [{"kind": "takeoff", "alt": 1.0},
                       {"kind": "mission", "target": "door", "action": "none", "question": "",
                        "after": "stay"}])
    assert "position feed is 4 seconds old" in said[-1]
    assert not any(l.startswith("goto") for l in p.log)


async def test_check_ai_runs_against_a_fake_vultr(capsys):
    """The key checker's request shapes and verdicts, against a mocked Vultr + ElevenLabs."""
    from station import check_ai
    door = [500 - 1000 * 0.15, 0, 500 + 1000 * 0.15, 1000]  # rough; parse only

    def handler(req: httpx.Request) -> httpx.Response:
        path = req.url.path
        if path.endswith("/models"):
            return httpx.Response(200, json={"data": [{"id": m} for m in (
                "glm-5.3-flash", "qwen3.8-flash-next", "qwen3.8-27b", "whisper-large-v3-turbo",
                "qwen3-embedding-4b", "bge-reranker-v2-m3", "chatterbox")]})
        if path.endswith("/chat/completions"):
            body = json.loads(req.content)
            if body["model"] == "qwen3.8-27b":
                prompt = body["messages"][-1]["content"][0]["text"]
                txt = ("It says EXIT 42." if "Read" in prompt else "84. 12 times 7." if "Solve"
                       in prompt else json.dumps({"found": True, "box": door, "conf": 0.9}))
                return httpx.Response(200, json={"choices": [{"message": {"content": txt}}]})
            user = body["messages"][-1]["content"]
            tc = ([] if "crash" in user else
                  [tool_call("move", up=0.5, yaw_deg=180)] if "little" in user else
                  [tool_call("recall", query="keys")] if "keys" in user else
                  [tool_call("mission", target="red poster", action="read", after="come_back")])
            return httpx.Response(200, json={"choices": [{"message": {
                "content": "Sure.", "tool_calls": tc}}]})
        if path.endswith("/embeddings"):
            return httpx.Response(200, json={"data": [{"index": 0, "embedding": [0.1] * 8}]})
        if path.endswith("/rerank"):
            return httpx.Response(200, json={"results": [{"index": 0, "relevance_score": 0.1},
                                                         {"index": 1, "relevance_score": 0.9}]})
        if path.endswith("/audio/speech") or req.url.host == "api.elevenlabs.io":
            return httpx.Response(200, content=b"ID3fake-mp3")
        if path.endswith("/audio/transcriptions"):
            return httpx.Response(200, json={"text": "Fly to the door and take a photo."})
        return httpx.Response(404)

    s = Settings(vultr_key="k", eleven_key="e", eleven_voice="v")
    check_ai.results.clear()
    code = await check_ai.main(None, s, AI(s, transport=httpx.MockTransport(handler)))
    out = capsys.readouterr().out
    by = {(r[0], r[1]) for r in check_ai.results}
    assert ("brain", "FAIL") not in by and ("stt", "PASS") in by and ("rerank", "PASS") in by
    assert ("models", "PASS") in by and ("voice e2e", "PASS") in by
    assert "read -> 'It says EXIT 42.'" in out
    assert code in (0, 1)  # vision box accuracy depends on the fake box; shape is what's tested


# --- QNX demo panel: Pi stats -> station UDP -> cloud -> dashboard ---------------------------
def test_cloud_keeps_qnx_state_for_late_viewers(tmp_path):
    from fastapi.testclient import TestClient
    from cloud.app import create_app

    app = create_app(Settings(vultr_key="", station_token="tok", data_dir=tmp_path), ai=FakeAI())
    with TestClient(app) as c, c.websocket_connect("/ws/station?token=tok") as st:
        for m in [
            {"src": "supervisor", "kind": "start", "name": "camgrab", "pid": 10},
            {"src": "supervisor", "kind": "exit", "name": "camgrab", "signal": 9},
            {"src": "supervisor", "kind": "restart", "name": "camgrab", "pid": 11, "restarts": 1,
             "spawn_ms": 3.5, "ready_ms": 208.0},
            {"src": "edge", "kind": "loop", "p50_ms": 0.1, "p99_ms": 0.4, "max_ms": 0.9},
            {"src": "edge", "kind": "stop_latency", "what": "person", "ms": 41.0, "edge_ms": 0.3},
            {"src": "jitter", "kind": "jitter_case", "hog": True, "prio": 50, "p99_ms": 0.2},
        ]:
            st.send_json({"type": "qnx", **m})
        st.send_json({"type": "telemetry", "state": "IDLE"})  # in order: qnx all handled first
        with c.websocket_connect("/ws/ui") as ui:
            while (m := ui.receive_json())["type"] != "qnx_state":
                pass
        q = c.get("/qnx").json()
    assert m == q
    assert q["procs"] == [{"name": "camgrab", "restarts": 1, "state": "up", "pid": 11,
                           "spawn_ms": 3.5, "ready_ms": 208.0, "t": None}]
    assert q["loop"][0]["p99_ms"] == 0.4 and q["stops"][0]["ms"] == 41.0
    assert q["jitter"][0]["hog"] is True
    log = c.get("/log").json()
    assert any("camgrab restarted: respawn 3.5 ms, serving again 208.0 ms" in r["text"] for r in log)
    assert any("person stop: 41.0 ms" in r["text"] for r in log)


def test_cloud_process_list_from_supervisor_status(tmp_path):
    """A dashboard that connects after the start events still sees every process."""
    from fastapi.testclient import TestClient
    from cloud.app import create_app

    app = create_app(Settings(vultr_key="", station_token="tok", data_dir=tmp_path), ai=FakeAI())
    with TestClient(app) as c, c.websocket_connect("/ws/station?token=tok") as st:
        for m in [{"src": "supervisor", "kind": "status", "name": "edge", "pid": 5,
                   "state": "up", "restarts": 0, "prio": 45},
                  {"src": "supervisor", "kind": "status", "name": "vision", "pid": None,
                   "state": "down", "restarts": 2, "prio": 11}]:
            st.send_json({"type": "qnx", **m})
        st.send_json({"type": "telemetry", "state": "IDLE"})
        with c.websocket_connect("/ws/ui") as ui:
            while ui.receive_json()["type"] != "qnx_state":
                pass
        q = c.get("/qnx").json()
    assert {p["name"]: p["state"] for p in q["procs"]} == {"edge": "up", "vision": "down"}
    assert q["events"] == []                       # snapshots aren't events


async def test_station_forwards_qnx_datagrams(tmp_path):
    import socket
    from station import station as st_mod
    port = free_port()
    st = st_mod.Station(Settings(vultr_key="", data_dir=tmp_path), stub_locate=True,
                        speaker=False, ai=FakeAI(), qnx_port=port)
    sent: list[dict] = []

    async def fake_send(m):
        sent.append(m)
        return True
    st._send_cloud = fake_send
    await st.qnx_listen()
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
        s.sendto(json.dumps({"t": 1.0, "kind": "restart", "name": "flow"}).encode(), ("127.0.0.1", port))
        s.sendto(b"not json", ("127.0.0.1", port))
        s.sendto(json.dumps({"type": "actions", "src": "edge", "kind": "loop"}).encode(),
                 ("127.0.0.1", port))
    await _until(lambda: len(sent) >= 2, timeout=3)
    assert sent[0]["type"] == "qnx" and sent[0]["src"] == "supervisor" and sent[0]["name"] == "flow"
    assert sent[1]["type"] == "qnx" and sent[1]["was_type"] == "actions"


async def test_edge_stats_loop_and_stop_latency():
    import socket, time as _t
    rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rx.bind(("127.0.0.1", 0))
    rx.settimeout(3)
    cfg = SafetyConfig()
    sim = SimVehicle(cfg, speedup=10.0)
    srv = EdgeServer(sim, cfg, host="127.0.0.1", port=0, perception_port=None, flow_port=None,
                     stats_addr=rx.getsockname())
    await srv.start()
    try:
        await asyncio.sleep(1.2)
        loop = json.loads(rx.recv(4096))
        assert loop["src"] == "edge" and loop["kind"] == "loop" and loop["n"] > 20
        assert 0 <= loop["p50_ms"] <= loop["p99_ms"] <= loop["max_ms"]
        srv._stop_latency("person", {"t": _t.time() - 0.05, "infer_ms": 12}, _t.time())
        while (m := json.loads(rx.recv(4096)))["kind"] != "stop_latency":
            pass
        assert m["what"] == "person" and m["ms"] >= 50 and m["infer_ms"] == 12 and "edge_ms" in m
    finally:
        await srv.close()
        rx.close()


# --- review3 ---------------------------------------------------------------------------------
async def test_target_filling_the_frame_is_not_approached(rig):
    """review3 #3: a range that is only an upper bound must not be flown towards."""
    sim, edge = rig
    p, said, _ = make_planner(sim, edge)

    async def upper(ai, name, jpeg, pose, K):
        hit = await localize_stub.locate(ai, name, jpeg, pose, K)
        return {**hit, "upper_bound": True} if hit else None
    p.locate = upper
    await run_plan(p, [{"kind": "takeoff", "alt": 1.0}])
    n0 = sim.state().n
    await run_plan(p, [{"kind": "mission", "target": "door", "action": "none",
                        "question": "", "after": "stay"}])
    assert abs(sim.state().n - n0) < 0.1 and not any(l.startswith("goto") for l in p.log)
    assert "right in front of me" in said[-2]


def test_planner_clamps_around_the_first_takeoff_point():
    """review3: the planner's fence clamp must agree with the edge's (first takeoff point)."""
    assert Planner(None, None, locate=None)._clamp(60.0, -30.0) == (60.0, -30.0)  # no fence
    p = Planner(None, None, locate=None, safety=SafetyConfig(fence_half=3.0))
    p.home = (2.0, 2.0)
    assert p._clamp(6.0, -3.0) == pytest.approx((4.6, -0.6))
    p.home = None
    assert p._clamp(6.0, -3.0) == pytest.approx((2.6, -2.6))


@pytest.mark.parametrize("text", ["take off right now", "calm down", "what's up", "shut down",
                                  "ok back to work everyone", "is anything left in the battery"])
def test_rule_parse_ignores_stray_direction_words(text):
    """review3 #9: everyday words used to move the drone in the no-key fallback."""
    r = rule_parse(text, {"armed": True})
    assert not any(a["kind"] == "move" for a in r.actions), r.actions


@pytest.mark.parametrize("text,key,want", [
    ("go forward 50 cm", "fwd", 0.5), ("move 30 centimetres left", "right", -0.3),
    ("move back 20cm please", "fwd", -0.2), ("fly up fifty centimeters", "up", 0.5)])
def test_rule_parse_reads_centimetres(text, key, want):
    a = rule_parse(text, {"armed": True}).actions
    assert len(a) == 1 and a[0][key] == pytest.approx(want)


def test_rule_parse_takeoff_in_centimetres():
    assert rule_parse("take off to 80 cm").actions == [{"kind": "takeoff", "alt": 0.8}]


def test_rule_parse_back_up_a_bit():
    a = rule_parse("back up a bit", {"armed": True}).actions
    assert a == [{"kind": "move", "fwd": -0.5, "right": 0.0, "up": 0.0, "yaw_deg": 0.0}]


def _next_actions(st) -> list[dict]:
    """The next `actions` the cloud sends the station (skipping spoken replies)."""
    for _ in range(20):
        m = st.receive_json()
        if m["type"] == "actions":
            return m["actions"]
    raise AssertionError("no actions sent to the station")


def test_any_viewer_can_fly(tmp_path):
    """No pilot PIN: any dashboard viewer, local or through the proxy, can fly, LAND and STOP.
    The dev station token still only works from this machine."""
    from fastapi.testclient import TestClient
    from cloud.app import create_app

    s = Settings(vultr_key="", station_token="dev-token", data_dir=tmp_path)
    app = create_app(s, ai=FakeAI())
    fwd = {"X-Forwarded-For": "203.0.113.9"}
    with TestClient(app) as c:
        with pytest.raises(Exception):  # the dev token through the proxy: refused
            with c.websocket_connect("/ws/station?token=dev-token", headers=fwd) as ws:
                ws.receive_json()
        with c.websocket_connect("/ws/station?token=dev-token") as st:
            r = c.post("/text", json={"text": "take off", "client": "judge"}, headers=fwd).json()
            assert r["ok"] and r["actions"][0]["kind"] == "takeoff"
            assert _next_actions(st)[0]["kind"] == "takeoff"
            with c.websocket_connect("/ws/ui?client=stranger") as ui:
                assert "pin_required" not in ui.receive_json()
                ui.send_json({"type": "op", "op": "land"})
                assert _next_actions(st) == [{"kind": "land"}]
                ui.send_json({"type": "op", "op": "stop"})
                assert _next_actions(st) == [{"kind": "stop"}]
        assert c.post("/memory/reset", json={}).json() == {"deleted": 0}


def test_streamed_voice_runs_the_command_and_reports_errors(tmp_path, monkeypatch):
    """/ws/voice: PCM frames while held, "end" on release -> the same reply as POST /voice.
    If streaming STT can't start, the dashboard gets {"error"} and uploads the clip instead."""
    from fastapi.testclient import TestClient
    import cloud.app as cloud_app

    class FakeStream:
        def __init__(self):
            self.pcm = b""

        @classmethod
        async def open(cls, s):
            if not s.stt_stream_model:
                raise RuntimeError("no model")
            return cls()

        async def send(self, pcm):
            self.pcm += pcm

        async def finish(self):
            return self.pcm.decode()  # "transcribes" by echoing the bytes

        async def close(self):
            pass

    monkeypatch.setattr(cloud_app, "StreamingSTT", FakeStream)
    s = Settings(vultr_key="", station_token="tok", data_dir=tmp_path,
                 stt_stream_model="voxtral-mini-4b-realtime")
    with TestClient(cloud_app.create_app(s, ai=FakeAI())) as c:
        with c.websocket_connect("/ws/station?token=tok") as st:
            with c.websocket_connect("/ws/voice?client=judge") as v:
                v.send_bytes(b"forward 1 ")
                v.send_bytes(b"metre")
                v.send_text("end")
                r = v.receive_json()
            assert r["text"] == "forward 1 metre" and r["ok"]
            assert _next_actions(st)[0] == {"kind": "move", "fwd": 1.0, "right": 0.0,
                                            "up": 0.0, "yaw_deg": 0.0}
        s.stt_stream_model = ""
        with c.websocket_connect("/ws/voice?client=judge") as v:
            assert "error" in v.receive_json()


def test_two_tabs_get_their_own_viewer(tmp_path):
    from fastapi.testclient import TestClient
    from cloud.app import create_app

    s = Settings(vultr_key="", station_token="tok", data_dir=tmp_path)
    with TestClient(create_app(s, ai=FakeAI())) as c:
        with c.websocket_connect("/ws/ui?client=phone-ab") as a, \
                c.websocket_connect("/ws/ui?client=phone-ab") as b:
            ya, yb = a.receive_json()["you"], b.receive_json()["you"]
            assert ya == "phone-ab" and yb != ya and yb.startswith("phone-ab")
            assert c.get("/health").json()["viewers"] == 2


async def test_slow_viewer_only_delays_itself():
    """A phone on bad Wi-Fi used to stall the station relay: every send was awaited in turn."""
    from cloud.app import Viewer

    class SlowWS:
        def __init__(self, delay):
            self.delay, self.got = delay, []

        async def send_json(self, m):
            await asyncio.sleep(self.delay)
            self.got.append(m)

        async def close(self):
            pass
    slow, fast = SlowWS(0.05), SlowWS(0.0)
    vs, vf = Viewer(slow), Viewer(fast)
    t0 = asyncio.get_running_loop().time()
    for i in range(10):
        for v in (vs, vf):
            v.put({"type": "frame", "i": i})
            v.put({"type": "log", "i": i})
    assert asyncio.get_running_loop().time() - t0 < 0.05  # put() never waits
    await asyncio.sleep(0.1)
    assert [m["i"] for m in fast.got if m["type"] == "log"] == list(range(10))
    await asyncio.sleep(1.0)  # the slow one: every log line in order, but only fresh frames
    assert [m["i"] for m in slow.got if m["type"] == "log"] == list(range(10))
    frames = [m["i"] for m in slow.got if m["type"] == "frame"]
    assert len(frames) < 10 and frames[-1] == 9
    vs.close(), vf.close()


def test_memory_search_is_rate_limited(tmp_path):
    from fastapi.testclient import TestClient
    import cloud.app as cloud_app

    s = Settings(vultr_key="", station_token="tok", data_dir=tmp_path)
    with TestClient(cloud_app.create_app(s, ai=FakeAI())) as c:
        codes = [c.post("/memory/search", json={"q": "keys"}).status_code
                 for _ in range(cloud_app.SEARCH_PER_MIN + 1)]
    assert codes[:-1] == [200] * cloud_app.SEARCH_PER_MIN and codes[-1] == 429


async def test_qnx_datagrams_from_strangers_are_ignored(tmp_path):
    """The stats port listens on every interface: only the Pi (EDGE_HOST) and this machine may
    put numbers on the judges' dashboard."""
    from station import station as st_mod
    st = st_mod.Station(Settings(vultr_key="", data_dir=tmp_path, edge_host="192.0.2.7"),
                        stub_locate=True, speaker=False, ai=FakeAI(), qnx_port=free_port())
    got: list[tuple[dict, str]] = []
    st.on_qnx = lambda m, host="": got.append((m, host))
    await st.qnx_listen()
    assert "192.0.2.7" in st.qnx_allowed
    msg = json.dumps({"kind": "restart", "name": "flow"}).encode()
    st.on_qnx_datagram(msg, "198.51.100.66")   # someone else on the Wi-Fi
    st.on_qnx_datagram(msg, "192.0.2.7")       # the Pi
    st.on_qnx_datagram(msg, "127.0.0.1")       # this machine
    assert [h for _, h in got] == ["192.0.2.7", "127.0.0.1"]


def test_planner_waits_long_enough_for_long_moves():
    """No distance cap outdoors: long moves must not hit the old fixed 60 s timeout."""
    class NoTelemetry:
        telemetry: dict = {}
    p = Planner(NoTelemetry(), None, locate=None)
    assert p._timeout("move_body", {"fwd": 100.0}) > 60
    assert p._timeout("goto", {"n": 0.0, "e": 100.0}) > 60
    assert p._timeout("takeoff", {"alt": 30.0}) > 60
    assert p._timeout("land", {}) >= 90
    assert p._timeout("move_body", {"fwd": 1.0}) < 60     # short moves still fail fast


# --- rangefinder height (PlannerConfig.rng): the real drone's takeoff path ------------------
def rng_planner(sim, edge, **cfg):
    p, said, _ = make_planner(sim, edge)
    p.cfg = PlannerConfig(rng=True, **cfg)
    return p, said


async def until(cond, timeout=10.0):
    end = asyncio.get_running_loop().time() + timeout
    while not cond():
        assert asyncio.get_running_loop().time() < end, "timed out"
        await asyncio.sleep(0.05)


async def test_rng_takeoff_holds_at_the_rangefinder_height(rig):
    """It asks the FC for the ceiling but stops the climb once the rangefinder reads the target,
    then keeps holding there."""
    sim, edge = rig
    p, said = rng_planner(sim, edge)
    await run_plan(p, [{"kind": "takeoff", "alt": 1.0}])  # the brain always says 1.0
    assert p.log[0] == "takeoff {'alt': 1.2}" and "stop" in p.log
    assert p._target == pytest.approx(0.5)
    await asyncio.sleep(2.5)
    assert sim.state().alt == pytest.approx(0.5, abs=0.15) and sim.state().armed
    await run_plan(p, [{"kind": "land"}])
    await until(lambda: not sim.state().armed)
    await asyncio.sleep(0.5)
    assert p._guard.done() and p._target is None


async def test_rng_sideways_moves_go_in_legs_and_height_stays_in_range(rig):
    sim, edge = rig
    p, said = rng_planner(sim, edge)
    await run_plan(p, [{"kind": "takeoff"}])
    await run_plan(p, [{"kind": "move", "fwd": 0.0, "right": -5.0, "up": 0.0, "yaw_deg": 0.0}], 90)
    legs = [l for l in p.log if l.startswith("move_body")]
    assert legs == ["move_body {'fwd': 0.0, 'right': -2.0}"] * 2 + ["move_body {'fwd': 0.0, 'right': -1.0}"]
    assert sim.state().e == pytest.approx(-5.0, abs=0.3)
    await run_plan(p, [{"kind": "move", "up": 0.3}])
    assert p._target == pytest.approx(0.8) and sim.state().alt == pytest.approx(0.8, abs=0.15)
    await run_plan(p, [{"kind": "move", "up": 1.0}])  # 1.8 m: past what the rangefinder holds
    assert "only lets me fly up to 1.0" in said[-1] and p._target == pytest.approx(0.8)
    await run_plan(p, [{"kind": "land"}])


async def test_rng_lands_when_the_rangefinder_is_lost(rig):
    sim, edge = rig
    p, said = rng_planner(sim, edge)
    await run_plan(p, [{"kind": "takeoff"}])
    p.rng_height = lambda: None  # the ultrasonic stops echoing (grass, tilt)
    await until(lambda: any("height sensor" in s for s in said), 5)
    await until(lambda: not sim.state().armed, 20)


async def test_rng_lands_when_too_high(rig):
    sim, edge = rig
    p, said = rng_planner(sim, edge)
    await run_plan(p, [{"kind": "takeoff"}])
    p.rng_height = lambda: 1.5
    await until(lambda: any("above the 1.2 metre limit" in s for s in said), 5)
    await until(lambda: not sim.state().armed, 20)


async def test_rng_refuses_takeoff_when_the_rangefinder_isnt_on_the_floor(rig):
    sim, edge = rig
    p, said = rng_planner(sim, edge)
    sim.s.range_m = None  # sim writes range_m = alt only while it steps; on the ground it stays
    await asyncio.sleep(0.3)
    await run_plan(p, [{"kind": "takeoff"}])
    assert "isn't pointing at the floor" in said[-1] and not sim.state().armed


async def test_rng_rise_turn_around_land_does_one_full_turn(rig):
    """'rise, turn around, land' (the filming preset): takeoff, one full scan turn, land.
    In rng mode the turn used to be dropped; a relative yaw of 360 would do nothing."""
    sim, edge = rig
    p, said = rng_planner(sim, edge)
    r = fast_parse("hey drone, rise, turn around, land")
    assert [a["kind"] for a in r.actions] == ["takeoff", "move", "land"] and r.actions[1]["yaw_deg"] == 360
    yaws = []

    async def watch():
        while True:
            yaws.append(sim.state().yaw)
            await asyncio.sleep(0.05)
    w = asyncio.ensure_future(watch())
    await run_plan(p, r.actions, 90)
    w.cancel()
    assert "scan {'rate_dps': 30.0}" in p.log
    turned = sum(abs((b - a + 180) % 360 - 180) for a, b in zip(yaws, yaws[1:]))
    assert turned > 330 and not sim.state().armed


# --- field 2026-11-21 (outdoor, GPS): what went wrong, so it can't again ---------------------
@pytest.mark.parametrize("text, kinds", [
    ("I want you to rise, turn around, and land.", ["takeoff", "move", "land"]),   # went to the LLM
    ("Can you rise, go left, then right, then come down?", ["takeoff", "move", "move", "land"]),
    ("Hey drone, rise, go left 5 metres, then right 5 metres, then come down",
     ["takeoff", "move", "move", "land"]),
])
def test_filming_phrases_skip_the_llm(text, kinds):
    r = fast_parse(text)
    assert r and [a["kind"] for a in r.actions] == kinds


async def test_rng_lands_when_it_sinks_instead_of_holding(rig):
    """It sank from 0.5 m to 5 cm while 'holding' and nothing noticed."""
    sim, edge = rig
    p, said = rng_planner(sim, edge, rng_sink_s=1.0)
    await run_plan(p, [{"kind": "takeoff"}])
    p.rng_height = lambda: 0.05
    await until(lambda: any(s.startswith("I'm sinking") for s in said), 5)
    await until(lambda: not sim.state().armed, 20)


async def test_rng_lands_when_it_turns_on_its_own(rig):
    """It then yawed on its own just above the floor; no turn was ever sent."""
    sim, edge = rig
    p, said = rng_planner(sim, edge)
    await run_plan(p, [{"kind": "takeoff"}])
    await asyncio.sleep(0.6)
    p._yaw = lambda: (sim.state().yaw + 90) % 360
    await until(lambda: any("turning on my own" in s for s in said), 5)
    await until(lambda: not sim.state().armed, 20)


async def test_rng_full_turn_is_not_mistaken_for_drift(rig):
    sim, edge = rig
    p, said = rng_planner(sim, edge)
    await run_plan(p, fast_parse("I want you to rise, turn around, and land.").actions, 90)
    assert "scan {'rate_dps': 30.0}" in p.log and not sim.state().armed
    assert not any("on my own" in s or "pilot" in s for s in said)  # our LAND isn't the pilot


async def test_rng_test3_left_right_and_back(rig):
    """Test 3: rise, 5 m left and 5 m right in 2 m legs, land back near the start."""
    sim, edge = rig
    p, said = rng_planner(sim, edge)
    r = fast_parse("Hey drone, rise, go left 5 metres, then right 5 metres, then come down")
    await run_plan(p, r.actions, 120)
    legs = [l for l in p.log if l.startswith("move_body") and "right" in l]
    assert len(legs) == 6 and abs(sim.state().e) < 0.35 and not sim.state().armed
    assert not any("sinking" in s or "on my own" in s or "pilot" in s for s in said)


async def test_rng_test2_rise_360_then_hover_until_told_to_land(rig):
    """Test 2 as filmed: 'rise, do a 360' (no land), it keeps hovering; 'land' said later lands it."""
    sim, edge = rig
    p, said = rng_planner(sim, edge)
    r = fast_parse("Hey drone, rise and do a 360")
    assert [a["kind"] for a in r.actions] == ["takeoff", "move"] and r.actions[1]["yaw_deg"] == 360
    await run_plan(p, r.actions, 90)
    assert "scan {'rate_dps': 30.0}" in p.log
    await asyncio.sleep(3)  # hovering, nobody has said land
    s = sim.state()
    assert s.armed and s.alt == pytest.approx(0.5, abs=0.15) and not p._guard.done()
    p.submit(fast_parse("land").actions)                     # said mid-flight
    await until(lambda: not sim.state().armed, 30)
    assert not any("sinking" in x or "on my own" in x or "pilot" in x for x in said)


async def test_rng_land_said_in_the_middle_of_the_360(rig):
    sim, edge = rig
    p, said = rng_planner(sim, edge)
    p.submit(fast_parse("rise and do a 360").actions)
    await until(lambda: any(l.startswith("scan") for l in p.log), 30)
    await asyncio.sleep(1.0)                                 # part way round
    p.submit(fast_parse("land").actions)
    await until(lambda: not sim.state().armed, 30)
    assert not any("pilot" in x or "went wrong" in x for x in said)
