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
from station.brain import Brain, rule_parse
from station.camera import Frame, Pose
from station.config import MissingKey, Settings
from station.localize import box_to_goal, locate, parse_box
from station.planner import Planner, compass


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

    async def vision(self, prompt, jpeg, system="", max_tokens=400):
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
    assert rule_parse("turn around", {"armed": True}).actions[0]["yaw_deg"] == 180
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
    pose = Pose(0, 0, 1.0, 0, 0)
    x0, y0, x1, y1 = sim_box("door", pose, K)
    norm = [x0 / 640 * 1000, y0 / 480 * 1000, x1 / 640 * 1000, y1 / 480 * 1000]
    ai = FakeAI(vision_reply=json.dumps({"found": True, "box": norm, "conf": 0.8}))
    hit = await locate(ai, "door", b"jpg", pose, K)
    assert hit["n"] == pytest.approx(3.0, abs=0.15) and hit["conf"] == 0.8
    ai.vision_reply = '{"found": true, "box": [1, 1, 900, 900], "conf": 0.1}'
    assert await locate(ai, "door", b"jpg", pose, K) is None  # low confidence


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
    await run_plan(p, [{"kind": "move", "fwd": 0, "right": 0, "up": 10, "yaw_deg": 0}])
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
                  [tool_call("move", up=0.5, yaw_deg=180)] if "half" in user else
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
