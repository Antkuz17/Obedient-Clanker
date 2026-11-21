"""Vultr cloud: dashboard, voice in, brain, voice out, visual memory, and the station relay.

    uvicorn cloud.app:app --host 0.0.0.0 --port 8000      # behind Caddy for HTTPS (deploy.md)

Links (docs/flow.md):
  phone  --HTTPS-->  POST /voice (audio clip), GET / (dashboard)
  phone  <--WSS-->   /ws/ui       telemetry, video frames, log, replies + audio out
  laptop --WSS-->    /ws/station?token=STATION_TOKEN   (the laptop dials out; NAT-safe)

A spoken or typed command: STT (Vultr) -> brain (Vultr LLM) -> `actions` to the station,
whose planner turns them into gated primitives; the reply is spoken (ElevenLabs) on every
dashboard and on the laptop's speaker (the Nest). STOP and LAND buttons skip the brain and go
straight to the station from any viewer. Only one viewer "has the controls" for everything
else (PRD A5): the first to send a command, until they've been idle for CONTROL_IDLE_S.

Station -> cloud messages: telemetry, frame {jpeg b64, pose}, event, log, say {text},
photo {jpeg, pose, caption}, memory_frame {jpeg, pose}, memory_search {q, req},
qnx {src: supervisor | jitter | edge, kind, ...} (QNX demo stats the station gets from the Pi
over UDP; kept in `Hub.qnx` so a viewer who joins late still sees them).
Cloud -> station: actions {actions, reply, text, req}, audio {mp3 b64, text},
memory_result {req, hits}.
"""
from __future__ import annotations

import asyncio
import base64
import contextlib
import hmac
import itertools
import json
import logging
import time
from pathlib import Path

from fastapi import (FastAPI, File, Form, HTTPException, Request, UploadFile, WebSocket,
                     WebSocketDisconnect)
from fastapi.responses import FileResponse, HTMLResponse
from starlette.requests import HTTPConnection

from station.ai import AI, StreamingSTT
from station.brain import Brain
from station.config import MissingKey, Settings, settings as load_settings

from .memory import Memory

log = logging.getLogger("cloud")
STATIC = Path(__file__).parent / "static"
CONTROL_IDLE_S = 120.0
LOG_KEEP = 200
LOCAL_HOSTS = ("127.0.0.1", "::1", "testclient")


def is_local(conn: HTTPConnection) -> bool:
    """A client on this machine. Behind Caddy every phone arrives from 127.0.0.1, so anything
    that came through a proxy (X-Forwarded-For / Forwarded) is never local."""
    if "x-forwarded-for" in conn.headers or "forwarded" in conn.headers:
        return False
    return conn.client is not None and conn.client.host in LOCAL_HOSTS
QNX_LOOP_KEEP = 120      # 1 Hz edge loop stats: two minutes of graph
QNX_EVENTS_KEEP = 50     # supervisor events
QNX_STOPS_KEEP = 20      # person / palm stop latencies


class QnxState:
    """Latest QNX demo numbers: what the dashboard's QNX card shows."""

    def __init__(self) -> None:
        self.loop: list[dict] = []
        self.jitter: dict[str, dict] = {}   # "hog|prio" -> latest case
        self.events: list[dict] = []        # supervisor start / exit / restart / ...
        self.procs: dict[str, dict] = {}    # name -> {state, pid, restarts, last spawn/ready ms}
        self.stops: list[dict] = []

    def add(self, m: dict) -> None:
        src, kind = m.get("src"), m.get("kind")
        if src == "edge" and kind == "loop":
            self.loop = (self.loop + [m])[-QNX_LOOP_KEEP:]
        elif src == "edge" and kind == "stop_latency":
            self.stops = (self.stops + [m])[-QNX_STOPS_KEEP:]
        elif src == "jitter" and kind == "jitter_case":
            self.jitter[f"{bool(m.get('hog'))}|{m.get('prio', 0)}"] = m
        elif src == "supervisor" and (name := m.get("name")):
            self.events = (self.events + [m])[-QNX_EVENTS_KEEP:]
            p = self.procs.setdefault(name, {"name": name, "restarts": 0})
            if kind in ("start", "restart"):
                p.update(state="up", pid=m.get("pid"))
            if kind == "restart":
                p.update(restarts=m.get("restarts", p["restarts"] + 1),
                         spawn_ms=m.get("spawn_ms"), ready_ms=m.get("ready_ms"), t=m.get("t"))
            elif kind in ("exit", "stopped", "spawn_failed"):
                p["state"] = "down" if kind != "spawn_failed" else "failed"

    def snapshot(self) -> dict:
        return {"type": "qnx_state", "loop": self.loop, "jitter": list(self.jitter.values()),
                "procs": list(self.procs.values()), "events": self.events[-20:],
                "stops": self.stops}


class Hub:
    def __init__(self, s: Settings, ai: AI, memory: Memory):
        self.s, self.ai, self.memory = s, ai, memory
        self.brain = Brain(ai if s.vultr_key else None)
        self.station: WebSocket | None = None
        self.uis: dict[str, WebSocket] = {}
        self.telemetry: dict = {}
        self.last_frame: dict | None = None
        self.flight_log: list[dict] = []
        self.qnx = QnxState()
        self.holder: str | None = None
        self.holder_t = 0.0
        self._req = itertools.count(1)
        self._bg: set[asyncio.Task] = set()

    # --- fan-out -----------------------------------------------------------------------------
    async def to_uis(self, msg: dict) -> None:
        dead = []
        for cid, ws in list(self.uis.items()):
            try:
                await ws.send_json(msg)
            except Exception:
                dead.append(cid)
        for cid in dead:
            self.uis.pop(cid, None)

    async def to_station(self, msg: dict) -> bool:
        if not self.station:
            return False
        try:
            await self.station.send_json(msg)
            return True
        except Exception:
            self.station = None
            return False

    async def log_line(self, who: str, text: str) -> None:
        rec = {"type": "log", "t": time.time(), "who": who, "text": text}
        self.flight_log = (self.flight_log + [rec])[-LOG_KEEP:]
        await self.to_uis(rec)

    def spawn(self, coro) -> None:
        t = asyncio.create_task(coro)
        self._bg.add(t)
        t.add_done_callback(self._bg.discard)

    # --- control -----------------------------------------------------------------------------
    def may_command(self, cid: str) -> bool:
        now = time.monotonic()
        free = (self.holder in (None, cid) or now - self.holder_t > CONTROL_IDLE_S
                or self.holder not in self.uis)  # the holder closed their tab
        if free:
            self.holder, self.holder_t = cid, now
            return True
        return False

    async def announce_control(self) -> None:
        for cid, ws in list(self.uis.items()):
            try:
                await ws.send_json({"type": "control", "holder": self.holder, "you": cid})
            except Exception:
                pass

    # --- speech ------------------------------------------------------------------------------
    async def speak(self, text: str) -> None:
        """Reply text now; the audio follows when TTS returns. Both go to UIs and the station."""
        await self.to_uis({"type": "reply", "text": text})
        await self.log_line("jarvis", text)
        mp3 = await self.ai.speak(text)
        if mp3:
            b64 = base64.b64encode(mp3).decode()
            await self.to_uis({"type": "audio", "mp3": b64, "text": text})
            await self.to_station({"type": "audio", "mp3": b64, "text": text})

    def pin_ok(self, pin, conn: HTTPConnection) -> bool:
        """The pilot PIN gates everything that can move the drone (review3 #4): the viewer
        name is whatever the browser says, so it can't be the lock. No PIN configured: only
        this machine (dev, `station.dev`) may fly."""
        if self.s.control_pin:
            return hmac.compare_digest(str(pin or ""), self.s.control_pin)
        return is_local(conn)

    async def command(self, text: str, cid: str, authorized: bool = True) -> dict:
        """A typed or transcribed command from a viewer (or the laptop mic)."""
        text = text.strip()
        if not text:
            return {"ok": False, "reply": "I didn't catch that."}
        await self.to_uis({"type": "heard", "text": text, "who": cid})
        await self.log_line(cid, text)
        if not authorized:
            reply = "You need the pilot PIN to fly me. Anyone can hit stop."
            await self.log_line("cloud", f"{cid}: no pilot PIN, command not sent")
            self.spawn(self.speak(reply))
            return {"ok": False, "reply": reply, "need_pin": True}
        if not self.may_command(cid):
            reply = "Someone else has the controls right now. You can still hit stop."
            await self.speak(reply)
            return {"ok": False, "reply": reply}
        await self.announce_control()
        r = await self.brain.think(text, self.telemetry)
        req = next(self._req)
        sent = True
        if r.actions:
            sent = await self.to_station({"type": "actions", "actions": r.actions,
                                          "reply": r.reply, "text": text, "req": req})
        reply = r.reply if sent else "I'd love to, but the drone isn't connected."
        if r.actions:
            await self.log_line("plan", json.dumps(r.actions))
        self.spawn(self.speak(reply))
        return {"ok": sent, "reply": reply, "actions": r.actions, "source": r.source}

    async def op(self, op: str, cid: str, authorized: bool = True) -> None:
        """STOP / LAND buttons: no brain, straight to the station. STOP: any viewer. LAND:
        pilot PIN, like everything else that moves the drone (review3 #4)."""
        if op not in ("stop", "land"):
            return
        if op == "land" and not authorized:
            await self.log_line("cloud", f"{cid}: LAND needs the pilot PIN (STOP doesn't)")
            return
        await self.log_line(cid, f"[{op.upper()} button]")
        if not await self.to_station({"type": "actions", "actions": [{"kind": op}],
                                      "reply": "", "text": op, "req": next(self._req)}):
            await self.log_line("cloud", "station not connected")

    # --- from the station --------------------------------------------------------------------
    async def from_station(self, m: dict) -> None:
        t = m.get("type")
        if t == "telemetry":
            self.telemetry = m
            await self.to_uis(m)
        elif t == "frame":
            self.last_frame = m
            await self.to_uis(m)
        elif t == "event":
            await self.to_uis(m)
            await self.log_line("edge", f"{m.get('kind')}: {m.get('detail')}")
        elif t == "log":
            await self.log_line("station", m.get("text", ""))
        elif t == "say":
            self.spawn(self.speak(m.get("text", "")))
        elif t == "photo":
            self.spawn(self._photo(m))
        elif t == "memory_frame":
            self.spawn(self._ingest(m))
        elif t == "qnx":
            self.qnx.add(m)
            await self.to_uis(m)
            if m.get("src") == "supervisor" and m.get("kind") == "restart":
                ready = m.get("ready_ms")
                await self.log_line("qnx", f"{m.get('name')} restarted: respawn {m.get('spawn_ms')} ms"
                                    + (f", serving again {ready} ms" if ready is not None else ""))
            elif m.get("kind") == "stop_latency" and "ms" in m:
                await self.log_line("qnx", f"{m.get('what')} stop: {m['ms']} ms frame to stop")
        elif t == "memory_search":
            hits = await self._search(m.get("q", ""))
            await self.to_station({"type": "memory_result", "req": m.get("req"), "hits": hits})

    async def _photo(self, m: dict) -> None:
        jpeg = base64.b64decode(m["jpeg"])
        try:
            rec = await self.memory.ingest(jpeg, m["pose"], m.get("caption") or "photo",
                                           kind="photo")
        except Exception as e:  # no key: still keep the photo
            log.warning("photo ingest failed: %s", e)
            return
        await self.to_uis({"type": "photo", "url": rec["jpeg_url"], "caption": rec["caption"]})

    async def _ingest(self, m: dict) -> None:
        try:
            await self.memory.ingest(base64.b64decode(m["jpeg"]), m["pose"])
        except MissingKey:
            pass
        except Exception as e:
            log.warning("memory ingest failed: %s", e)

    async def _search(self, q: str) -> list[dict]:
        try:
            return await self.memory.search(q)
        except Exception as e:
            log.warning("memory search failed: %s", e)
            return []


def create_app(s: Settings | None = None, ai: AI | None = None) -> FastAPI:
    s = s or load_settings()
    ai = ai or AI(s)
    hub = Hub(s, ai, Memory(s.data_dir, ai))
    @contextlib.asynccontextmanager
    async def lifespan(_app):
        yield
        await ai.aclose()

    app = FastAPI(title="Hope Drone cloud", lifespan=lifespan)
    app.state.hub = hub

    @app.get("/", response_class=HTMLResponse)
    async def index():
        return (STATIC / "index.html").read_text()

    @app.get("/health")
    async def health():
        return {"ok": True, "station": hub.station is not None, "viewers": len(hub.uis),
                "memory_frames": hub.memory.count(), "keys": {
                    "vultr": bool(s.vultr_key), "elevenlabs": bool(s.eleven_key and s.eleven_voice)}}

    @app.post("/voice")
    async def voice(request: Request, audio: UploadFile = File(...),
                    client: str = Form("phone"), pin: str = Form("")):
        if not hub.pin_ok(pin, request):
            return await hub.command("(voice)", client, authorized=False)  # skip STT: no PIN
        data = await audio.read()
        t0 = time.perf_counter()
        try:
            text = await ai.transcribe(data, audio.filename or "clip.webm")
        except MissingKey as e:
            raise HTTPException(503, str(e))
        except Exception as e:
            raise HTTPException(502, f"speech-to-text failed: {e}")
        if not text:
            return {"text": "", "reply": "I didn't catch that."}
        t1 = time.perf_counter()
        r = await hub.command(text, client)
        stt_ms, plan_ms = round((t1 - t0) * 1000), round((time.perf_counter() - t1) * 1000)
        await hub.log_line("cloud", f"voice latency: speech-to-text {stt_ms} ms, "
                                    f"brain + send {plan_ms} ms ({r.get('source', '?')})")
        return {"text": text, **r, "latency_ms": {"stt": stt_ms, "plan": plan_ms}}

    @app.websocket("/ws/voice")
    async def voice_stream(ws: WebSocket, client: str = "phone", pin: str = ""):
        """Push-to-talk, streamed: binary frames of 24 kHz mono s16le PCM while the button is
        held, then the text "end". Transcribed live (StreamingSTT), so the command runs ~0.4 s
        after release. Replies with the same JSON as POST /voice, or {"error"} (the dashboard
        then falls back to uploading the clip to /voice)."""
        await ws.accept()
        stt = None
        try:
            if not hub.pin_ok(pin, ws):  # no PIN: don't transcribe, just say so on release
                while (m := await ws.receive())["type"] != "websocket.disconnect":
                    if m.get("text") == "end":
                        break
                else:
                    return
                await ws.send_json(await hub.command("(voice)", client, authorized=False))
                return
            try:
                stt = await StreamingSTT.open(s)
            except Exception as e:
                await ws.send_json({"error": f"streaming speech-to-text unavailable: {e}"})
                return
            while True:
                m = await ws.receive()
                if m["type"] == "websocket.disconnect":
                    return
                if m.get("bytes"):
                    await stt.send(m["bytes"])
                elif m.get("text") == "end":
                    break
            t0 = time.perf_counter()
            try:
                text = await stt.finish()
            except Exception as e:
                await ws.send_json({"error": f"speech-to-text failed: {e}"})
                return
            if not text:
                await ws.send_json({"text": "", "reply": "I didn't catch that."})
                return
            t1 = time.perf_counter()
            r = await hub.command(text, client)
            stt_ms, plan_ms = round((t1 - t0) * 1000), round((time.perf_counter() - t1) * 1000)
            await hub.log_line("cloud", f"voice latency (streamed): speech-to-text {stt_ms} ms "
                                        f"after release, brain + send {plan_ms} ms "
                                        f"({r.get('source', '?')})")
            await ws.send_json({"text": text, **r, "latency_ms": {"stt": stt_ms, "plan": plan_ms}})
        except (WebSocketDisconnect, RuntimeError):
            pass
        finally:
            if stt:
                await stt.close()
            with contextlib.suppress(Exception):
                await ws.close()

    @app.post("/text")
    async def text_cmd(request: Request, body: dict):
        ok = hub.pin_ok(body.get("pin"), request)
        return await hub.command(str(body.get("text", "")), str(body.get("client", "api")),
                                 authorized=ok)

    @app.post("/memory/reset")
    async def memory_reset(request: Request, body: dict):
        """Forget every memory frame, e.g. after rehearsing on the sim (review3)."""
        if not hub.pin_ok(body.get("pin"), request):
            raise HTTPException(403, "pilot PIN required")
        return {"deleted": hub.memory.clear()}

    @app.post("/memory/search")
    async def memory_search(body: dict):
        return await hub._search(str(body.get("q", "")))

    @app.get("/memory/recent")
    async def memory_recent(kind: str | None = None):
        return hub.memory.recent(kind)

    @app.get("/memory/img/{fid}.jpg")
    async def memory_img(fid: int):
        p = hub.memory.jpeg_path(fid)
        if not p.exists():
            raise HTTPException(404)
        return FileResponse(p, media_type="image/jpeg")

    @app.get("/log")
    async def flight_log():
        return hub.flight_log

    @app.get("/qnx")
    async def qnx_state():
        return hub.qnx.snapshot()

    @app.websocket("/ws/station")
    async def ws_station(ws: WebSocket, token: str = ""):
        local = is_local(ws)  # not via the proxy: the dev token is localhost only
        if token != s.station_token or (s.station_token == "dev-token" and not local):
            await ws.close(code=4401)  # the default token only works on localhost
            return
        await ws.accept()
        if hub.station:
            try:
                await hub.station.close(code=4409)
            except Exception:
                pass
        hub.station = ws
        await hub.to_uis({"type": "station", "connected": True})
        await hub.log_line("cloud", "station connected")
        try:
            while True:
                await hub.from_station(await ws.receive_json())
        except (WebSocketDisconnect, RuntimeError):
            pass
        finally:
            if hub.station is ws:
                hub.station = None
                await hub.to_uis({"type": "station", "connected": False})
                await hub.log_line("cloud", "station disconnected")

    @app.websocket("/ws/ui")
    async def ws_ui(ws: WebSocket, client: str = ""):
        await ws.accept()
        cid = client or f"viewer{len(hub.uis) + 1}"
        hub.uis[cid] = ws
        await ws.send_json({"type": "hello", "you": cid, "station": hub.station is not None,
                            "holder": hub.holder, "log": hub.flight_log[-50:],
                            "pin_required": bool(s.control_pin),
                            "telemetry": hub.telemetry})
        if hub.last_frame:
            await ws.send_json(hub.last_frame)
        await ws.send_json(hub.qnx.snapshot())
        try:
            while True:
                m = await ws.receive_json()
                ok = hub.pin_ok(m.get("pin"), ws)
                if m.get("type") == "text":
                    hub.spawn(hub.command(str(m.get("text", "")), cid, authorized=ok))
                elif m.get("type") == "op":
                    await hub.op(str(m.get("op")), cid, authorized=ok)
        except (WebSocketDisconnect, RuntimeError):
            pass
        finally:
            if hub.uis.get(cid) is ws:
                hub.uis.pop(cid, None)

    return app


def __getattr__(name: str):  # `uvicorn cloud.app:app` builds the app on first use
    if name == "app":
        globals()["app"] = a = create_app()
        return a
    raise AttributeError(name)
