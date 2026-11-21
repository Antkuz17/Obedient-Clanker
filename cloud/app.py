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
photo {jpeg, pose, caption}, memory_frame {jpeg, pose}, memory_search {q, req}.
Cloud -> station: actions {actions, reply, text, req}, audio {mp3 b64, text},
memory_result {req, hits}.
"""
from __future__ import annotations

import asyncio
import base64
import contextlib
import itertools
import json
import logging
import time
from pathlib import Path

from fastapi import FastAPI, File, Form, HTTPException, UploadFile, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse, HTMLResponse

from station.ai import AI
from station.brain import Brain
from station.config import MissingKey, Settings, settings as load_settings

from .memory import Memory

log = logging.getLogger("cloud")
STATIC = Path(__file__).parent / "static"
CONTROL_IDLE_S = 120.0
LOG_KEEP = 200


class Hub:
    def __init__(self, s: Settings, ai: AI, memory: Memory):
        self.s, self.ai, self.memory = s, ai, memory
        self.brain = Brain(ai if s.vultr_key else None)
        self.station: WebSocket | None = None
        self.uis: dict[str, WebSocket] = {}
        self.telemetry: dict = {}
        self.last_frame: dict | None = None
        self.flight_log: list[dict] = []
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

    async def command(self, text: str, cid: str) -> dict:
        """A typed or transcribed command from a viewer (or the laptop mic)."""
        text = text.strip()
        if not text:
            return {"ok": False, "reply": "I didn't catch that."}
        await self.to_uis({"type": "heard", "text": text, "who": cid})
        await self.log_line(cid, text)
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

    async def op(self, op: str, cid: str) -> None:
        """STOP / LAND buttons: no brain, any viewer, straight to the station."""
        if op not in ("stop", "land"):
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
    async def voice(audio: UploadFile = File(...), client: str = Form("phone")):
        data = await audio.read()
        try:
            text = await ai.transcribe(data, audio.filename or "clip.webm")
        except MissingKey as e:
            raise HTTPException(503, str(e))
        except Exception as e:
            raise HTTPException(502, f"speech-to-text failed: {e}")
        if not text:
            return {"text": "", "reply": "I didn't catch that."}
        r = await hub.command(text, client)
        return {"text": text, **r}

    @app.post("/text")
    async def text_cmd(body: dict):
        return await hub.command(str(body.get("text", "")), str(body.get("client", "api")))

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

    @app.websocket("/ws/station")
    async def ws_station(ws: WebSocket, token: str = ""):
        if token != s.station_token:
            await ws.close(code=4401)
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
                            "telemetry": hub.telemetry})
        if hub.last_frame:
            await ws.send_json(hub.last_frame)
        try:
            while True:
                m = await ws.receive_json()
                if m.get("type") == "text":
                    hub.spawn(hub.command(str(m.get("text", "")), cid))
                elif m.get("type") == "op":
                    await hub.op(str(m.get("op")), cid)
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
