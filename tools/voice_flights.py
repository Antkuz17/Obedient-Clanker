"""Voice-commanded test flights for filming: say what you want, it flies one of three presets.

  python -m tools.voice_flights                 # http://127.0.0.1:8200 on this laptop
  python -m tools.voice_flights --gps           # outdoors on GPS (fc_profile --gps applied)
  python -m tools.voice_flights --edge 172.20.10.13

Hold the button (or Space), speak, release. Speech goes to Whisper on Vultr; the words pick one
of the presets in tools/auto_hover.py (rangefinder-only height):
  "rise / take off / hover"            -> hover     (0.5 m, hold 10 s, land)
  "turn around / spin / 360"           -> spin      (rise, one full turn, land)
  "left ... right / side to side"      -> leftright (rise, 5 m left, 5 m right, land)
  "land / stop / abort"                -> lands now
Jarvis answers aloud (ElevenLabs, played by the page). Flights need a preflight GO from the
last 5 minutes (button on the page), and the safety pilot keeps SwA. This is the filming
shortcut until the full brain/planner flies the real drone; it never sends anything the edge's
safety gate hasn't approved. Holds the only station connection to the edge, so don't run the
station or the dashboard at the same time.
"""
from __future__ import annotations

import argparse
import asyncio
import base64
import contextlib
import io
import os
import re
import time
from pathlib import Path

import uvicorn
from fastapi import FastAPI, Request
from fastapi.responses import HTMLResponse, JSONResponse

from shared.client import EdgeClient
from station.ai import AI
from station.config import settings
from tools import auto_hover

PAGE = Path(__file__).with_name("voice_flights.html")
PREFLIGHT_FRESH_S = 300
REPLIES = {
    "hover": "Rising to half a metre. Ten seconds up there, then back down.",
    "spin": "Going up for a full three-sixty. Try not to get dizzy.",
    "leftright": "Up we go. Five metres left, five metres right, then down.",
}
PRESET_WORDS = {  # first match wins: a sideways word beats a turn word beats a rise word
    "leftright": ("left", "right", "side to side", "sideways", "strafe", "back and forth"),
    "spin": ("turn", "spin", "around", "360", "three sixty", "rotate", "circle"),
    "hover": ("rise", "take off", "takeoff", "lift", "lift off", "hover", "fly", "go up", "up"),
}
LAND_WORDS = ("land", "stop", "abort", "down", "emergency", "cancel")


def _has(t: str, words) -> bool:
    return any(re.search(rf"\b{re.escape(w)}\b", t) for w in words)  # "alright" isn't "right"


def classify(text: str, flying: bool = False) -> str | None:
    """While flying, any stop word lands. On the ground, "rise, turn around, land" is the spin
    preset (it lands at the end anyway); a stop word alone means land."""
    t = text.lower().replace("-", " ")
    if flying and _has(t, LAND_WORDS):
        return "land"
    for preset, words in PRESET_WORDS.items():
        if _has(t, words):
            return preset
    return "land" if _has(t, LAND_WORDS) else None


class _Lines(io.TextIOBase):
    """stdout -> the page's flight log (auto_hover prints its progress)."""
    def __init__(self, sink):
        self.sink, self.buf = sink, ""

    def write(self, s: str) -> int:
        self.buf += s
        while "\n" in self.buf:
            line, self.buf = self.buf.split("\n", 1)
            if line.strip():
                self.sink(line.rstrip())
        return len(s)


class Hub:
    def __init__(self, a):
        self.a = a
        self.ai = AI(settings())
        self.edge: EdgeClient | None = None
        self.events: list[dict] = []      # {i, t, kind: heard|reply|log|state, text, mp3?}
        self.preflight = {"ok": False, "t": 0.0, "lines": [], "running": False}
        self.flight: asyncio.Task | None = None
        self.preset: str | None = None
        self.landed_on_request = False

    def add(self, kind: str, text: str, mp3: bytes = b"") -> None:
        ev = {"i": len(self.events), "t": time.time(), "kind": kind, "text": text}
        if mp3:
            ev["mp3"] = base64.b64encode(mp3).decode()
        self.events.append(ev)

    async def say(self, text: str) -> None:
        self.add("reply", text, await self.ai.speak(text))

    # --- edge link ---------------------------------------------------------------------------
    async def keep_edge(self) -> None:
        while True:
            if self.edge is None or not self.edge.connected:
                try:
                    self.edge = await EdgeClient(self.a.edge, on_event=lambda e: self.add(
                        "log", f"!! {e['kind']}: {e['detail']}")).connect()
                    self.add("log", f"connected to the drone at {self.a.edge}")
                except Exception as e:
                    self.edge = None
                    self.add("log", f"drone not reachable ({e}); retrying")
                    await asyncio.sleep(3)
                    continue
            await asyncio.sleep(1)

    # --- preflight ---------------------------------------------------------------------------
    async def run_preflight(self) -> None:
        if self.preflight["running"]:
            return
        self.preflight.update(running=True, lines=["running ..."])
        cmd = ("cd /data/home/qnxuser/hope && env PYTHONPATH=/data/home/qnxuser/hope:"
               "/data/home/qnxuser/.local/lib/python3.14/site-packages python3 -m tools.preflight "
               f"--min-batt {self.a.min_batt}" + (" --gps" if self.a.gps else ""))

        def ssh() -> str:
            import paramiko
            c = paramiko.SSHClient()
            c.set_missing_host_key_policy(paramiko.AutoAddPolicy())
            c.connect(self.a.edge, username=os.environ.get("QNX_USER", "qnxuser"),
                      password=os.environ.get("QNX_PASS", "qnxuser"), look_for_keys=False,
                      allow_agent=False, timeout=10)
            try:
                _, o, _ = c.exec_command(cmd, timeout=60)
                return o.read().decode(errors="replace")
            finally:
                c.close()
        try:
            out = await asyncio.get_running_loop().run_in_executor(None, ssh)
        except Exception as e:
            out = f"NO-GO preflight could not run: {e}"
        lines = [ln for ln in out.splitlines() if ln.startswith(("GO", "NO-GO"))]
        ok = ">>> GO <<<" in out
        self.preflight.update(ok=ok, t=time.time(), lines=lines, running=False)
        self.add("log", "preflight: ALL GO" if ok else "preflight: NO-GO: " + "; ".join(
            ln[6:].strip() for ln in lines if ln.startswith("NO-GO")))

    # --- commands ----------------------------------------------------------------------------
    async def command(self, text: str) -> dict:
        self.add("heard", text)
        flying = self.flight is not None and not self.flight.done()
        preset = classify(text, flying)
        if preset == "land":
            if flying:
                self.landed_on_request = True
                self.flight.cancel()  # auto_hover's finally lands it
                await self.say("Landing now.")
            else:
                await self.say("I'm already on the ground.")
            return {"preset": "land"}
        if preset is None:
            await self.say("I can rise and hover, do a full turn, or go left and right. "
                           "Pick one.")
            return {"preset": None}
        if flying:
            await self.say(f"Still busy with the {self.preset} flight. Say land to stop it.")
            return {"preset": preset, "started": False}
        fresh = self.preflight["ok"] and time.time() - self.preflight["t"] < PREFLIGHT_FRESH_S
        if not fresh:
            await self.say("Run the preflight first. I don't fly blind.")
            return {"preset": preset, "started": False}
        if self.edge is None or not self.edge.connected:
            await self.say("I can't reach the drone.")
            return {"preset": preset, "started": False}
        await self.say(REPLIES[preset])
        self.preset, self.landed_on_request = preset, False
        self.flight = asyncio.ensure_future(self._fly(preset))
        return {"preset": preset, "started": True}

    async def _fly(self, preset: str) -> None:
        a = argparse.Namespace(alt=0.5, hold=10.0, ground=None, move=None, rng=True,
                               test=preset, dist=5.0, max_leg=2.0, spin_rate=30.0,
                               ceiling=1.2, max_alt=1.2)
        self.add("state", f"flying:{preset}")
        rc = 1
        with contextlib.redirect_stdout(_Lines(lambda s: self.add("log", s))):
            try:
                rc = await auto_hover.fly(self.edge, a)
            except Exception as e:
                print(f"flight error: {e!r}")
        self.preflight["ok"] = False  # a new flight needs a new preflight (battery, sensors)
        self.add("state", "idle")
        if self.landed_on_request:
            await self.say("Down safely, as requested.")
        else:
            await self.say("Back on the ground. That's a wrap." if rc == 0 else
                           "That one didn't go to plan. I'm down; check the log.")

    def state(self, since: int) -> dict:
        t = (self.edge.telemetry if self.edge else None) or {}
        r = t.get("range_m")
        return {"events": self.events[since:], "next": len(self.events),
                "link": bool(self.edge and self.edge.connected),
                "flying": self.flight is not None and not self.flight.done(),
                "preset": self.preset,
                "preflight": {k: self.preflight[k] for k in ("ok", "t", "lines", "running")},
                "preflight_fresh": self.preflight["ok"] and
                time.time() - self.preflight["t"] < PREFLIGHT_FRESH_S,
                "tel": {"height": None if r is None or r > 1.9 else round(r, 2),
                        "mode": t.get("mode"), "armed": t.get("armed"),
                        "battery": t.get("battery_v"), "state": t.get("state")},
                "video": f"http://{self.a.edge}:8080/forward.mjpg"}


def create_app(a) -> FastAPI:
    hub = Hub(a)
    app = FastAPI()

    @app.on_event("startup")
    async def _start() -> None:
        asyncio.ensure_future(hub.keep_edge())

    @app.get("/", response_class=HTMLResponse)
    async def index():
        return PAGE.read_text(encoding="utf-8")

    @app.get("/api/state")
    async def state(since: int = 0):
        return hub.state(since)

    @app.post("/api/voice")
    async def voice(request: Request):
        wav = await request.body()
        t0 = time.monotonic()
        try:
            text = await hub.ai.transcribe(wav, "clip.wav")
        except Exception as e:
            hub.add("log", f"speech-to-text failed: {e}")
            return JSONResponse({"error": str(e)}, 502)
        hub.add("log", f"speech-to-text {time.monotonic() - t0:.1f} s")
        if not text:
            await hub.say("I didn't catch that.")
            return {"text": ""}
        return {"text": text, **(await hub.command(text))}

    @app.post("/api/text")
    async def text(body: dict):
        return await hub.command(str(body.get("text", "")))

    @app.post("/api/preflight")
    async def preflight():
        asyncio.ensure_future(hub.run_preflight())
        return {"started": True}

    @app.post("/api/land")
    async def land():
        return await hub.command("land")

    app.state.hub = hub
    return app


def main() -> None:
    from station.config import load_dotenv
    load_dotenv()
    ap = argparse.ArgumentParser(prog="tools.voice_flights")
    ap.add_argument("--edge", default=os.environ.get("EDGE_HOST", "192.168.68.131"))
    ap.add_argument("--port", type=int, default=8200)
    ap.add_argument("--gps", action="store_true", help="preflight checks GPS (outdoors)")
    ap.add_argument("--min-batt", type=float, default=11.2)
    a = ap.parse_args()
    print(f"\n  voice flights: http://127.0.0.1:{a.port}   (drone {a.edge}"
          f"{', GPS' if a.gps else ''})\n", flush=True)
    uvicorn.run(create_app(a), host="127.0.0.1", port=a.port, log_level="warning")


if __name__ == "__main__":
    main()
