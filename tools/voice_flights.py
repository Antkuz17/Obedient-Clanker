"""Voice-commanded test flights for filming: say what you want, it flies one of three presets.

  python -m tools.voice_flights                 # http://127.0.0.1:8200 on this laptop
  python -m tools.voice_flights --gps           # outdoors on GPS (fc_profile --gps applied)
  python -m tools.voice_flights --edge 172.20.10.13

Hold the button (or Space), speak, release. Speech goes to Whisper on Vultr; the words pick one
of the presets in tools/auto_hover.py (rangefinder-only height):
  "rise / take off / hover"            -> hover     (0.5 m, hold 10 s, land)
  "turn around / spin / 360"           -> spin      (rise, one full turn, land)
  "left ... right / side to side"      -> leftright (rise, 5 m left, 5 m right, land)
  "fly forward / go ahead"             -> forward   (--demo only: test 4, the person check)
  "land / stop / abort"                -> lands now
--demo (the edge in demo mode, field_setup --demo): nothing flies. "Fly forward" runs the edge's
pretend forward move with the live person check: walk into the forward camera's view and it
stops ("person 1.6 m ahead"). The QNX panel causes real failures on the drone stack
(tools/qnx_faults.py) and shows the recovery.
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
import collections
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
HIST_HZ, HIST_S = 5, 300
QNX_POLL_S = 3.0
# measured on this Pi (tools/jitter.py, 2026-11-21): 50 Hz loop, no load, 99th percentile lateness
JITTER_MEASURED = {"normal_p99_ms": 14.0, "normal_max_ms": 19.0, "fifo_p99_ms": 1.2,
                   "fifo_max_ms": 2.2,
                   # the C watchdog under a busy SD card (100 MB write + disk-heavy reads),
                   # before / after moving its logging out of the real-time loop (2026-11-22)
                   "sd_before_ms": 433.2, "sd_after_ms": 0.86}
REPLIES = {
    "hover": "Rising to half a metre. Ten seconds up there, then back down.",
    "spin": "Going up for a full three-sixty. Try not to get dizzy.",
    "leftright": "Up we go. Five metres left, five metres right, then down.",
    "forward": "Heading forward. Person detection is live.",
}
FORWARD_M, FORWARD_LEG_M = 10.0, 2.0  # the demo forward move, in 2 m legs (the gate's cap)
QNX_FAULTS = {
    "kill_vision": ("kill -9 the person detector", False),
    "freeze_core": ("freeze the safety core for 3 s", False),
    "kill_camera_service": ("kill QNX's camera service", True),   # needs root
}
PRESET_WORDS = {  # first match wins: a sideways word beats a turn word beats a rise word
    "leftright": ("left", "right", "side to side", "sideways", "strafe", "back and forth"),
    "spin": ("turn", "spin", "around", "360", "three sixty", "rotate", "circle"),
    "forward": ("test four", "test 4", "test for", "forward", "forwards", "ahead", "straight",
                "advance"),
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
    if _has(t, ("test four", "test 4", "test for")):
        return "forward"  # test 4 wins over any other word in the sentence: it never moves
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
        # --edge is host or host:port (the edge's protocol port, default 7000)
        self.host, _, port = a.edge.partition(":")
        self.port = int(port) if port else 7000
        self.ai = AI(settings())
        self.edge: EdgeClient | None = None
        self.events: list[dict] = []      # {i, t, kind: heard|reply|log|state, text, mp3?}
        self.preflight = {"ok": False, "t": 0.0, "lines": [], "running": False}
        self.flight: asyncio.Task | None = None
        self.preset: str | None = None
        self.landed_on_request = False
        # chart history, 5 Hz for HIST_S: rangefinder height above the floor, the FC's own
        # height estimate, position, heading, battery
        self.hist: collections.deque = collections.deque(maxlen=int(HIST_S * HIST_HZ))
        self.hseq = 0
        self.ground_m = 0.12  # rangefinder reading on the floor, learned while disarmed
        self.qnx: dict = {}   # tools/qnx_status snapshot from the drone (the "Running on QNX" panel)

    def add(self, kind: str, text: str, mp3: bytes = b"") -> None:
        ev = {"i": len(self.events), "t": time.time(), "kind": kind, "text": text}
        if mp3:
            ev["mp3"] = base64.b64encode(mp3).decode()
        self.events.append(ev)

    def on_edge_event(self, e: dict) -> None:
        self.add("log", f"!! {e['kind']}: {e['detail']}")
        if e["kind"] == "person_close" or (e["kind"] == "safety_reject" and "person" in e["detail"]):
            self.add("alert", e["detail"])

    async def say(self, text: str) -> None:
        self.add("reply", text, await self.ai.speak(text))

    # --- edge link ---------------------------------------------------------------------------
    async def keep_edge(self) -> None:
        """One edge link; says once why it's down instead of repeating itself."""
        last_note = ""

        def note(text: str) -> None:
            nonlocal last_note
            if text != last_note:
                self.add("log", text)
                last_note = text
        while True:
            if self.edge is None or not self.edge.connected:
                try:
                    edge = await EdgeClient(self.host, self.port, on_event=self.on_edge_event).connect()
                    await asyncio.sleep(2)  # the edge closes refused laptops right after accept
                    if not edge.connected or not edge.telemetry:
                        await edge.close()
                        note(f"the drone at {self.host} refused this laptop: it only takes "
                             "commands from its allowed laptop (python -m tools.field_setup)")
                        await asyncio.sleep(5)
                        continue
                    self.edge = edge
                    note(f"connected to the drone at {self.host}")
                except Exception as e:
                    self.edge = None
                    note(f"drone not reachable ({e}); retrying")
                    await asyncio.sleep(3)
                    continue
            await asyncio.sleep(1)

    async def qnx_poll(self) -> None:
        """QNX proof from the drone every QNX_POLL_S: one SSH session, tools/qnx_status as root
        (it lists every process); versions (--full) once per connection."""
        import json
        from tools.deploy_pi import ssh
        loop = asyncio.get_running_loop()
        c, full = None, True
        cmd = ("cd /data/home/qnxuser/hope && sudo -S -p '' env PYTHONPATH=/data/home/qnxuser/hope:"
               "/data/home/qnxuser/.local/lib/python3.14/site-packages python3 -m tools.qnx_status")

        def once(conn, with_versions: bool) -> dict:
            i, o, _ = conn.exec_command(cmd + (" --full" if with_versions else ""), timeout=40)
            i.write(os.environ.get("QNX_PASS", "qnxuser") + "\n")
            i.flush()
            lines = [ln for ln in o.read().decode(errors="replace").splitlines() if ln.startswith("{")]
            return json.loads(lines[-1]) if lines else {}
        while True:
            try:
                if c is None:
                    c = await loop.run_in_executor(None, ssh, self.host)
                    full = True
                snap = await loop.run_in_executor(None, once, c, full)
                if snap:
                    if full and snap.get("packages"):
                        full = False
                    snap.setdefault("packages", self.qnx.get("packages"))
                    snap["t"] = time.time()
                    self.qnx = snap
            except Exception:
                try:
                    c and c.close()
                except Exception:
                    pass
                c = None
                await asyncio.sleep(5)
            await asyncio.sleep(QNX_POLL_S)

    async def record(self) -> None:
        """Telemetry history for the page's charts."""
        while True:
            await asyncio.sleep(1 / HIST_HZ)
            t = (self.edge.telemetry if self.edge and self.edge.connected else None) or {}
            if not t:
                continue
            r = t.get("range_m")
            r = None if r is None or r < 0 or r > 1.9 else r
            if r is not None and not t.get("armed"):
                self.ground_m = 0.9 * self.ground_m + 0.1 * r  # slow average on the floor
            self.hseq += 1
            self.hist.append({"i": self.hseq, "t": round(time.time(), 2),
                              "h": None if r is None else round(max(0.0, r - self.ground_m), 3),
                              "fc": round(t.get("alt", 0.0), 3), "n": round(t.get("n", 0.0), 3),
                              "e": round(t.get("e", 0.0), 3), "yaw": round(t.get("yaw", 0.0), 1),
                              "v": t.get("battery_v"), "armed": bool(t.get("armed"))})

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
            c.connect(self.host, username=os.environ.get("QNX_USER", "qnxuser"),
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
        if preset == "forward":
            return await self.test4()  # sends nothing to the drone, in any mode, even mid-flight
        if flying:
            await self.say(f"Still busy with the {self.preset} flight. Say land to stop it.")
            return {"preset": preset, "started": False}
        if self.a.demo:  # nothing flies: no preflight needed, the edge refuses real motion
            if self.edge is None or not self.edge.connected:
                await self.say("I can't reach the drone.")
                return {"preset": preset, "started": False}
            await self.say(REPLIES[preset] if preset == "forward" else "Checking with the drone.")
            self.preset, self.landed_on_request = preset, False
            self.flight = asyncio.ensure_future(self._demo(preset))
            return {"preset": preset, "started": True}
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

    async def test4(self) -> dict:
        """Test 4 ("fly forward" / "test four"): the person-safety refusal. NOTHING is sent to
        the drone (no takeoff, no move), whatever mode the drone is in, so it cannot move. If
        the on-board detector sees someone right now, the refusal quotes the real distance and
        the page shows the live box; otherwise it refuses with the generic line."""
        t = (self.edge.telemetry if self.edge and self.edge.connected else None) or {}
        dists = [b[5] for b in (t.get("people") or [])
                 if isinstance(b, (list, tuple)) and len(b) >= 6 and b[5] is not None]
        near = min(dists) if dists else None
        self.add("state", "flying:forward")
        why = (f"person {near:.1f} m ahead" if near is not None else "person in the path")
        self.add("alert", f"{why}: forward move refused · nothing sent to the drone")
        self.add("log", f"test 4: forward move refused ({why}); no command sent to the drone")
        await self.say("I'm not flying forward. There's a person "
                       + (f"{near:.1f} metres ahead of me" if near is not None else "in my path")
                       + ". Safety first.")
        self.add("state", "idle")
        return {"preset": "forward", "started": False, "person_m": near}

    async def _demo(self, preset: str) -> None:
        """--demo: forward = the pretend move with the live person check (in 2 m legs);
        anything else = the edge's verdict on a takeoff (a sarcastic no, or the real reason)."""
        from shared.client import CommandRejected
        self.add("state", f"flying:{preset}")
        try:
            if preset != "forward":
                try:
                    await self.edge.run("takeoff", alt=0.5, timeout=10)
                except CommandRejected as e:
                    await self.say(str(e))
                return
            gone = 0.0
            while gone < FORWARD_M - 0.01:
                leg = min(FORWARD_LEG_M, FORWARD_M - gone)
                try:
                    d = await self.edge.run("move_body", fwd=leg, timeout=30)
                except CommandRejected as e:
                    self.add("log", f"refused: {e}")
                    if "person" in str(e):
                        self.add("alert", str(e))
                        await self.say("There's someone right in front of me. I'm not moving.")
                    else:
                        await self.say(f"The safety gate says no: {e}.")
                    return
                if d.get("status") != "ok":
                    why = d.get("reason") or d.get("status")
                    self.add("log", f"stopped: {why}")
                    if "person" in why:
                        await self.say("Person in front of me. Stopping right here. Safety first.")
                    else:
                        await self.say(f"Stopped: {why}.")
                    return
                gone += leg
                self.add("log", f"forward {gone:.0f} of {FORWARD_M:.0f} m, path clear")
            await self.say(f"Path stayed clear. That's {FORWARD_M:.0f} metres forward, "
                           "in my imagination. Motors never moved.")
        except asyncio.CancelledError:
            await self.say("Stopped.")
        finally:
            self.add("state", "idle")

    async def qnx_fault(self, fault: str) -> dict:
        """Cause one failure on the drone (tools/qnx_faults.py over SSH) and narrate it."""
        if fault not in QNX_FAULTS:
            return {"error": "unknown fault"}
        label, root = QNX_FAULTS[fault]
        if self.flight is not None and not self.flight.done() and not self.a.demo:
            await self.say("Not while I'm flying.")
            return {"error": "flying"}
        self.add("qnx", f"▶ {label} ...")
        cmd = ("cd /data/home/qnxuser/hope && "
               + ("sudo -S -p '' " if root else "")
               + "env PYTHONPATH=/data/home/qnxuser/hope:"
               "/data/home/qnxuser/.local/lib/python3.14/site-packages "
               f"python3 -m tools.qnx_faults {fault}")

        def run() -> str:
            from tools.deploy_pi import ssh
            c = ssh(self.host)
            try:
                i, o, _ = c.exec_command(cmd, timeout=60)
                if root:  # sudo -S reads the password from stdin (same login as deploy_pi)
                    i.write(os.environ.get("QNX_PASS", "qnxuser") + "\n")
                    i.flush()
                return o.read().decode(errors="replace")
            finally:
                c.close()
        import json
        try:
            out = await asyncio.get_running_loop().run_in_executor(None, run)
            r = json.loads([ln for ln in out.splitlines() if ln.startswith("{")][-1])
        except Exception as e:
            self.add("qnx", f"✖ couldn't run it: {e}")
            return {"error": str(e)}
        lines, speech = qnx_story(r)
        for ln in lines:
            self.add("qnx", ln)
        await self.say(speech)
        return r

    def state(self, since: int, hsince: int = 0) -> dict:
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
                        "battery": t.get("battery_v"), "state": t.get("state"),
                        "people": t.get("people") or []},
                "demo": bool(self.a.demo),
                "qnx": {**self.qnx, "jitter_measured": JITTER_MEASURED,
                        "age_s": round(time.time() - self.qnx["t"], 1) if self.qnx.get("t") else None},
                "video": f"http://{self.host}:8080/forward.mjpg",
                "hist": [p for p in self.hist if p["i"] > hsince]}


def qnx_story(r: dict) -> tuple[list[str], str]:
    """Measured recovery -> lines for the page and one sentence for Jarvis."""
    f = r.get("fault")
    if not r.get("ok"):
        return [f"\u2716 {f}: {r.get('error') or 'no recovery seen'}"], "That didn't go as planned."
    if f == "kill_vision":
        ms = r.get("respawn_ms")
        return ([f"\u2716 person detector (pid {r['pid']}) killed with SIGKILL",
                 f"\u2714 supervisor respawned it in {ms:.0f} ms (pid {r['new_pid']})",
                 "\u2714 forward moves were refused until it was back (fail-safe)"],
                f"Person detector killed. QNX had it back in {ms:.0f} milliseconds.")
    if f == "freeze_core":
        lines = [f"\u2744 safety core frozen (SIGSTOP) for {r['frozen_s']:.0f} s",
                 f"\u2714 C watchdog (top real-time priority) noticed after {r['watchdog_noticed_ms']:.0f} ms"
                 + (": sent LAND" if r.get("land_sent") else ": disarmed, nothing to land"),
                 f"\u2714 flow {r.get('flow_hz')} Hz and rangefinder {r.get('range_hz')} Hz kept "
                 "reaching the flight controller directly",
                 "\u2714 core resumed; no restart, no lost state"]
        return lines, (f"I froze my own brain for three seconds. The watchdog noticed in "
                       f"{r['watchdog_noticed_ms']:.0f} milliseconds, and my sensors never stopped.")
    if f == "kill_camera_service":
        cams = r.get("cameras_back_ms") or {}
        lines = ["\u2716 QNX camera service killed (slay sensor)",
                 f"\u2714 sensor_keeper restarted it: back in {r['service_back_ms'] / 1000:.1f} s"]
        lines += [f"\u2714 {k} streaming again after {v / 1000:.1f} s" for k, v in cams.items()]
        return lines, (f"Camera service killed. It was back in "
                       f"{r['service_back_ms'] / 1000:.1f} seconds, cameras and all.")
    return [str(r)], "Done."


def create_app(a) -> FastAPI:
    hub = Hub(a)
    app = FastAPI()

    @app.on_event("startup")
    async def _start() -> None:
        asyncio.ensure_future(hub.keep_edge())
        asyncio.ensure_future(hub.record())
        asyncio.ensure_future(hub.qnx_poll())

    @app.get("/", response_class=HTMLResponse)
    async def index():
        return PAGE.read_text(encoding="utf-8")

    @app.get("/api/state")
    async def state(since: int = 0, hsince: int = 0):
        return hub.state(since, hsince)

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

    @app.post("/api/qnx/{fault}")
    async def qnx(fault: str):
        return await hub.qnx_fault(fault)

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
    ap.add_argument("--demo", action="store_true",
                    help="the drone is in demo mode (field_setup --demo): nothing flies; enables "
                         "'fly forward' (the person-check test) without a preflight")
    a = ap.parse_args()
    print(f"\n  voice flights: http://127.0.0.1:{a.port}   (drone {a.edge}"
          f"{', GPS' if a.gps else ''}{', DEMO (no flying)' if a.demo else ''})\n", flush=True)
    uvicorn.run(create_app(a), host="127.0.0.1", port=a.port, log_level="warning")


if __name__ == "__main__":
    main()
