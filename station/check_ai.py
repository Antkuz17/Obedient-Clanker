"""Check every hosted model on the real keys, and time the voice path. Run it the moment the
keys are in `.env` (docs/TODO.md "CRITICAL"):

    python -m station.check_ai            # everything
    python -m station.check_ai --only brain,vision

Each check prints PASS / FAIL / SKIP with its latency. Exits 1 if a critical check fails
(brain, speech-to-text, vision). No drone needed: vision runs on a frame rendered from the sim
room, where the true object positions are known.
"""
from __future__ import annotations

import argparse
import asyncio
import shutil
import subprocess
import sys
import tempfile
import time
from pathlib import Path

import httpx

from .ai import AI
from .brain import Brain
from .camera import Pose
from .config import Settings, settings
from .localize import locate
from . import vision

CRITICAL = {"brain", "stt", "vision"}
results: list[tuple[str, str, str, float]] = []  # name, PASS/FAIL/SKIP, detail, seconds


def record(name: str, status: str, detail: str, secs: float = 0.0) -> None:
    results.append((name, status, detail, secs))
    mark = {"PASS": "✅", "FAIL": "❌", "SKIP": "⏭ "}[status]
    print(f"{mark} {name:<14} {secs:5.2f} s  {detail}", flush=True)


async def timed(coro):
    t0 = time.monotonic()
    out = await coro
    return out, time.monotonic() - t0


def sim_frame(n=0.0, e=0.0, alt=1.0, yaw=0.0) -> tuple[bytes, Pose, dict]:
    from edge.vehicle import VehicleState
    from edge.video import SimCamera
    st = VehicleState(connected=True, armed=True, n=n, e=e, alt=alt, yaw=yaw)
    cam = SimCamera(lambda: st)
    jpg, _, t = cam.capture()
    return jpg, Pose(n, e, alt, yaw, t), cam.K


def text_card(text: str) -> bytes:
    import cv2
    import numpy as np
    img = np.full((480, 640, 3), 255, np.uint8)
    cv2.putText(img, text, (40, 260), cv2.FONT_HERSHEY_SIMPLEX, 2.0, (0, 0, 0), 4)
    return cv2.imencode(".jpg", img)[1].tobytes()


async def check_models(ai: AI, s: Settings) -> None:
    try:
        r, dt = await timed(ai.http.get(f"{s.vultr_base}/models", headers=ai._vultr_headers()))
        r.raise_for_status()
        have = {m["id"] for m in r.json().get("data", [])}
        want = {"brain": s.brain_model, "backup": s.brain_backup_model, "vision": s.vision_model,
                "stt": s.stt_model, "embed": s.embed_model, "rerank": s.rerank_model,
                "tts": s.vultr_tts_model}
        missing = [f"{k}={v}" for k, v in want.items() if v not in have]
        record("models", "FAIL" if missing else "PASS",
               f"missing from catalog: {', '.join(missing)}" if missing else
               f"all {len(want)} configured models listed", dt)
    except Exception as e:
        record("models", "FAIL", f"GET /models: {e}")


async def check_brain(ai: AI) -> None:
    cases = [("go up a little and turn around", {"move"}),
             ("fly to the red poster and read it, then come back", {"mission"}),
             ("crash into the wall", set()),
             ("where did I leave my keys?", {"recall"})]
    ok = True
    for text, want in cases:
        try:
            r, dt = await timed(Brain(ai).think(text, {"state": "HOVER", "armed": True,
                                                       "alt": 1.0}))
        except Exception as e:
            record("brain", "FAIL", f"{text!r}: {e}")
            return
        kinds = {a["kind"] for a in r.actions}
        good = r.source == "llm" and (kinds & want if want else not kinds)
        ok &= bool(good)
        record("brain", "PASS" if good else "FAIL",
               f"{text!r} -> {sorted(kinds) or 'refusal'} | {r.reply!r}"
               + ("" if r.source == "llm" else " (fell back to rules!)"), dt)


async def check_vision(ai: AI) -> None:
    for name, n, e, yaw in (("door", 3.0, 0.0, 0.0), ("chair", -1.0, 2.0, 116.6)):
        jpg, pose, K = sim_frame(yaw=yaw)
        try:
            hit, dt = await timed(locate(ai, name, jpg, pose, K))
        except Exception as e:
            record("vision", "FAIL", f"locate({name}): {e}")
            continue
        if not hit:
            record("vision", "FAIL", f"locate({name}) found nothing (VLM gave no usable box)", dt)
            continue
        err = ((hit["n"] - n) ** 2 + (hit["e"] - e) ** 2) ** 0.5
        record("vision", "PASS" if err < 0.8 else "FAIL",
               f"locate({name}) error {err:.2f} m (conf {hit['conf']:.2f})", dt)
    try:
        ans, dt = await timed(vision.ask(ai, "read", text_card("EXIT 42")))
        record("vision", "PASS" if "42" in ans and "EXIT" in ans.upper() else "FAIL",
               f"read -> {ans!r}", dt)
        ans, dt = await timed(vision.ask(ai, "solve", text_card("12 x 7 = ?")))
        record("vision", "PASS" if "84" in ans else "FAIL", f"solve -> {ans!r}", dt)
    except Exception as e:
        record("vision", "FAIL", f"read/solve: {e}")


async def check_memory(ai: AI) -> None:
    try:
        v, dt = await timed(ai.embed(["silver keys on a table"]))
        record("embed", "PASS", f"dim {len(v[0])}", dt)
    except Exception as e:
        record("embed", "FAIL", str(e))
    try:
        rr, dt = await timed(ai.rerank("my keys", ["a green plant", "silver keys on a table"]))
        record("rerank", "PASS" if rr[1] > rr[0] else "FAIL", f"scores {rr}", dt)
    except Exception as e:
        record("rerank", "SKIP", f"no /rerank ({e}); memory search uses first-stage order")


async def check_voice(ai: AI, s: Settings) -> None:
    phrase = "Fly to the door and take a photo."
    audio, tts_dt, how = b"", 0.0, ""
    if s.eleven_key and s.eleven_voice:
        try:
            audio, tts_dt = await timed(ai.speak(phrase))
            how = "ElevenLabs"
            record("tts", "PASS" if audio else "FAIL", f"ElevenLabs {len(audio)} bytes", tts_dt)
        except Exception as e:
            record("tts", "FAIL", f"ElevenLabs: {e}")
    else:
        record("tts", "SKIP", "ELEVENLABS_API_KEY / ELEVENLABS_VOICE_ID not set")
    try:
        r, dt = await timed(ai._post("tts", "/audio/speech", json={
            "model": s.vultr_tts_model, "input": phrase, "voice": "default",
            "response_format": "mp3"}))
        record("tts-vultr", "PASS", f"{s.vultr_tts_model} {len(r.content)} bytes", dt)
        if not audio:
            audio, tts_dt, how = r.content, dt, s.vultr_tts_model
    except Exception as e:
        record("tts-vultr", "SKIP", f"{s.vultr_tts_model}: {e}")
    if not audio and sys.platform == "darwin" and shutil.which("say"):
        with tempfile.TemporaryDirectory() as d:
            p = Path(d) / "x.aiff"
            subprocess.run(["say", "-o", str(p), phrase], check=True)
            audio, how = p.read_bytes(), "macOS say"
    if not audio:
        record("stt", "SKIP", "no audio source to test with")
        return
    if not shutil.which("ffmpeg"):
        print("   note: ffmpeg not installed; audio goes to Whisper unconverted")
    try:
        text, dt = await timed(ai.transcribe(audio, "clip.mp3" if how != "macOS say" else "clip.aiff"))
        good = "door" in text.lower() and "photo" in text.lower()
        record("stt", "PASS" if good else "FAIL", f"({how} audio) -> {text!r}", dt)
        brain = next((r[3] for r in results if r[0] == "brain"), 0.8)
        total = 0.3 + dt + brain + 0.2 + 0.1
        record("voice e2e", "PASS" if total < 3.0 else "FAIL",
               f"estimated voice->moving {total:.1f} s (upload 0.3 + STT + brain + link 0.3); "
               f"reply audio {tts_dt:.1f} s", total)
    except Exception as e:
        record("stt", "FAIL", str(e))


async def main(only: set[str] | None, s: Settings | None = None, ai: AI | None = None) -> int:
    s = s or settings()
    if not s.vultr_key:
        print("VULTR_INFERENCE_KEY is not set in .env: nothing to check (docs/TODO.md CRITICAL).")
        return 1
    ai = ai or AI(s, timeout=30)
    checks = {"models": lambda: check_models(ai, s), "brain": lambda: check_brain(ai),
              "vision": lambda: check_vision(ai), "memory": lambda: check_memory(ai),
              "voice": lambda: check_voice(ai, s)}
    for name, fn in checks.items():
        if not only or name in only:
            print(f"--- {name}")
            await fn()
    await ai.aclose()
    failed = {r[0] for r in results if r[1] == "FAIL"}
    bad = failed & (CRITICAL | {"voice e2e"})
    print(f"\n{sum(r[1] == 'PASS' for r in results)} passed, "
          f"{sum(r[1] == 'FAIL' for r in results)} failed, "
          f"{sum(r[1] == 'SKIP' for r in results)} skipped"
          + (f"; CRITICAL failures: {', '.join(sorted(bad))}" if bad else ""))
    return 1 if bad else 0


if __name__ == "__main__":
    ap = argparse.ArgumentParser(prog="station.check_ai")
    ap.add_argument("--only", help="comma list: models,brain,vision,memory,voice")
    a = ap.parse_args()
    sys.exit(asyncio.run(main(set(a.only.split(",")) if a.only else None)))
