"""The whole stack on one laptop, for building and rehearsing the Vultr demo with no drone:

    python -m station.dev            # sim edge + sim video + cloud + station
    python -m station.dev --no-station   # run the station yourself (e.g. with -v)

Then open http://127.0.0.1:8100 (dashboard: type commands; the mic needs HTTPS or localhost).
Ports avoid macOS AirPlay on :7000. Ctrl-C stops everything. Logs: data/dev/*.log.
"""
from __future__ import annotations

import argparse
import os
import signal
import subprocess
import sys
import time
import urllib.request
from pathlib import Path

from .config import ROOT, load_dotenv

EDGE, POSE, VIDEO, CLOUD = 7100, 7113, 8180, 8100


def wait_http(url: str, secs: float = 15) -> bool:
    end = time.time() + secs
    while time.time() < end:
        try:
            urllib.request.urlopen(url, timeout=1).read()
            return True
        except Exception:
            time.sleep(0.3)
    return False


def main() -> None:
    ap = argparse.ArgumentParser(prog="station.dev")
    ap.add_argument("--no-station", action="store_true")
    ap.add_argument("--audio", action="store_true", help="play replies on this laptop")
    ap.add_argument("--rng", action="store_true",
                    help="station flies with rangefinder height, like the real drone (--rng)")
    a = ap.parse_args()
    load_dotenv()

    def _term(*_):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, _term)
    signal.signal(signal.SIGINT, _term)  # also when started in the background (SIGINT ignored)
    logs = ROOT / "data" / "dev"
    logs.mkdir(parents=True, exist_ok=True)
    env = {**os.environ, "DATA_DIR": str(ROOT / "data" / "dev"),
           "STATION_TOKEN": os.environ.get("STATION_TOKEN") or "dev-token"}
    py = sys.executable
    procs: list[tuple[str, subprocess.Popen]] = []

    def start(name: str, *args: str) -> None:
        f = open(logs / f"{name}.log", "w")
        procs.append((name, subprocess.Popen([py, *args], cwd=ROOT, env=env, stdout=f,
                                             stderr=subprocess.STDOUT)))

    start("edge", "-m", "edge", "--sim", "--port", str(EDGE), "--video-proc", f"127.0.0.1:{POSE}",
          "--stats-udp", "127.0.0.1:7010")  # QNX card: live loop jitter
    start("video", "-m", "edge.video", "--pose-port", str(POSE), "--camera", "sim",
          "--port", str(VIDEO))
    start("cloud", "-m", "uvicorn", "cloud.app:app", "--port", str(CLOUD))
    ok = wait_http(f"http://127.0.0.1:{CLOUD}/health") and wait_http(
        f"http://127.0.0.1:{VIDEO}/camera.json")
    if not ok:
        print(f"something didn't start; see {logs}/*.log")
    if not a.no_station:
        start("station", "-m", "station", "--edge", f"127.0.0.1:{EDGE}", "--video-port",
              str(VIDEO), "--cloud", f"ws://127.0.0.1:{CLOUD}", "--stub-locate",
              *([] if a.audio else ["--no-audio"]), *(["--rng"] if a.rng else []))
    keys = "with Vultr key" if os.environ.get("VULTR_INFERENCE_KEY") else \
        "NO Vultr key: rule-based brain, no vision/memory"
    print(flush=True)
    print(f"\n  dashboard  http://127.0.0.1:{CLOUD}   ({keys})\n"
          f"  edge       127.0.0.1:{EDGE}  (python -m tools.cmd 127.0.0.1:{EDGE})\n"
          f"  video      http://127.0.0.1:{VIDEO}/forward.mjpg\n"
          f"  logs       {logs}/\n  Ctrl-C to stop.\n", flush=True)
    try:
        while True:
            for name, p in procs:
                if p.poll() is not None:
                    print(f"{name} exited ({p.returncode}); see {logs / (name + '.log')}")
                    raise KeyboardInterrupt
            time.sleep(1)
    except KeyboardInterrupt:
        pass
    finally:
        for _, p in reversed(procs):
            if p.poll() is None:
                p.send_signal(signal.SIGINT)
        time.sleep(1)
        for _, p in procs:
            if p.poll() is None:
                p.kill()


if __name__ == "__main__":
    main()
