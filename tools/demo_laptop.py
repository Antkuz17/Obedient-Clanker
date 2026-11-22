"""The indoor demo on one laptop, against the REAL drone: cloud app + dashboard + station.

  python -m tools.demo_laptop                 # dashboard http://127.0.0.1:8100
  python -m tools.demo_laptop --edge 192.168.68.131
  python -m tools.demo_laptop --edge <pi> --rng      # fly it: the rangefinder decides height

Like `python -m station.dev`, but the station drives the drone at EDGE_HOST (put the drone in
demo mode first if it mustn't fly: `python -m tools.deploy_pi --stations <ips> --demo`).
The dashboard (on this laptop, so the browser mic works without HTTPS) shows the drone's video,
telemetry and the QNX card (safety-loop jitter, supervisor restarts, jitter benchmark), and
plays Jarvis's replies (turn its voice on); the station itself stays quiet so replies aren't
played twice. Ctrl-C stops everything. Logs: data/demo/*.log.
"""
from __future__ import annotations

import argparse
import os
import subprocess
import sys
import time

from station.config import ROOT, load_dotenv
from station.dev import wait_http

CLOUD = 8100


def main() -> None:
    ap = argparse.ArgumentParser(prog="tools.demo_laptop")
    ap.add_argument("--edge", help="the Pi (default EDGE_HOST from .env)")
    ap.add_argument("--port", type=int, default=CLOUD, help="dashboard port")
    ap.add_argument("--rng", action="store_true",
                    help="station flies with rangefinder height (the real drone)")
    ap.add_argument("--station-audio", action="store_true",
                    help="also play replies from the station (not just the dashboard)")
    a = ap.parse_args()
    load_dotenv()
    edge = a.edge or os.environ.get("EDGE_HOST") or "192.168.68.131"
    logs = ROOT / "data" / "demo"
    logs.mkdir(parents=True, exist_ok=True)
    env = {**os.environ, "DATA_DIR": str(logs),
           "STATION_TOKEN": os.environ.get("STATION_TOKEN") or "dev-token"}
    procs: list[tuple[str, subprocess.Popen]] = []

    def start(name: str, *args: str) -> None:
        f = open(logs / f"{name}.log", "w")
        procs.append((name, subprocess.Popen([sys.executable, *args], cwd=ROOT, env=env,
                                             stdout=f, stderr=subprocess.STDOUT)))

    start("cloud", "-m", "uvicorn", "cloud.app:app", "--port", str(a.port))
    if not wait_http(f"http://127.0.0.1:{a.port}/health"):
        print(f"cloud app didn't start; see {logs / 'cloud.log'}")
    start("station", "-m", "station", "--edge", edge, "--cloud", f"ws://127.0.0.1:{a.port}",
          *([] if a.station_audio else ["--no-audio"]), *(["--rng"] if a.rng else []))
    print(f"\n  dashboard  http://127.0.0.1:{a.port}\n  drone      {edge}  "
          f"(video http://{edge}:8080/forward.mjpg)\n  logs       {logs}\n  Ctrl-C to stop.\n",
          flush=True)
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
                p.terminate()
        time.sleep(1)
        for _, p in procs:
            if p.poll() is None:
                p.kill()


if __name__ == "__main__":
    main()
