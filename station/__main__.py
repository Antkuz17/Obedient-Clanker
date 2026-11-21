"""Run the laptop station.

  python -m station --edge 127.0.0.1:7100 --local --stub-locate   # sim edge (edge --sim --port 7100)
  python -m station --edge <pi ip>                           # real drone, via the Vultr cloud
  python -m station --edge <pi ip> --local --mic             # no cloud, laptop mic

Keys and URLs come from .env (see .env.example).
"""
from __future__ import annotations

import argparse
import asyncio
import logging
import os

from .config import settings
from .station import Station

if __name__ == "__main__":
    ap = argparse.ArgumentParser(prog="station")
    ap.add_argument("--edge", help="edge (Pi) host[:port]; default EDGE_HOST or 127.0.0.1")
    ap.add_argument("--video-port", type=int, help="edge video port; default 8080")
    ap.add_argument("--cloud", help="cloud URL, e.g. wss://drone.example.com; default CLOUD_URL")
    ap.add_argument("--local", action="store_true", help="no cloud: brain/STT/TTS here")
    ap.add_argument("--mic", action="store_true", help="local mode: Enter = push-to-talk")
    ap.add_argument("--stub-locate", action="store_true",
                    help="find objects from the sim room's ground truth (python -m edge --sim)")
    ap.add_argument("--qnx-port", type=int, default=7010,
                    help="UDP port for QNX demo stats from the Pi (0 = off)")
    ap.add_argument("--no-audio", action="store_true", help="don't play replies")
    ap.add_argument("-v", action="store_true")
    a = ap.parse_args()
    logging.basicConfig(level=logging.DEBUG if a.v else logging.INFO,
                        format="%(asctime)s %(name)s %(message)s", datefmt="%H:%M:%S")
    logging.getLogger("httpx").setLevel(logging.WARNING)
    s = settings()
    if a.edge:
        host, _, port = a.edge.partition(":")
        s.edge_host = host
        if port:
            s.edge_port = int(port)
    if a.video_port:
        s.video_port = a.video_port
    if a.cloud:
        s.cloud_url = a.cloud
    if not s.vultr_key:
        logging.warning("VULTR_INFERENCE_KEY not set: rule-based brain, no vision or memory")
    st = Station(s, local=a.local, stub_locate=a.stub_locate, speaker=not a.no_audio,
                 qnx_port=a.qnx_port or None)
    try:
        asyncio.run(st.run(mic=a.mic))
    except KeyboardInterrupt:
        os._exit(0)
