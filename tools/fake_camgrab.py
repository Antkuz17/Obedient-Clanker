"""Stand-in for camgrab on a laptop: publish frames into the same shared memory layout.

  python -m tools.fake_camgrab --name camgrab_down --webcam 0     # laptop webcam (B: flow dev)
  python -m tools.fake_camgrab --name camgrab_fwd --sim-edge 127.0.0.1:8080   # mirror sim video
Then read with edge.camera.ShmFrameReader(name), exactly as on the Pi.
"""
from __future__ import annotations

import argparse
import time
import urllib.request

import cv2
import numpy as np

from edge.camera import FMT_NV12, FMT_YUYV, ShmFrameWriter


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--name", default="camgrab_fwd")
    src = ap.add_mutually_exclusive_group(required=True)
    src.add_argument("--webcam", type=int)
    src.add_argument("--sim-edge", help="host:port of an edge --sim video server")
    ap.add_argument("--width", type=int, default=640)
    ap.add_argument("--height", type=int, default=480)
    ap.add_argument("--fps", type=float, default=30)
    ap.add_argument("--yuyv", action="store_true", help="publish YUYV instead of NV12")
    a = ap.parse_args()
    w = ShmFrameWriter(a.name, a.width, a.height, FMT_YUYV if a.yuyv else FMT_NV12)
    cap = cv2.VideoCapture(a.webcam) if a.webcam is not None else None
    print(f"publishing {a.width}x{a.height} to shared memory '{a.name}' (Ctrl-C to stop)")
    try:
        while True:
            t0 = time.time()
            if cap is not None:
                ok, frame = cap.read()
                if not ok:
                    time.sleep(0.05)
                    continue
            else:
                jpg = urllib.request.urlopen(f"http://{a.sim_edge}/forward.jpg", timeout=2).read()
                frame = cv2.imdecode(np.frombuffer(jpg, np.uint8), cv2.IMREAD_COLOR)
            w.write_bgr(frame, t0)
            time.sleep(max(0.0, 1 / a.fps - (time.time() - t0)))
    except KeyboardInterrupt:
        pass
    finally:
        w.close()


if __name__ == "__main__":
    main()
