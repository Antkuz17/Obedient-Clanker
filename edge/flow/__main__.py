"""Run the optical-flow process: camgrab shared memory -> edge (UDP :7002).

  # on the Pi (camgrab already running; the down camera until the 2nd C270 arrives is camgrab_fwd)
  python3 -m edge.flow --shm camgrab_down --print
  # laptop: python -m tools.fake_camgrab --name camgrab_down --webcam 0  then the same command
"""
from __future__ import annotations

import argparse
import json
import logging
import socket
import time

from edge.camera import ShmFrameReader
from edge.flow import FlowTracker, focal_px, to_rates, C270_HFOV_DEG, PROC_W

log = logging.getLogger("flow")


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--shm", default="camgrab_down", help="camgrab shared-memory name")
    ap.add_argument("--edge", default="127.0.0.1:7002")
    ap.add_argument("--hfov", type=float, default=C270_HFOV_DEG, help="camera horizontal FOV, deg")
    ap.add_argument("--flip-x", action="store_true")
    ap.add_argument("--flip-y", action="store_true")
    ap.add_argument("--swap-xy", action="store_true")
    ap.add_argument("--print", action="store_true", help="print a line per second")
    ap.add_argument("--fc", help="also send OPTICAL_FLOW straight to the FC through the USB "
                    "relay's aux inlet, e.g. 127.0.0.1:14556, so position hold survives an edge "
                    "core crash (the core then only records quality)")
    a = ap.parse_args()
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(name)s %(message)s")

    host, port = a.edge.rsplit(":", 1)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    fc = None
    if a.fc:
        from edge.fcdirect import FLOW_COMPONENT, FcDirect
        fc = FcDirect(a.fc, FLOW_COMPONENT)
    f_px = focal_px(PROC_W, a.hfov)
    tracker = FlowTracker()
    reader = ShmFrameReader(a.shm)
    log.info("flow on '%s', f=%.1f px at %d px wide, sending to %s", a.shm, f_px, PROC_W, a.edge)

    last_seq, last_t = -1, None
    n, t_stat, sum_q, sum_ms, sum_fx, sum_fy = 0, time.monotonic(), 0, 0.0, 0.0, 0.0
    while True:
        frame = reader.wait_new(after_seq=last_seq, timeout=0.5)
        if frame is None:
            log.warning("no new frame from '%s' in 0.5 s", a.shm)
            continue
        t0 = time.monotonic()
        dt = 0.0 if last_t is None else frame.t - last_t
        last_seq, last_t = frame.seq, frame.t
        s = tracker.step(frame.gray())
        if s is None:
            continue
        fx, fy = to_rates(s, dt, f_px, a.flip_x, a.flip_y, a.swap_xy)
        if fc:
            fc.optical_flow(fx, fy, s.quality)
        sock.sendto(json.dumps({"fx": fx, "fy": fy, "q": s.quality, "dt_us": int(dt * 1e6),
                                "direct": fc is not None}).encode(), (host, int(port)))
        n += 1
        sum_q += s.quality
        sum_ms += (time.monotonic() - t0) * 1000
        sum_fx += fx
        sum_fy += fy
        if a.print and time.monotonic() - t_stat >= 1.0:
            print(f"{n:3d} Hz  q={sum_q / n:5.1f}  tracks={s.inliers:3d}  "
                  f"fx={sum_fx / n:+.3f} fy={sum_fy / n:+.3f} rad/s  "
                  f"proc={sum_ms / n:4.1f} ms", flush=True)
            n, t_stat, sum_q, sum_ms, sum_fx, sum_fy = 0, time.monotonic(), 0, 0.0, 0.0, 0.0


if __name__ == "__main__":
    main()
