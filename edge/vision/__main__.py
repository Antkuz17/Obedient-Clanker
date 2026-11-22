"""Run the on-board person detector: camgrab shared memory -> TFLite -> edge (UDP :7001).

  # on the Pi (camgrab_fwd running; model fetched with `python -m tools.fetch_person_model`
  # on the laptop and copied by tools/deploy_pi.py)
  python3 -m edge.vision --shm camgrab_fwd --print
  # laptop: python -m tools.fake_camgrab --name camgrab_fwd --webcam 0   then the same command

Sends one message per processed frame:
  {"person_m": float | null, "palm": false, "t": <frame capture time>, "infer_ms": float,
   "boxes": [[x0, y0, x1, y1, score, metres], ...]}   (boxes: dashboard overlay only)
`t` is the camera's capture time, so the dashboard's stop-latency tile shows frame -> stop.
Nothing is sent while frames aren't arriving: the edge then marks the detector dead after 1 s
and refuses forward moves, which is the point.
"""
from __future__ import annotations

import argparse
import json
import logging
import socket
import sys
import time

from edge.camera import ShmFrameReader
from edge.vision import (C270_HFOV_DEG, DEFAULT_MODEL, PERSON_CLASS, PersonDetector,
                         load_interpreter, nearest_person, person_distance)

log = logging.getLogger("vision")


def run(reader, detector: PersonDetector, send, hz: float, hfov: float,
        printer=None, stop=lambda: False) -> None:
    """The loop, separate from main() so tests can drive it with fakes."""
    period = 1.0 / hz if hz > 0 else 0.0
    last_seq, next_t = -1, 0.0
    n, t_stat, sum_ms, seen = 0, time.monotonic(), 0.0, 0
    # health line in the log every 10 s, so a working detector is visible (tools/system_check)
    hn, ht, hms = 0, time.monotonic(), 0.0
    while not stop():
        frame = reader.wait_new(after_seq=last_seq, timeout=0.5)
        if frame is None:
            log.warning("no new frame in 0.5 s; not reporting (edge will treat us as dead)")
            continue
        last_seq = frame.seq
        now = time.monotonic()
        if now < next_t:  # cap the rate: leave CPU for flow and the core
            continue
        next_t = now + period
        t0 = time.perf_counter()
        people = detector.detect(frame.bgr())
        infer_ms = (time.perf_counter() - t0) * 1000
        dist = nearest_person(people, frame.width, frame.height, hfov)
        # boxes for the dashboard overlay (normalised x0, y0, x1, y1, score, metres); additive:
        # the gate only reads person_m
        boxes = [[round(d.x0, 3), round(d.y0, 3), round(d.x1, 3), round(d.y1, 3), round(d.score, 2),
                  round(person_distance(d, frame.width, frame.height, hfov), 2)] for d in people[:5]]
        send({"person_m": None if dist is None else round(dist, 2), "palm": False,
              "t": frame.t, "infer_ms": round(infer_ms, 1), "boxes": boxes})
        n += 1
        sum_ms += infer_ms
        seen += dist is not None
        hn += 1
        hms += infer_ms
        if time.monotonic() - ht >= 10.0:
            log.info("running: %.1f Hz, infer %.0f ms", hn / (time.monotonic() - ht), hms / hn)
            hn, ht, hms = 0, time.monotonic(), 0.0
        if printer and time.monotonic() - t_stat >= 1.0:
            printer(f"{n:2d} Hz  infer={sum_ms / n:5.1f} ms  person in {seen}/{n} frames"
                    + (f"  nearest {dist:.2f} m" if dist is not None else ""))
            n, t_stat, sum_ms, seen = 0, time.monotonic(), 0.0, 0


def main() -> None:
    ap = argparse.ArgumentParser(prog="edge.vision")
    ap.add_argument("--shm", default="camgrab_fwd", help="camgrab shared-memory name")
    ap.add_argument("--model", default=str(DEFAULT_MODEL), help="SSD TFLite model")
    ap.add_argument("--edge", default="127.0.0.1:7001")
    ap.add_argument("--hz", type=float, default=10.0, help="max detections per second")
    ap.add_argument("--hfov", type=float, default=C270_HFOV_DEG, help="camera horizontal FOV, deg")
    ap.add_argument("--min-score", type=float, default=0.5)
    ap.add_argument("--person-class", type=int, default=PERSON_CLASS)
    ap.add_argument("--threads", type=int, default=2, help="TFLite threads (leave cores for "
                    "the core, flow and hbwd)")
    ap.add_argument("--print", action="store_true", help="print a line per second")
    a = ap.parse_args()
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(name)s %(message)s")

    try:
        detector = PersonDetector(load_interpreter(a.model, a.threads), a.min_score,
                                  a.person_class)
    except (RuntimeError, ValueError, OSError) as e:
        # exit non-zero: the supervisor retries (backing off), and the gate keeps refusing
        # forward moves meanwhile
        log.error("person detector can't start: %s", e)
        sys.exit(2)
    host, port = a.edge.rsplit(":", 1)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    send = lambda m: sock.sendto(json.dumps(m).encode(), (host, int(port)))
    reader = ShmFrameReader(a.shm)
    log.info("person detector on '%s' (%dx%d model input), sending to %s", a.shm,
             detector.in_w, detector.in_h, a.edge)
    run(reader, detector, send, a.hz, a.hfov,
        printer=(lambda s: print(s, flush=True)) if a.print else None)


if __name__ == "__main__":
    main()
