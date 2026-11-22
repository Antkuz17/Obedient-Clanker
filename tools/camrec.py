"""Camera recorder on the Pi: what the cameras saw during each flight, as JPEG frames.

  python3 -m tools.camrec             # runs under the supervisor (lowest priority)

While the FC is armed (plus PRE_S before and POST_S after) it saves
  logs/blackbox/cam_<YYYYmmdd_HHMMSS>/down_<t>.jpg   down camera (optical flow), DOWN_HZ, gray
  logs/blackbox/cam_<YYYYmmdd_HHMMSS>/fwd_<t>.jpg    forward camera, FWD_HZ, colour
<t> is the frame's capture time (time.time(), 2 decimals), so frames line up with the
blackbox flight log. Reads the FC's armed state from the relay tap (read-only, like
tools/blackbox). Built after the first computer takeoff: flow quality fell to 0 in the air and
nothing showed what the down camera was seeing.
"""
from __future__ import annotations

import argparse
import collections
import time
from pathlib import Path

import cv2

from edge.camera import ShmFrameReader
from tools import fclink

M = fclink.M
DOWN_HZ, FWD_HZ, PRE_S, POST_S, QUALITY = 10.0, 5.0, 3.0, 5.0, 70


class Cam:
    def __init__(self, shm: str, prefix: str, hz: float, gray: bool):
        self.shm, self.prefix, self.period, self.gray = shm, prefix, 1.0 / hz, gray
        self.reader: ShmFrameReader | None = None
        self.next_t = 0.0
        self.pre: collections.deque = collections.deque()

    def grab(self) -> tuple[float, bytes] | None:
        now = time.monotonic()
        if now < self.next_t:
            return None
        self.next_t = now + self.period
        try:
            if self.reader is None:
                self.reader = ShmFrameReader(self.shm, open_timeout=0.2)
            f = self.reader.wait_new(timeout=0.05)  # newest frame
        except Exception:
            self.reader = None  # camgrab restarting: reopen next time
            return None
        if f is None:
            return None
        img = f.gray() if self.gray else f.bgr()
        ok, jpg = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, QUALITY])
        return (f.t, jpg.tobytes()) if ok else None


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--addr", default=fclink.TAP)
    ap.add_argument("--dir", default="logs/blackbox")
    a = ap.parse_args()
    out = Path(a.dir)
    out.mkdir(parents=True, exist_ok=True)
    while True:
        try:
            run(a.addr, out)
        except Exception as e:  # FC / relay restarting: reconnect
            print(f"camrec: {e!r}; reconnecting", flush=True)
            time.sleep(2)


def run(addr: str, out: Path) -> None:
    m = fclink.connect(addr, timeout=30)
    cams = [Cam("camgrab_down", "down", DOWN_HZ, True), Cam("camgrab_fwd", "fwd", FWD_HZ, False)]
    armed, folder, disarmed_at, last_keepalive = False, None, None, time.monotonic()
    print("camrec ready", flush=True)
    while True:
        msg = m.recv_match(type="HEARTBEAT", blocking=True, timeout=0.02)
        now = time.monotonic()
        if now - last_keepalive > 5:  # stay registered with the relay tap
            m.mav.command_long_send(1, 1, M.MAV_CMD_REQUEST_MESSAGE, 0, 0, 0, 0, 0, 0, 0, 0)
            last_keepalive = now
        if msg is not None and msg.get_srcSystem() == 1 and msg.type != M.MAV_TYPE_GCS:
            armed = bool(msg.base_mode & M.MAV_MODE_FLAG_SAFETY_ARMED)
            if armed and folder is None:
                folder = out / f"cam_{time.strftime('%Y%m%d_%H%M%S')}"
                folder.mkdir(parents=True, exist_ok=True)
                for c in cams:  # the PRE_S before arming
                    for t, jpg in c.pre:
                        (folder / f"{c.prefix}_{t:.2f}.jpg").write_bytes(jpg)
                    c.pre.clear()
                print(f"armed: saving frames to {folder}", flush=True)
            disarmed_at = None if armed else (disarmed_at or now)
        for c in cams:
            got = c.grab()
            if got is None:
                continue
            t, jpg = got
            if folder is not None:
                (folder / f"{c.prefix}_{t:.2f}.jpg").write_bytes(jpg)
            else:
                c.pre.append((t, jpg))
                while c.pre and time.time() - c.pre[0][0] > PRE_S:
                    c.pre.popleft()
        if folder is not None and not armed and disarmed_at and now - disarmed_at > POST_S:
            n = len(list(folder.glob("*.jpg")))
            print(f"disarmed: {n} frames in {folder}", flush=True)
            folder = None


if __name__ == "__main__":
    main()
