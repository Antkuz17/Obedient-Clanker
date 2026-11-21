"""Forward-camera HTTP server (protocol v1 video).

GET /forward.mjpg   multipart MJPEG stream
GET /forward.jpg    one JPEG, header X-Pose: n,e,alt,yaw,t  (pose at capture time)
GET /camera.json    intrinsics {width, height, fx, fy, cx, cy, hfov_deg}

SimCamera renders a fake room by projecting labelled boxes through a pinhole camera, so the
station's detector / target localization can be developed without the drone.

Runs inside the edge (default) or as its own process (QNX demo: video can die or lag without
touching the flight loop). Separate process: the core streams its pose over localhost UDP.

  python -m edge --mav ... --video-proc 127.0.0.1:7003          # core: send pose, no video
  python -m edge.video --pose-port 7003 --camera shm:camgrab_fwd   # video process
Frames then also carry X-Pose-Age (s since the last pose from the core; large = core gone).
"""
from __future__ import annotations

import argparse
import json
import logging
import math
import socket
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Callable

import numpy as np

try:
    import cv2
except ImportError:  # pragma: no cover
    cv2 = None

from dataclasses import fields

from .vehicle import VehicleState, ned_to_body

_STATE_FIELDS = {f.name for f in fields(VehicleState)}


class PoseReceiver:
    """VehicleState from the core's UDP pose stream (JSON of VehicleState.as_dict())."""

    def __init__(self, port: int, host: str = "127.0.0.1"):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((host, port))
        self.sock.settimeout(0.5)
        self.port = self.sock.getsockname()[1]
        self._s = VehicleState()
        self._t = 0.0
        self.received = 0
        self._stop = threading.Event()
        threading.Thread(target=self._run, daemon=True, name="pose-rx").start()

    def _run(self) -> None:
        while not self._stop.is_set():
            try:
                data = self.sock.recv(4096)
                d = json.loads(data)
                s = VehicleState(**{k: v for k, v in d.items() if k in _STATE_FIELDS})
            except socket.timeout:
                continue
            except (OSError, ValueError, TypeError):
                if self._stop.is_set():
                    return
                continue
            self._s, self._t = s, time.monotonic()
            self.received += 1

    def state(self) -> VehicleState:
        return self._s

    def age(self) -> float:
        return time.monotonic() - self._t if self._t else float("inf")

    def close(self) -> None:
        self._stop.set()
        self.sock.close()

W, H = 640, 480
HFOV_DEG = 60.0  # measure the real C270 and update


def intrinsics(width: int = W, height: int = H, hfov_deg: float = HFOV_DEG) -> dict:
    fx = (width / 2) / math.tan(math.radians(hfov_deg) / 2)
    return {"width": width, "height": height, "fx": fx, "fy": fx, "cx": width / 2,
            "cy": height / 2, "hfov_deg": hfov_deg}


# world objects for the sim: name, n, e, bottom z, height, width, BGR colour
SIM_WORLD = [
    ("door", 3.0, 0.0, 0.0, 2.0, 0.9, (40, 90, 160)),
    ("whiteboard", 0.0, 3.0, 0.8, 1.0, 1.5, (235, 235, 235)),
    ("keys", -2.0, -1.5, 0.7, 0.08, 0.12, (0, 200, 255)),
    ("chair", -1.0, 2.0, 0.0, 0.9, 0.5, (60, 60, 200)),
    ("plant", 2.0, -2.5, 0.0, 1.2, 0.5, (40, 160, 40)),
]


class SimCamera:
    def __init__(self, get_state: Callable[[], VehicleState]):
        self.get_state = get_state
        self.K = intrinsics()

    def capture(self) -> tuple[bytes, VehicleState, float]:
        s = self.get_state()
        t = time.time()
        img = np.full((H, W, 3), (70, 70, 70), np.uint8)
        img[H // 2:, :] = (110, 100, 90)  # floor
        fx, cx, cy = self.K["fx"], self.K["cx"], self.K["cy"]
        cam_z = max(s.alt, 0.05)
        drawn = []
        for name, n, e, z0, h, w, col in SIM_WORLD:
            fwd, right = ned_to_body(n - s.n, e - s.e, s.yaw)
            if fwd < 0.3:
                continue
            u = cx + fx * right / fwd
            half = fx * (w / 2) / fwd
            v_top = cy - fx * (z0 + h - cam_z) / fwd
            v_bot = cy - fx * (z0 - cam_z) / fwd
            drawn.append((fwd, name, int(u - half), int(v_top), int(u + half), int(v_bot), col))
        for fwd, name, x0, y0, x1, y1, col in sorted(drawn, reverse=True):
            if cv2 is not None:
                cv2.rectangle(img, (x0, y0), (x1, y1), col, -1)
                cv2.putText(img, name, (x0, max(12, y0 - 4)), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                            (255, 255, 255), 1)
        if cv2 is None:
            return b"", s, t
        cv2.putText(img, f"SIM n={s.n:.2f} e={s.e:.2f} alt={s.alt:.2f} yaw={s.yaw:.0f}",
                    (8, H - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        ok, jpg = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 80])
        return jpg.tobytes(), s, t


class ShmCamera:
    """Frames from camgrab's shared memory (the only way to get camera frames on QNX)."""

    def __init__(self, get_state: Callable[[], VehicleState], name: str = "camgrab_fwd"):
        from .camera import ShmFrameReader
        self.get_state = get_state
        self.r = ShmFrameReader(name)
        self.K = intrinsics(self.r.width, self.r.height)

    def capture(self) -> tuple[bytes, VehicleState, float]:
        f = self.r.read()
        s = self.get_state()
        if f is None:
            return b"", s, time.time()
        ok, jpg = cv2.imencode(".jpg", f.bgr(), [cv2.IMWRITE_JPEG_QUALITY, 80])
        return jpg.tobytes(), s, f.t


class UsbCamera:
    """cv2.VideoCapture camera, for laptops only. On QNX use ShmCamera + camgrab."""

    def __init__(self, get_state: Callable[[], VehicleState], index: int = 0):
        self.get_state = get_state
        self.cap = cv2.VideoCapture(index)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, W)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, H)
        self.K = intrinsics()
        self._lock = threading.Lock()

    def capture(self) -> tuple[bytes, VehicleState, float]:
        with self._lock:
            ok, frame = self.cap.read()
        s, t = self.get_state(), time.time()
        if not ok:
            return b"", s, t
        ok, jpg = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
        return jpg.tobytes(), s, t


def serve_video(camera, host: str = "0.0.0.0", port: int = 8080, fps: float = 15.0,
                pose_age: Callable[[], float] | None = None):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *a):
            pass

        def do_GET(self):
            if self.path.startswith("/forward.jpg"):
                jpg, s, t = camera.capture()
                self.send_response(200)
                self.send_header("Content-Type", "image/jpeg")
                self.send_header("X-Pose", f"{s.n:.3f},{s.e:.3f},{s.alt:.3f},{s.yaw:.1f},{t:.3f}")
                if pose_age:
                    self.send_header("X-Pose-Age", f"{min(pose_age(), 1e6):.3f}")
                self.send_header("Access-Control-Allow-Origin", "*")
                self.send_header("Access-Control-Expose-Headers", "X-Pose, X-Pose-Age")
                self.end_headers()
                self.wfile.write(jpg)
            elif self.path.startswith("/forward.mjpg"):
                self.send_response(200)
                self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
                self.send_header("Access-Control-Allow-Origin", "*")
                self.end_headers()
                try:
                    while True:
                        jpg, s, t = camera.capture()
                        self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n"
                                         + f"X-Pose: {s.n:.3f},{s.e:.3f},{s.alt:.3f},"
                                           f"{s.yaw:.1f},{t:.3f}\r\n".encode()
                                         + f"Content-Length: {len(jpg)}\r\n\r\n".encode()
                                         + jpg + b"\r\n")
                        time.sleep(1.0 / fps)
                except (BrokenPipeError, ConnectionResetError):
                    pass
            elif self.path.startswith("/camera.json"):
                body = json.dumps(camera.K).encode()
                self.send_response(200)
                self.send_header("Content-Type", "application/json")
                self.send_header("Access-Control-Allow-Origin", "*")
                self.end_headers()
                self.wfile.write(body)
            else:
                self.send_error(404)

    srv = ThreadingHTTPServer((host, port), Handler)
    srv.daemon_threads = True
    threading.Thread(target=srv.serve_forever, daemon=True, name="video").start()
    return srv


def make_camera(spec: str, get_state: Callable[[], VehicleState]):
    """sim | shm:<name> | usb:<index> (or a bare index)."""
    if spec == "sim":
        return SimCamera(get_state)
    if spec.startswith("shm:"):
        return ShmCamera(get_state, spec[4:])
    return UsbCamera(get_state, int(spec.removeprefix("usb:")))


def main(argv: list[str] | None = None) -> None:
    ap = argparse.ArgumentParser(description="forward-camera video server, own process")
    ap.add_argument("--pose-port", type=int, default=7003, help="UDP pose stream from the core")
    ap.add_argument("--camera", default="shm:camgrab_fwd", help="sim | shm:<name> | usb:<index>")
    ap.add_argument("--host", default="0.0.0.0")
    ap.add_argument("--port", type=int, default=8080)
    ap.add_argument("--fps", type=float, default=15.0)
    a = ap.parse_args(argv)
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(name)s %(message)s")
    pose = PoseReceiver(a.pose_port)
    cam = make_camera(a.camera, pose.state)
    serve_video(cam, a.host, a.port, a.fps, pose_age=pose.age)
    logging.info("video on http://%s:%d/forward.mjpg (pose from udp :%d)", a.host, a.port,
                 a.pose_port)
    try:
        threading.Event().wait()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
