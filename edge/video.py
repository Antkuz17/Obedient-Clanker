"""Forward-camera HTTP server (protocol v1 video).

GET /forward.mjpg   multipart MJPEG stream
GET /forward.jpg    one JPEG, header X-Pose: n,e,alt,yaw,t  (pose at capture time)
GET /camera.json    intrinsics {width, height, fx, fy, cx, cy, hfov_deg}

SimCamera renders a fake room by projecting labelled boxes through a pinhole camera, so the
station's detector / target localization can be developed without the drone.
"""
from __future__ import annotations

import json
import math
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Callable

import numpy as np

try:
    import cv2
except ImportError:  # pragma: no cover
    cv2 = None

from .vehicle import VehicleState, ned_to_body

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


class UsbCamera:
    """cv2.VideoCapture camera. On QNX this may need the Sensor Framework instead (unit 5)."""

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


def serve_video(camera, host: str = "0.0.0.0", port: int = 8080, fps: float = 15.0):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *a):
            pass

        def do_GET(self):
            if self.path.startswith("/forward.jpg"):
                jpg, s, t = camera.capture()
                self.send_response(200)
                self.send_header("Content-Type", "image/jpeg")
                self.send_header("X-Pose", f"{s.n:.3f},{s.e:.3f},{s.alt:.3f},{s.yaw:.1f},{t:.3f}")
                self.send_header("Access-Control-Allow-Origin", "*")
                self.send_header("Access-Control-Expose-Headers", "X-Pose")
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
