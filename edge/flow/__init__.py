"""Software optical flow: a downward webcam (camgrab shared memory) -> OPTICAL_FLOW via the edge.

The drone has no optical-flow sensor, so this process plays one. Per frame pair:
  1. track Shi-Tomasi corners with pyramidal Lucas-Kanade, keep forward-backward-consistent ones
  2. median pixel shift (robust to a few bad tracks)
  3. pixels -> radians with the camera's focal length, / dt -> rad/s
  4. quality 0-255 from the inlier count and how well the tracks agree
  5. send {"fx","fy","q","dt_us"} to the edge (UDP 127.0.0.1:7002), which forwards OPTICAL_FLOW

ArduPilot (FLOW_TYPE=5) reads flow_rate_x/y as raw angular flow and subtracts its own gyro
(FLOW_OPTIONS=0), so no derotation here. The axis mapping is a guess until the hand-roll check
(docs/flight-setup.md): roll the drone right by hand and OF.flowX must follow OF.bodyX, pitch for
flowY. Fix signs with --flip-x/--flip-y/--swap-xy (or FLOW_ORIENT_YAW on the FC).

Only numpy + OpenCV, so it runs on QNX's Python unchanged. Develop on a laptop with
tools/fake_camgrab.py.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

try:
    import cv2
except ImportError:  # pragma: no cover - cv2 is on the Pi (apk python3-opencv) and laptops
    cv2 = None

PROC_W, PROC_H = 320, 240        # process at this size (fast LK; the C270 is 640x480)
# ~55 deg diagonal spec -> ~45 deg horizontal at 4:3 (60 overstated flow speed by ~25-40 %).
# Measure it (edge/video.py HFOV_DEG has how) and pass --hfov; fine-tune with FLOW_FXSCALER.
C270_HFOV_DEG = 45.0
MIN_TRACKS = 12                  # fewer good tracks than this -> quality 0


@dataclass
class FlowSample:
    dx_px: float      # median image shift between the two frames, at PROC size (+x right)
    dy_px: float      # (+y down)
    inliers: int
    quality: int      # 0-255, what ArduPilot's EKF uses to accept/reject the sample


def focal_px(width: int = PROC_W, hfov_deg: float = C270_HFOV_DEG) -> float:
    return (width / 2) / math.tan(math.radians(hfov_deg) / 2)


class FlowTracker:
    """Frame-to-frame flow on grayscale images (any size; resized to PROC_W x PROC_H)."""

    def __init__(self, max_corners: int = 150, fb_max_px: float = 1.0):
        if cv2 is None:
            raise RuntimeError("OpenCV (cv2) is required for optical flow")
        self.max_corners, self.fb_max_px = max_corners, fb_max_px
        self.prev: np.ndarray | None = None
        self.lk = dict(winSize=(21, 21), maxLevel=3,
                       criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 20, 0.03))

    @staticmethod
    def prep(gray: np.ndarray) -> np.ndarray:
        if gray.shape[1] != PROC_W or gray.shape[0] != PROC_H:
            gray = cv2.resize(gray, (PROC_W, PROC_H), interpolation=cv2.INTER_AREA)
        return gray

    def step(self, gray: np.ndarray) -> FlowSample | None:
        """Feed the next frame; returns the shift from the previous one (None on the first)."""
        cur = self.prep(gray)
        prev, self.prev = self.prev, cur
        if prev is None:
            return None
        p0 = cv2.goodFeaturesToTrack(prev, self.max_corners, 0.01, 8, blockSize=7)
        if p0 is None or len(p0) < MIN_TRACKS:
            return FlowSample(0.0, 0.0, 0 if p0 is None else len(p0), 0)
        p1, st1, _ = cv2.calcOpticalFlowPyrLK(prev, cur, p0, None, **self.lk)
        p0r, st2, _ = cv2.calcOpticalFlowPyrLK(cur, prev, p1, None, **self.lk)
        fb = np.linalg.norm((p0 - p0r).reshape(-1, 2), axis=1)
        good = (st1.ravel() == 1) & (st2.ravel() == 1) & (fb < self.fb_max_px)
        n = int(good.sum())
        if n < MIN_TRACKS:
            return FlowSample(0.0, 0.0, n, 0)
        d = (p1 - p0).reshape(-1, 2)[good]
        dx, dy = float(np.median(d[:, 0])), float(np.median(d[:, 1]))
        # agreement: median absolute deviation of the tracks around the median shift
        mad = float(np.median(np.linalg.norm(d - (dx, dy), axis=1)))
        q = 255.0 * min(1.0, n / 60.0) * max(0.0, 1.0 - mad / 2.0)
        return FlowSample(dx, dy, n, int(max(0, min(255, round(q)))))


def to_rates(s: FlowSample, dt: float, f_px: float, flip_x: bool = False,
             flip_y: bool = False, swap_xy: bool = False) -> tuple[float, float]:
    """Image shift -> ArduPilot flow_rate_x/y (rad/s).

    Default mapping (camera looking down, image top = drone nose): rolling right swings the view
    right, so the floor slides left in the image (dx < 0) -> flow_x = -dx; pitching nose-up
    swings the view back, the floor slides down-to-up... (dy < 0) -> flow_y = -dy. UNVERIFIED:
    settle it with the hand-roll check and the flip/swap options.
    """
    if dt <= 0:
        return 0.0, 0.0
    fx, fy = -s.dx_px / f_px / dt, -s.dy_px / f_px / dt
    if swap_xy:
        fx, fy = fy, fx
    return (-fx if flip_x else fx), (-fy if flip_y else fy)
