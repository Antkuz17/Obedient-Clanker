"""Optical flow on synthetic frames: a random texture shifted by a known number of pixels."""
import numpy as np
import pytest

cv2 = pytest.importorskip("cv2")

from edge.flow import PROC_H, PROC_W, FlowTracker, focal_px, to_rates  # noqa: E402


def texture(seed=0, w=PROC_W + 80, h=PROC_H + 80):
    rng = np.random.default_rng(seed)
    img = rng.integers(0, 255, (h // 4, w // 4), dtype=np.uint8)
    return cv2.GaussianBlur(cv2.resize(img, (w, h), interpolation=cv2.INTER_NEAREST), (5, 5), 0)


def crop(img, ox, oy):
    return np.ascontiguousarray(img[40 + oy:40 + oy + PROC_H, 40 + ox:40 + ox + PROC_W])


@pytest.mark.parametrize("sx,sy", [(3, 0), (0, -4), (-2, 2), (0, 0)])
def test_known_shift_is_recovered(sx, sy):
    tex, t = texture(), FlowTracker()
    assert t.step(crop(tex, 0, 0)) is None
    # camera moves by (sx, sy) over the texture -> content moves by (-sx, -sy) in the image
    s = t.step(crop(tex, sx, sy))
    assert s.quality > 150, s
    assert s.dx_px == pytest.approx(-sx, abs=0.3)
    assert s.dy_px == pytest.approx(-sy, abs=0.3)


def test_blank_floor_gives_zero_quality():
    t = FlowTracker()
    flat = np.full((PROC_H, PROC_W), 128, np.uint8)
    t.step(flat)
    s = t.step(flat)
    assert s.quality == 0


def test_rates_scale_and_options():
    from edge.flow import FlowSample
    f = focal_px()
    s = FlowSample(dx_px=-f * 0.01, dy_px=0.0, inliers=100, quality=255)
    fx, fy = to_rates(s, dt=0.01, f_px=f)        # 0.01 rad in 0.01 s = 1 rad/s
    assert fx == pytest.approx(1.0) and fy == pytest.approx(0.0)
    assert to_rates(s, 0.01, f, flip_x=True)[0] == pytest.approx(-1.0)
    assert to_rates(s, 0.01, f, swap_xy=True) == pytest.approx((0.0, 1.0))
    assert to_rates(s, 0.0, f) == (0.0, 0.0)
