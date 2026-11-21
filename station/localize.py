"""Target localization: "the door" in a forward frame -> a local NED goal.

    hit = await locate(ai, "door", jpeg, pose, K)   # -> {n, e, dist, bearing, conf} | None

1. The vision model (Vultr, VISION_MODEL) returns a bounding box as JSON. Vultr hosts no
   open-vocabulary detector (models.md), so we ask the VLM for a box in 0-1000 coordinates.
2. Bearing from the box centre and the intrinsics.
3. Range from the box height and a size prior for the object (door 2 m, chair 0.9 m, ...);
   for unknown objects that stand on the floor, from where the box bottom meets the floor
   (camera height from the pose). Whichever is available, size prior first.
4. Body offset -> NED using the pose the frame was captured at (X-Pose), not the current one.

`n, e` is the object's estimated position; the planner stops short of it.
"""
from __future__ import annotations

import json
import logging
import math
import re

from edge.vehicle import body_to_ned

from .ai import AI
from .camera import Pose

log = logging.getLogger("localize")

# Typical heights (m). Used for range from box size. Lower-case substrings match.
SIZE_PRIOR = {
    "door": 2.0, "person": 1.7, "human": 1.7, "whiteboard": 1.0, "board": 1.0, "chair": 0.9,
    "table": 0.75, "desk": 0.75, "plant": 1.2, "poster": 0.6, "sign": 0.4, "monitor": 0.35,
    "screen": 0.35, "laptop": 0.25, "backpack": 0.45, "bag": 0.4, "bottle": 0.25, "cup": 0.1,
    "mug": 0.1, "box": 0.3, "keys": 0.08, "phone": 0.15, "trash": 0.6, "bin": 0.6,
}
DEFAULT_SIZE = 0.5
MIN_CONF = 0.35

PROMPT = """Find "{name}" in this image. Reply with JSON only, no prose:
{{"found": true/false, "box": [x0, y0, x1, y1], "conf": 0.0-1.0}}
Box corners are in 0-1000 coordinates (0,0 = top left, 1000,1000 = bottom right). If it is \
not clearly visible, reply {{"found": false}}."""


def size_prior(name: str) -> float | None:
    n = name.lower()
    for k, h in SIZE_PRIOR.items():
        if k in n:
            return h
    return None


def parse_box(text: str, width: int, height: int) -> tuple[list[float], float] | None:
    """VLM reply -> (pixel box, conf). Accepts 0-1000 or pixel coordinates, or fenced JSON."""
    m = re.search(r"\{.*\}", text, re.S)
    if not m:
        return None
    try:
        d = json.loads(m.group(0))
    except json.JSONDecodeError:
        return None
    if not d.get("found") or not isinstance(d.get("box"), list) or len(d["box"]) != 4:
        return None
    try:
        x0, y0, x1, y1 = (float(v) for v in d["box"])
    except (TypeError, ValueError):
        return None
    if max(x0, y0, x1, y1) <= 1.0:          # 0-1 normalized
        sx, sy = width, height
    elif max(x0, y0, x1, y1) <= 1000:       # 0-1000, as the prompt asks
        sx, sy = width / 1000, height / 1000
    else:                                   # the model answered in pixels anyway
        sx = sy = 1.0
    box = [min(x0, x1) * sx, min(y0, y1) * sy, max(x0, x1) * sx, max(y0, y1) * sy]
    if box[2] - box[0] < 2 or box[3] - box[1] < 2:
        return None
    return box, float(d.get("conf", 0.6) or 0.6)


def box_to_goal(name: str, box: list[float], pose: Pose, K: dict, conf: float = 1.0) -> dict | None:
    """Pixel box + capture pose + intrinsics -> {n, e, dist, bearing, conf, upper_bound}.

    A box cut off by the top or bottom of the frame shows less of the object than there is,
    so the size-based range is only an *upper bound*: the object is at most that far away
    (review3 #3: a door filling the frame read as 2.3-2.9 m at any closer distance, and the
    drone flew towards it). Then:
      - bottom visible: the floor-contact range is used if it is closer (both overestimate,
        never underestimate, so the smaller one is the safer one);
      - bottom cut off too: no usable range, the result is flagged `upper_bound` and the
        planner doesn't fly towards it ("it's right in front of me").
    """
    fx, fy, cx, cy = K["fx"], K["fy"], K["cx"], K["cy"]
    x0, y0, x1, y1 = box
    u = (x0 + x1) / 2
    h_real = size_prior(name)
    top_cut, bottom_cut = y0 <= 1, y1 >= K["height"] - 1
    size = fy * (h_real or DEFAULT_SIZE) / max(y1 - y0, 1.0)
    floor = (pose.alt * fy / (y1 - cy)
             if not bottom_cut and y1 > cy + 2 and pose.alt > 0.1 else None)  # bottom on floor
    upper = False
    if h_real and not top_cut and not bottom_cut:
        depth = size
    elif not top_cut and not bottom_cut:
        depth = floor if floor is not None else size   # unknown object: floor first
    elif floor is not None:
        depth = min(size, floor)                        # top cut: both are upper bounds
    else:
        depth, upper = size, True                       # bottom cut: closer than this, unknown
    if not (0.2 < depth < 15):
        return None
    right = depth * (u - cx) / fx
    dn, de = body_to_ned(depth, right, pose.yaw)
    bearing = (pose.yaw + math.degrees(math.atan2(u - cx, fx))) % 360
    return {"n": pose.n + dn, "e": pose.e + de, "dist": math.hypot(depth, right),
            "bearing": bearing, "conf": conf, "upper_bound": upper}


async def locate(ai: AI, name: str, jpeg: bytes, pose: Pose, K: dict) -> dict | None:
    try:
        # no reasoning: it ate all 120 tokens and the box never came (fast, too: ~40 tokens)
        text = await ai.vision(PROMPT.format(name=name), jpeg, max_tokens=200, think=False)
    except Exception as e:
        log.warning("locate(%s) failed: %s", name, e)
        return None
    parsed = parse_box(text, K["width"], K["height"])
    if not parsed:
        return None
    box, conf = parsed
    if conf < MIN_CONF:
        return None
    return box_to_goal(name, box, pose, K, conf)
