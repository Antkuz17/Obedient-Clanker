"""Ground-truth `locate()` for `python -m edge --sim`: reads the sim room (edge.video.SIM_WORLD).

Same signature and result as station.localize.locate, so the planner runs unchanged. Only
"sees" an object that is inside the camera's field of view from the capture pose.
"""
from __future__ import annotations

import math

from edge.vehicle import ned_to_body
from edge.video import SIM_WORLD

from .camera import Pose


def _match(query: str, name: str) -> bool:
    q = query.lower().replace("the ", "").strip()
    return name in q or q in name


async def locate(ai, name: str, jpeg: bytes, pose: Pose, K: dict) -> dict | None:
    half = math.radians(K.get("hfov_deg", 45.0)) / 2
    for obj, n, e, *_ in SIM_WORLD:
        if not _match(name, obj):
            continue
        fwd, right = ned_to_body(n - pose.n, e - pose.e, pose.yaw)
        if fwd < 0.3 or abs(math.atan2(right, fwd)) > half * 0.9:
            return None
        bearing = math.degrees(math.atan2(e - pose.e, n - pose.n)) % 360
        return {"n": n, "e": e, "dist": math.hypot(fwd, right), "bearing": bearing, "conf": 1.0}
    return None
