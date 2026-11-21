"""Hand-roll sign check for the software optical flow (props OFF, camera mounted pointing down).

  python3 -m tools.flow_signcheck            # then tilt the drone by hand for 20 s

ArduPilot expects flow to match the gyro during pure rotation: OF.flowX ~ bodyX (roll rate),
OF.flowY ~ bodyY (pitch rate) (docs/flight-setup.md). This records the FC's OPTICAL_FLOW
(what it received from edge/flow) and ATTITUDE (gyro), correlates them, and says which
edge/flow options to use: none, --flip-x, --flip-y, --swap-xy.

Tilt slowly: roll right/left a few times, then nose up/down a few times, over a textured floor.
"""
from __future__ import annotations

import argparse
import bisect
import time

from tools import fclink

M = fclink.M


def corr(a: list[float], b: list[float]) -> float:
    n = len(a)
    if n < 10:
        return 0.0
    ma, mb = sum(a) / n, sum(b) / n
    va = sum((x - ma) ** 2 for x in a)
    vb = sum((y - mb) ** 2 for y in b)
    if va <= 1e-12 or vb <= 1e-12:
        return 0.0
    return sum((x - ma) * (y - mb) for x, y in zip(a, b)) / (va * vb) ** 0.5


def advise(cxr: float, cyp: float, cxp: float, cyr: float, thresh: float = 0.5) -> list[str]:
    """Options for edge/flow from the four correlations (x/y flow vs roll/pitch rate)."""
    opts: list[str] = []
    if abs(cxp) + abs(cyr) > abs(cxr) + abs(cyp):  # axes crossed
        opts.append("--swap-xy")
        cxr, cyp = cyr, cxp  # after the swap, x pairs with what y used to
    if cxr < -thresh:
        opts.append("--flip-x")
    if cyp < -thresh:
        opts.append("--flip-y")
    return opts


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--addr", default=fclink.TAP)
    ap.add_argument("--seconds", type=float, default=20.0)
    ap.add_argument("--min-quality", type=int, default=100,
                    help="ignore flow samples below this quality (lost tracking dilutes the result)")
    a = ap.parse_args()
    m = fclink.connect(a.addr)
    fclink.stream(m, M.MAVLINK_MSG_ID_OPTICAL_FLOW, 20)
    fclink.stream(m, M.MAVLINK_MSG_ID_ATTITUDE, 50)
    print(f"recording {a.seconds:.0f} s: roll the drone right/left, then pitch nose up/down...")

    att_t, att_r, att_p, flow = [], [], [], []
    t0 = time.monotonic()
    while time.monotonic() - t0 < a.seconds:
        msg = m.recv_match(type=["ATTITUDE", "OPTICAL_FLOW"], blocking=True, timeout=0.5)
        if not msg or msg.get_srcSystem() != 1:
            continue
        now = time.monotonic()
        if msg.get_type() == "ATTITUDE":
            att_t.append(now)
            att_r.append(msg.rollspeed)
            att_p.append(msg.pitchspeed)
        else:
            flow.append((now, msg.flow_rate_x, msg.flow_rate_y, msg.quality))

    fx, fy, gr, gp, qs = [], [], [], [], []
    dropped = sum(1 for f in flow if f[3] < a.min_quality)
    for t, x, y, q in flow:
        if q < a.min_quality:
            continue
        i = bisect.bisect_left(att_t, t)
        if not att_t or i >= len(att_t):
            continue
        fx.append(x), fy.append(y), gr.append(att_r[i]), gp.append(att_p[i]), qs.append(q)
    if len(fx) < 50:
        print(f"only {len(fx)} flow samples: is edge/flow running and FLOW_TYPE=5 (rebooted)?")
        return 1
    motion = max(max(abs(r) for r in gr), max(abs(p) for p in gp))
    if motion < 0.3:
        print(f"barely any rotation (max {motion:.2f} rad/s): tilt more and rerun")
        return 1
    cxr, cyp, cxp, cyr = corr(fx, gr), corr(fy, gp), corr(fx, gp), corr(fy, gr)
    print(f"(ignored {dropped} samples below quality {a.min_quality})")
    print(f"{len(fx)} samples, mean quality {sum(qs) / len(qs):.0f}, max rotation {motion:.2f} rad/s")
    print(f"corr(flowX, roll rate)  = {cxr:+.2f}    corr(flowX, pitch rate) = {cxp:+.2f}")
    print(f"corr(flowY, pitch rate) = {cyp:+.2f}    corr(flowY, roll rate)  = {cyr:+.2f}")
    opts = advise(cxr, cyp, cxp, cyr)
    strong = max(abs(cxr), abs(cyp), abs(cxp), abs(cyr)) > 0.5
    if not strong:
        print("correlations are weak: more texture/light, tilt more, check the camera is fixed")
        return 1
    print("signs are right: no options needed" if not opts else
          f"run edge/flow with: {' '.join(opts)}   (then rerun this check)")
    return 0 if not opts else 2


if __name__ == "__main__":
    raise SystemExit(main())
