"""Computer-flown hover test: take off, hold, (optionally move), land, then report.

  python -m tools.auto_hover                      # 0.5 m, hold 15 s, land
  python -m tools.auto_hover --alt 0.6 --hold 20
  python -m tools.auto_hover --move right=0.5     # also a small move during the hold

Runs on the laptop against the edge on the Pi (protocol v1, the same path the voice planner
uses), so every command passes the Pi's safety gate. The safety pilot keeps a thumb on SwA:
flipping it takes control (down = land) and the edge never fights the pilot.
Ctrl+C at any time sends LAND.
"""
from __future__ import annotations

import argparse
import asyncio
import math
import os
import statistics
import time

from shared.client import CommandRejected, EdgeClient

GROUND_M = 0.12


def height_disagrees(t: dict, ground_m: float = 0.12, tol: float = 0.2) -> str | None:
    """The FC's height vs the rangefinder (the real distance to the floor). The rangefinder
    reads ground_m with the drone on the floor, so height above ground = range - ground_m.
    None when they agree or there's no echo. A big gap means the FC's height is wrong."""
    r, alt = t.get("range_m"), t.get("alt")
    if r is None or alt is None or r > 1.9:  # 2.01 = no echo
        return None
    real = r - ground_m
    if abs(alt - real) > tol:
        return (f"FC height {alt:.2f} m but rangefinder says {real:.2f} m above the floor: "
                "FC height is wrong, landing")
    return None


def pos(t: dict) -> str:
    return (f"n={t.get('n', 0):+.2f} e={t.get('e', 0):+.2f} alt={t.get('alt', 0):.2f} "
            f"range={t.get('range_m')} mode={t.get('mode')} state={t.get('state')}")


async def rng_flight(edge: EdgeClient, a) -> int:
    """Height decided by the rangefinder only (--rng). The FC's own height estimate drifts up
    on the ground while the motors spool, so it can't be trusted to say when the drone has
    lifted. Here: ask for a takeoff to the ceiling, watch the rangefinder, hold the moment it
    really reaches --alt, then keep correcting toward --alt from the rangefinder alone."""
    def real() -> float | None:
        r = (edge.telemetry or {}).get("range_m")
        return None if r is None or r < 0 or r > 1.9 else r - GROUND_M

    def fc_ok() -> bool:
        t = edge.telemetry or {}
        return bool(t.get("armed")) and t.get("mode") == "GUIDED"

    print(f"rangefinder-only flight: climb until the rangefinder says {a.alt} m above the floor")
    cid, ack = await edge.send("takeoff", alt=a.ceiling)
    if not ack.get("ok"):
        print("takeoff refused:", ack.get("reason"))
        return 1
    t0, lost_since = time.monotonic(), None
    while True:  # climb
        await asyncio.sleep(0.1)
        h = real()
        if h is None:
            lost_since = lost_since or time.monotonic()
            if time.monotonic() - lost_since > 1.0:
                print("ABORT: rangefinder lost the floor")
                return 1
            continue
        lost_since = None
        if h >= a.alt - 0.05:
            await edge.run("stop", timeout=10)  # hold here
            print(f"reached {h:.2f} m by rangefinder after {time.monotonic() - t0:.1f} s: holding")
            break
        if time.monotonic() - t0 > 3 and not fc_ok():
            print("ABORT: FC not armed in GUIDED (pilot, failsafe or arming failed):",
                  pos(edge.telemetry))
            return 1
        if time.monotonic() - t0 > 8:
            print(f"ABORT: still {h:.2f} m after 8 s, not lifting")
            return 1

    t1, hs = time.monotonic(), []
    while time.monotonic() - t1 < a.hold:  # hold, corrected by the rangefinder once a second
        await asyncio.sleep(1.0)
        h = real()
        if not fc_ok():
            print("pilot or failsafe took over:", pos(edge.telemetry))
            return 1
        if h is None:
            print("ABORT: rangefinder lost the floor")
            return 1
        hs.append(h)
        print(f"   height {h:.2f} m (rangefinder)")
        if h > a.max_alt:
            print(f"ABORT: {h:.2f} m is above {a.max_alt} m")
            return 1
        err = a.alt - h
        if abs(err) > 0.12:
            try:
                await edge.run("move_body", up=max(-0.3, min(0.3, err)), timeout=10)
            except CommandRejected as e:
                print("   correction refused:", e)
    print(f"hovered {a.hold:.0f} s at {min(hs):.2f}-{max(hs):.2f} m by rangefinder "
          f"(mean {statistics.mean(hs):.2f})")
    return 0


async def main(a) -> int:
    edge = await EdgeClient(a.host, on_event=lambda e: print(f"   !! {e['kind']}: {e['detail']}")
                            ).connect()
    await asyncio.sleep(1.5)
    t = edge.telemetry or {}
    print("before:", pos(t), f"battery={t.get('battery_v')} flow_q={t.get('flow_quality')}")
    global GROUND_M
    GROUND_M = a.ground if a.ground is not None else (t.get("range_m") or 0.12)
    if GROUND_M > 0.5:
        print(f"rangefinder reads {GROUND_M} m on the ground: not pointing at the floor? not flying")
        return 1
    if a.rng:
        try:
            return await rng_flight(edge, a)
        except (KeyboardInterrupt, asyncio.CancelledError):
            print("interrupted")
            return 1
        finally:
            print("landing ...")
            try:
                print("   ->", await edge.run("land", timeout=60))
            except Exception as e:
                print("   land:", e, "(flip SwA down if it is still flying)")
            await asyncio.sleep(1)
            print("after:", pos(edge.telemetry or {}))
    try:
        print(f"takeoff to {a.alt} m ...")
        d = await edge.run("takeoff", alt=a.alt, timeout=40)
        if d.get("status") != "ok":
            print("takeoff did not finish:", d)
            await edge.run("land", timeout=60)
            return 1
        print("airborne:", pos(edge.telemetry))
        bad = height_disagrees(edge.telemetry, GROUND_M)
        if bad:  # first try: EKF (baro) said 0.4 m while the rangefinder said 0.16 m: on the ground
            print("ABORT: " + bad)
            return 1

        t0, n0, e0 = time.monotonic(), edge.telemetry["n"], edge.telemetry["e"]
        alts, drift, disagree = [], [], 0
        while time.monotonic() - t0 < a.hold:
            await asyncio.sleep(0.5)
            t = edge.telemetry
            alts.append(t["alt"])
            drift.append(math.hypot(t["n"] - n0, t["e"] - e0))
            bad = height_disagrees(t, GROUND_M)
            if bad:
                disagree += 1
                if disagree >= 4:  # 2 s
                    print("ABORT: " + bad)
                    return 1
            else:
                disagree = 0
            if not t.get("armed") or t.get("mode") not in ("GUIDED",):
                print("pilot or failsafe took over:", pos(t))
                return 1
        print(f"hover {a.hold:.0f} s: height {min(alts):.2f}-{max(alts):.2f} m "
              f"(sd {statistics.pstdev(alts):.2f}), estimated drift max {max(drift):.2f} m")

        if a.move:
            k, _, v = a.move.partition("=")
            args = {"fwd": 0.0, "right": 0.0, "up": 0.0}
            args[{"right": "right", "left": "right", "back": "fwd", "fwd": "fwd",
                  "up": "up", "down": "up"}[k]] = float(v) * (-1 if k in ("left", "back", "down") else 1)
            print(f"move {args} ...")
            try:
                print("   ->", await edge.run("move_body", timeout=30, **args), pos(edge.telemetry))
            except CommandRejected as e:
                print("   gate refused:", e)
            await asyncio.sleep(3)
    except (KeyboardInterrupt, asyncio.CancelledError):
        print("interrupted")
    finally:
        print("landing ...")
        try:
            print("   ->", await edge.run("land", timeout=60))
        except Exception as e:
            print("   land:", e, "(flip SwA down if it is still flying)")
        await asyncio.sleep(1)
        print("after:", pos(edge.telemetry or {}))
    return 0


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default=os.environ.get("EDGE_HOST", "192.168.68.131"))
    ap.add_argument("--alt", type=float, default=0.5)
    ap.add_argument("--hold", type=float, default=15.0)
    ap.add_argument("--ground", type=float, default=None,
                    help="rangefinder reading on the floor (default: measured before takeoff)")
    ap.add_argument("--rng", action="store_true",
                    help="rangefinder-only height: climb until the rangefinder reads --alt, hold "
                         "and correct from it (the FC's estimate drifts up on the ground)")
    ap.add_argument("--ceiling", type=float, default=1.2,
                    help="--rng: takeoff target given to the FC so it keeps climbing (m)")
    ap.add_argument("--max-alt", type=float, default=1.2,
                    help="--rng: land if the rangefinder reads above this (m)")
    ap.add_argument("--move", help="e.g. right=0.5, left=0.5, back=0.5, up=0.2 (forward is "
                    "refused on the real drone until person detection runs)")
    raise SystemExit(asyncio.run(main(ap.parse_args())))
