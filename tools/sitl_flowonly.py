"""Flow-only SITL check: no GPS, simulated optical flow, baro height, no rangefinder.

Answers one question before real props go on: will ArduPilot (and our edge) take off, hold
and move in GUIDED with this sensor set at all? SITL flow is perfect, so a FAIL here means
the configuration is wrong; a PASS says nothing about the real camera's flow quality.

  docker build -t hope-sitl tools/sitl
  docker run --rm -d --name hope-sitl-flow -p 5761:5760 -v "$PWD/tools/sitl:/sitl/cfg:ro" \\
      --entrypoint ./arducopter hope-sitl --model + \\
      --defaults copter.parm,cfg/flowonly.parm,cfg/flow_rangefinder.parm \\
      --home 43.0096,-81.2737,250,0 --speedup 1 -I0
  python -m tools.sitl_flowonly            # default tcp:127.0.0.1:5761
(Drop cfg/flow_rangefinder.parm to repeat the no-rangefinder case. Give SITL ~30 s to warm up.)

Results 2026-11-21 (Copter stable SITL):
  * flowonly.parm alone (no rangefinder): the EKF never gets a horizontal position, on the
    ground or airborne in AltHold ("Need Position Estimate"; "Mode change to Guided failed:
    requires position"). A rangefinder is REQUIRED.
  * + rangefinder with RNGFND1_MIN 0.2 (out of range on the ground): still no position.
  * + rangefinder with RNGFND1_MIN 0 (valid on the ground): GUIDED takeoff works; hover drift
    0.03 m over 20 s at 0.72-0.94 m altitude.
  * The EKF origin must be re-sent until the FC confirms it ("EKF3 IMU0 origin set"); the
    edge sends it once at link-up, which the EKF ignored before it initialised.
  * move_body fwd=0.5 reported done at +0.24 m (pos_tolerance 0.3 is too big for short moves).
"""
from __future__ import annotations

import asyncio
import logging
import math
import sys
import time

from edge.config import SafetyConfig, WatchdogConfig
from edge.mav import MavVehicle
from edge.server import EdgeServer
from shared.client import CommandRejected, EdgeClient

results: list[tuple[str, bool, str]] = []


def check(name: str, cond: bool, info: str = "") -> None:
    results.append((name, bool(cond), info))
    print(f"[{'PASS' if cond else 'FAIL'}] {name} {info}", flush=True)


def pos(v) -> str:
    s = v.state()
    return f"n={s.n:+.2f} e={s.e:+.2f} alt={s.alt:.2f} yaw={s.yaw:.0f} mode={s.mode}"


async def main(device: str) -> int:
    cfg = SafetyConfig(flow_quality_min=0)  # SITL flow is internal to the FC, not from our UDP
    v = MavVehicle(device, speed_xy=cfg.speed_xy)
    srv = EdgeServer(v, cfg, WatchdogConfig(), host="127.0.0.1", port=0,
                     perception_port=None, flow_port=None)
    await srv.start()
    edge = await EdgeClient("127.0.0.1", srv.port,
                            on_event=lambda e: print("   event:", e["kind"], e["detail"])
                            ).connect()
    t0 = time.monotonic()
    while not v.state().connected and time.monotonic() - t0 < 30:
        await asyncio.sleep(0.5)
    check("FC heartbeat", v.state().connected)

    took_off, last_err = False, ""
    t0 = time.monotonic()
    while time.monotonic() - t0 < 120:  # the EKF needs time to converge on flow
        try:
            d = await edge.run("takeoff", alt=0.8, timeout=40)
            if d.get("status") == "ok":
                took_off = True
                break
            last_err = f"done {d.get('status')}: {d.get('reason')}"
        except CommandRejected as e:
            last_err = f"rejected: {e}"
        except Exception as e:
            last_err = f"{type(e).__name__}: {e}"
        print(f"   takeoff not yet ({time.monotonic() - t0:.0f} s): {last_err}", flush=True)
        await asyncio.sleep(5)
    check("takeoff 0.8 m (flow only)", took_off and abs(v.state().alt - 0.8) < 0.25,
          pos(v) if took_off else last_err)
    if not took_off:
        return summary()

    s0 = v.state()
    worst = 0.0
    alts = []
    for _ in range(40):  # 20 s hover
        await asyncio.sleep(0.5)
        s = v.state()
        worst = max(worst, math.hypot(s.n - s0.n, s.e - s0.e))
        alts.append(s.alt)
    check("hover 20 s, estimated drift < 0.5 m", worst < 0.5,
          f"max {worst:.2f} m; alt {min(alts):.2f}-{max(alts):.2f} m")

    n0 = v.state().n
    try:
        await edge.run("move_body", fwd=0.5, timeout=30)
        moved = True
    except Exception as e:
        moved, err = False, str(e)
    check("move_body fwd 0.5", moved and 0.3 < v.state().n - n0 < 0.8,
          f"dn={v.state().n - n0:+.2f} {pos(v)}" if moved else err)

    try:
        await edge.run("land", timeout=60)
    except Exception as e:
        print("   land:", e)
    t0 = time.monotonic()
    while v.state().armed and time.monotonic() - t0 < 30:
        await asyncio.sleep(0.5)
    check("landed and disarmed", not v.state().armed, pos(v))
    return summary()


def summary() -> int:
    ok = all(r[1] for r in results)
    print("ALL PASS" if ok else "SOME FAILED", flush=True)
    return 0 if ok else 1


if __name__ == "__main__":
    logging.basicConfig(level=logging.WARNING, format="%(asctime)s %(name)s %(message)s")
    sys.exit(asyncio.run(main(sys.argv[1] if len(sys.argv) > 1 else "tcp:127.0.0.1:5761")))
