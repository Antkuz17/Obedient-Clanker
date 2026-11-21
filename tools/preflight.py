"""GO / NO-GO checklist before every flight. Read-only: changes nothing on the FC.

  python3 -m tools.preflight                  # on the Pi, through the relay tap
  python3 -m tools.preflight --addr tcp:127.0.0.1:5761   # SITL

Checks: FC link and mode, the indoor parameter profile, the FC's own pre-arm checks, EKF
position, optical flow (rate + quality as the FC sees it), the rangefinder reading on the
ground, battery, the pilot switch and throttle, and that the camera frames are fresh.
"""
from __future__ import annotations

import argparse
import time

from tools import fclink
from tools.fc_profile import PROFILE

M = fclink.M


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--addr", default=fclink.TAP)
    ap.add_argument("--shm", default="camgrab_down", help="flow camera's shared memory ('' = skip)")
    ap.add_argument("--min-batt", type=float, default=11.4, help="3S volts to allow takeoff")
    a = ap.parse_args()
    results: list[tuple[str, bool, str]] = []

    def check(name: str, ok: bool, info: str = "") -> None:
        results.append((name, ok, info))

    m = fclink.connect(a.addr)
    for mid, hz in ((M.MAVLINK_MSG_ID_EKF_STATUS_REPORT, 5), (M.MAVLINK_MSG_ID_OPTICAL_FLOW, 20),
                    (M.MAVLINK_MSG_ID_DISTANCE_SENSOR, 10), (M.MAVLINK_MSG_ID_SYS_STATUS, 2),
                    (M.MAVLINK_MSG_ID_RC_CHANNELS, 5)):
        fclink.stream(m, mid, hz)
    m.mav.command_long_send(1, 1, M.MAV_CMD_RUN_PREARM_CHECKS, 0, 0, 0, 0, 0, 0, 0, 0)

    seen: dict[str, list] = {}
    texts: list[str] = []
    hb = None
    t0 = time.monotonic()
    while time.monotonic() - t0 < 6.0:
        msg = m.recv_match(blocking=True, timeout=0.5)
        if not msg or msg.get_srcSystem() != 1:
            continue
        t = msg.get_type()
        if t == "HEARTBEAT" and msg.type != M.MAV_TYPE_GCS:
            hb = msg
        elif t == "STATUSTEXT":
            texts.append(msg.text)
        elif t in ("EKF_STATUS_REPORT", "OPTICAL_FLOW", "DISTANCE_SENSOR", "SYS_STATUS",
                   "RC_CHANNELS"):
            seen.setdefault(t, []).append(msg)

    armed = bool(hb and hb.base_mode & M.MAV_MODE_FLAG_SAFETY_ARMED)
    mode = fclink.mavutil.mode_string_v10(hb) if hb else "?"
    check("FC link + disarmed", hb is not None and not armed, f"mode {mode}, armed={armed}")

    bad = []
    for k, (want, _, _) in PROFILE.items():
        cur = fclink.get_param(m, k)
        if cur is None or abs(cur - want) > 1e-3 * max(1.0, abs(want)):
            bad.append(f"{k}={cur} (want {want})")
    check("parameter profile", not bad, "; ".join(bad[:4]) or "all match")

    prearm = sorted({x for x in texts if x.startswith(("PreArm", "Arm"))})
    check("FC pre-arm checks", not prearm, "; ".join(prearm[:4]) or "none reported")

    ekf = seen.get("EKF_STATUS_REPORT", [])
    flags = fclink.ekf_flags(ekf[-1].flags) if ekf else []
    check("EKF horizontal position", "pos_h_rel" in flags or "pred_h_rel" in flags,
          ",".join(flags) or "no EKF_STATUS_REPORT")

    of = seen.get("OPTICAL_FLOW", [])
    q = sum(x.quality for x in of) / len(of) if of else 0
    check("optical flow at the FC", len(of) >= 30 and q >= 100,
          f"{len(of) / 6:.0f} Hz, mean quality {q:.0f}")

    ds = seen.get("DISTANCE_SENSOR", [])
    d = ds[-1].current_distance / 100 if ds else None
    check("rangefinder valid on the ground", d is not None and 0.02 <= d < 0.5,
          f"{d} m ({len(ds) / 6:.0f} Hz)" if ds else "no DISTANCE_SENSOR")

    ss = seen.get("SYS_STATUS", [])
    v = ss[-1].voltage_battery / 1000 if ss else 0
    check("battery", v >= a.min_batt, f"{v:.2f} V (min {a.min_batt})")

    rc = seen.get("RC_CHANNELS", [])
    if rc:
        r = rc[-1]
        check("transmitter: throttle low", r.chan3_raw and r.chan3_raw < 1100, f"ch3={r.chan3_raw}")
        check("transmitter: SwA up (AltHold)", r.chan5_raw and r.chan5_raw < 1230,
              f"ch5={r.chan5_raw}")
    else:
        check("transmitter", False, "no RC_CHANNELS (transmitter off?)")

    if a.shm:
        try:
            from edge.camera import ShmFrameReader
            f = ShmFrameReader(a.shm, open_timeout=1.0).wait_new(timeout=1.0)
            age = (time.time() - f.t) * 1000 if f else None
            check(f"camera '{a.shm}' fresh", f is not None and age < 200, f"frame age {age:.0f} ms"
                  if f else "no new frame")
        except Exception as e:
            check(f"camera '{a.shm}' fresh", False, str(e)[:60])

    width = max(len(n) for n, _, _ in results)
    for n, ok, info in results:
        print(f"{'GO   ' if ok else 'NO-GO'} {n:<{width}}  {info}")
    go = all(ok for _, ok, _ in results)
    print("\n>>> GO <<<" if go else "\n>>> NO-GO: fix the lines above <<<")
    return 0 if go else 1


if __name__ == "__main__":
    raise SystemExit(main())
