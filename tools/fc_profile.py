"""Flight parameter profile for the flight controller: check, back up, apply.

  python3 -m tools.fc_profile              # check only: prints OK / DIFF per parameter
  python3 -m tools.fc_profile --outdoor    # outdoors: GPS navigation instead of flow (no 2 m cap)
  python3 -m tools.fc_profile --apply      # back up current values, write the diffs, verify
  python3 -m tools.fc_profile --apply --reboot   # also reboot the FC if a reboot-only param changed
  python3 -m tools.fc_profile --addr tcp:127.0.0.1:5761   # SITL instead of the relay tap

Every value here was either verified on the real board or in SITL (tools/sitl_flowonly.py);
the "why" column is the source. Battery, compass and RC calibration stay board-specific and
are not touched.
"""
from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

from tools import fclink

# name: (value, needs_reboot, why)
PROFILE: dict[str, tuple[float, bool, str]] = {
    # failsafes (review A4, set on the board 2026-11-21)
    "FS_GCS_ENABLE": (5, False, "GCS (edge core) lost -> Land"),
    "FS_GCS_TIMEOUT": (2, False, "seconds without a GCS heartbeat"),
    "FS_EKF_ACTION": (1, False, "EKF failsafe -> Land"),
    # pilot switch: SwA on channel 5, up = AltHold, down = Land (verified on the board)
    "FLTMODE_CH": (5, False, "SwA"),
    "FLTMODE1": (2, False, "SwA up: AltHold"),
    "FLTMODE2": (2, False, "AltHold"),
    "FLTMODE3": (2, False, "AltHold"),
    "FLTMODE4": (2, False, "AltHold"),
    "FLTMODE5": (2, False, "AltHold"),
    "FLTMODE6": (9, False, "SwA down: Land"),
    # no GPS indoors; position from software optical flow (edge/flow) + rangefinder
    "GPS1_TYPE": (0, True, "no GPS indoors"),
    "FLOW_TYPE": (5, True, "MAVLink optical flow from edge/flow"),
    "FLOW_OPTIONS": (0, False, "FC subtracts its own gyro (no derotation on the Pi)"),
    "EK3_FLOW_DELAY": (60, False, "ms; frame age + exposure + processing, tune from logs"),
    "EK3_SRC1_POSXY": (0, False, "no absolute horizontal position"),
    "EK3_SRC1_VELXY": (5, False, "optical flow"),
    "EK3_SRC1_POSZ": (1, False, "baro (SITL flow-only pass used baro)"),
    # first autonomous try: the baro read prop wash near the floor as a 1 m climb while the
    # rangefinder said 0.16 m, so the FC cut throttle on the ground. Use the rangefinder for
    # height whenever it's below 70% of its max range (1.4 m), baro above that.
    "EK3_RNG_USE_HGT": (70, False, "rangefinder height below 1.4 m (baro lies in prop wash)"),
    "EK3_SRC1_VELZ": (0, False, "none"),
    "EK3_SRC1_YAW": (1, False, "compass"),
    # HC-SR04 via Arduino -> tools/rangefinder.py (verified end to end on the board)
    "RNGFND1_TYPE": (10, True, "MAVLink rangefinder"),
    "RNGFND1_ORIENT": (25, False, "pointing down"),
    "RNGFND1_MIN": (0.02, False, "m; must read valid on the ground (SITL)"),
    "RNGFND1_MAX": (2.0, False, "m; HC-SR04 is reliable to about 2 m"),
}

# Outdoors (2026-11-21): navigate on the HGLRC M100-PRO GPS. These are ArduPilot's own
# defaults for a GPS copter. Indoors, flow + a 2 m ultrasonic capped the height at ~2 m (above
# it the EKF lost position and the edge landed). Run the edge with --gps to match.
OUTDOOR: dict[str, tuple[float, bool, str]] = {
    "GPS1_TYPE": (1, True, "GPS on (auto-detect)"),
    "EK3_SRC1_POSXY": (3, False, "GPS horizontal position"),
    "EK3_SRC1_VELXY": (3, False, "GPS horizontal velocity"),
    "EK3_SRC1_POSZ": (1, False, "baro height (works at any height; the HC-SR04 only to 2 m)"),
    "EK3_SRC1_VELZ": (3, False, "GPS vertical velocity"),
    "EK3_SRC1_YAW": (1, False, "compass"),
}


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--addr", default=fclink.TAP)
    ap.add_argument("--apply", action="store_true")
    ap.add_argument("--reboot", action="store_true", help="reboot the FC if needed after apply")
    ap.add_argument("--outdoor", action="store_true", help="GPS navigation (see OUTDOOR)")
    a = ap.parse_args()
    profile = {**PROFILE, **OUTDOOR} if a.outdoor else PROFILE
    m = fclink.connect(a.addr)

    current = {k: fclink.get_param(m, k) for k in profile}
    diffs = []
    for k, (want, rb, why) in profile.items():
        cur = current[k]
        ok = cur is not None and abs(cur - want) < 1e-3 * max(1.0, abs(want))
        if not ok:
            diffs.append(k)
        print(f"{'OK  ' if ok else 'DIFF'} {k:<16} {cur!s:>8} -> {want:<6} {'(reboot) ' if rb else ''}{why}")
    if not diffs:
        print("profile matches")
        return 0
    if not a.apply:
        print(f"{len(diffs)} differ; run with --apply to write them")
        return 1

    backup = Path(f"fc_params_backup_{time.strftime('%Y%m%d_%H%M%S')}.json")
    backup.write_text(json.dumps(current, indent=2))
    print(f"backed up current values to {backup}")
    need_reboot = False
    for k in diffs:
        want, rb, _ = profile[k]
        got = fclink.set_param(m, k, want)
        ok = got is not None and abs(got - want) < 1e-3 * max(1.0, abs(want))
        need_reboot |= rb and ok
        print(f"{'set ' if ok else 'FAIL'} {k} = {got}")
    if need_reboot:
        if a.reboot:
            m.mav.command_long_send(1, 1, fclink.M.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN, 0,
                                    1, 0, 0, 0, 0, 0, 0)
            print("FC reboot sent (the relay reconnects by itself)")
        else:
            print("a reboot-only parameter changed: power-cycle the FC or rerun with --reboot")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
