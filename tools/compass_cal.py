"""Onboard compass calibration from the Pi (props OFF). Rotate the drone through every
orientation until it finishes (about 30-90 s): nose down, each side down, upside down, nose up,
spinning slowly around each axis. Do it away from metal, laptops and power bricks.

  python3 -m tools.compass_cal          # through the relay tap
Results are saved automatically on success; the FC must then be rebooted.
"""
from __future__ import annotations

import argparse
import time

from tools import fclink

M = fclink.M


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--addr", default=fclink.TAP)
    ap.add_argument("--timeout", type=float, default=180)
    ap.add_argument("--reboot", action="store_true", help="reboot the FC after a good calibration")
    a = ap.parse_args()
    m = fclink.connect(a.addr)
    # mag mask 0 = all, retry on failure, autosave, no delay, autoreboot off
    m.mav.command_long_send(1, 1, M.MAV_CMD_DO_START_MAG_CAL, 0, 0, 1, 1, 0, 0, 0, 0)
    print("compass calibration started: rotate the drone through every orientation...")
    t0, last, done = time.monotonic(), {}, {}
    while time.monotonic() - t0 < a.timeout:
        msg = m.recv_match(type=["MAG_CAL_PROGRESS", "MAG_CAL_REPORT", "STATUSTEXT", "COMMAND_ACK"],
                           blocking=True, timeout=1)
        if not msg or msg.get_srcSystem() != 1:
            continue
        t = msg.get_type()
        if t == "COMMAND_ACK" and msg.command == M.MAV_CMD_DO_START_MAG_CAL:
            print("FC accepted the calibration" if msg.result == 0 else f"FC refused: {msg.result}")
            if msg.result != 0:
                return 1
        elif t == "MAG_CAL_PROGRESS":
            pct = msg.completion_pct
            if last.get(msg.compass_id, -10) + 10 <= pct:
                last[msg.compass_id] = pct
                print(f"  compass {msg.compass_id}: {pct}%")
        elif t == "MAG_CAL_REPORT":
            ok = msg.cal_status == M.MAG_CAL_SUCCESS
            done[msg.compass_id] = ok
            print(f"compass {msg.compass_id}: {'SUCCESS' if ok else 'FAILED'} "
                  f"(fitness {msg.fitness:.1f}, offsets {msg.ofs_x:.0f},{msg.ofs_y:.0f},{msg.ofs_z:.0f})")
            if len(done) >= len(last) and len(done) > 0:
                break
        elif t == "STATUSTEXT" and "mag" in msg.text.lower():
            print("FC:", msg.text)
            # ArduPilot prints "Mag(N) good orientation" when compass N finished; the
            # MAG_CAL_REPORT itself may not reach the relay tap (seen on the board)
            if "good orientation" in msg.text.lower():
                cid = int(msg.text.split("(")[1].split(")")[0]) if "(" in msg.text else 0
                done[cid] = True
                last.setdefault(cid, 100)
                if len(done) >= len(last):
                    break
    if done and all(done.values()):
        print("calibration saved. Reboot the FC to use it.")
        if a.reboot:
            m.mav.command_long_send(1, 1, M.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN, 0, 1, 0, 0, 0, 0, 0, 0)
            print("FC reboot sent")
        return 0
    m.mav.command_long_send(1, 1, M.MAV_CMD_DO_CANCEL_MAG_CAL, 0, 0, 0, 0, 0, 0, 0, 0)
    print("calibration did not finish (cancelled). Rotate more, away from metal, and retry.")
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
