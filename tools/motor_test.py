"""Props-OFF motor test: spin each motor alone, in ArduPilot's test order, to check that all
work, spin the right way and are wired to the right outputs.

  python3 -m tools.motor_test                 # 10% for 2 s each, all 4 in sequence

Order (Quad X): A front-right (CCW), B back-right (CW), C back-left (CCW), D front-left (CW),
seen from above. Wrong order -> ESC signal on the wrong FC output; wrong direction -> swap any
two of that motor's three wires. Verified on the real drone 2026-11-21.
"""
from __future__ import annotations

import argparse
import time

from tools import fclink

M = fclink.M


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--addr", default=fclink.TAP)
    ap.add_argument("--percent", type=float, default=10)
    ap.add_argument("--seconds", type=float, default=2)
    a = ap.parse_args()
    if a.percent > 20:
        raise SystemExit("refusing more than 20%: this is a props-off test")
    m = fclink.connect(a.addr)
    m.mav.command_long_send(1, 1, M.MAV_CMD_DO_MOTOR_TEST, 0, 1, M.MOTOR_TEST_THROTTLE_PERCENT,
                            a.percent, a.seconds, 4, M.MOTOR_TEST_ORDER_DEFAULT, 0)
    print("PROPS OFF? motors A-D (front-right, back-right, back-left, front-left) in turn...")
    t0 = time.monotonic()
    while time.monotonic() - t0 < 4 * a.seconds + 4:
        x = m.recv_match(type=["COMMAND_ACK", "STATUSTEXT"], blocking=True, timeout=1)
        if not x or x.get_srcSystem() != 1:
            continue
        if x.get_type() == "COMMAND_ACK" and x.command == M.MAV_CMD_DO_MOTOR_TEST:
            if x.result != 0:
                print(f"FC refused the motor test (result {x.result})")
                return 1
        elif x.get_type() == "STATUSTEXT":
            print("FC:", x.text)
            if "finished motor test" in x.text:
                return 0
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
