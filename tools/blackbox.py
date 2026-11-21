"""Flight recorder on the Pi: every FC message worth keeping, to JSON lines, one file per flight.

  python3 -m tools.blackbox                      # runs under the supervisor; logs/blackbox/
  python3 -m tools.flight_report                 # summary of the last flight (laptop or Pi)

Listens on the USB relay's tap (read-only: sysid 254, never sends heartbeats or commands
except stream-rate requests and a keep-alive). Writes logs/blackbox/idle.jsonl while disarmed
(rotated at 20 MB) and starts flight_<YYYYmmdd_HHMMSS>.jsonl at each arm, with the previous
10 s of data copied in, closed 10 s after disarm. The FC's own SD-card log is better when it
works (LOG_BACKEND_TYPE=1); this one always works and needs nothing on the FC.
"""
from __future__ import annotations

import argparse
import collections
import json
import os
import time
from pathlib import Path

from tools import fclink

M = fclink.M
KEEP = {
    "HEARTBEAT", "STATUSTEXT", "ATTITUDE", "LOCAL_POSITION_NED", "OPTICAL_FLOW",
    "DISTANCE_SENSOR", "BATTERY_STATUS", "SYS_STATUS", "VIBRATION", "EKF_STATUS_REPORT",
    "RC_CHANNELS", "SERVO_OUTPUT_RAW", "VFR_HUD", "POSITION_TARGET_LOCAL_NED",
}
# only rates the edge doesn't already set higher (stream rates are shared on the FC link)
RATES = {M.MAVLINK_MSG_ID_OPTICAL_FLOW: 10, M.MAVLINK_MSG_ID_VIBRATION: 4,
         M.MAVLINK_MSG_ID_EKF_STATUS_REPORT: 4, M.MAVLINK_MSG_ID_RC_CHANNELS: 5,
         M.MAVLINK_MSG_ID_BATTERY_STATUS: 2, M.MAVLINK_MSG_ID_SERVO_OUTPUT_RAW: 5,
         M.MAVLINK_MSG_ID_VFR_HUD: 5, M.MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED: 5}
PRE_S, POST_S, IDLE_MAX = 10.0, 10.0, 20 * 1024 * 1024


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--addr", default=fclink.TAP)
    ap.add_argument("--dir", default="logs/blackbox")
    a = ap.parse_args()
    out = Path(a.dir)
    out.mkdir(parents=True, exist_ok=True)
    while True:
        try:
            run(a.addr, out)
        except Exception as e:  # FC rebooting / relay restarting: reconnect
            print(f"blackbox: {e!r}; reconnecting", flush=True)
            time.sleep(2)


def run(addr: str, out: Path) -> None:
    m = fclink.connect(addr, timeout=30)
    for mid, hz in RATES.items():
        fclink.stream(m, mid, hz)
    pre: collections.deque = collections.deque()
    idle = open(out / "idle.jsonl", "a")
    flight, armed_since, disarmed_at = None, None, None
    last_keepalive, last_rates, last_rx = time.monotonic(), time.monotonic(), time.monotonic()
    print("blackbox recording", flush=True)
    while True:
        msg = m.recv_match(blocking=True, timeout=1.0)
        now = time.monotonic()
        if now - last_keepalive > 5:  # stay registered with the relay tap
            m.mav.command_long_send(1, 1, M.MAV_CMD_REQUEST_MESSAGE, 0, 0, 0, 0, 0, 0, 0, 0)
            last_keepalive = now
        if now - last_rates > 30:  # FC reboots forget stream rates
            for mid, hz in RATES.items():
                fclink.stream(m, mid, hz)
            last_rates = now
        if msg is None:
            if now - last_rx > 10:
                raise TimeoutError("no FC data for 10 s")
            continue
        if msg.get_srcSystem() != 1 or msg.get_type() not in KEEP:
            continue
        last_rx = now
        rec = msg.to_dict()
        rec["_t"] = round(time.time(), 3)
        line = json.dumps(rec, default=str) + "\n"

        if msg.get_type() == "HEARTBEAT" and msg.type != M.MAV_TYPE_GCS:
            armed = bool(msg.base_mode & M.MAV_MODE_FLAG_SAFETY_ARMED)
            if armed and flight is None:
                name = out / f"flight_{time.strftime('%Y%m%d_%H%M%S')}.jsonl"
                flight = open(name, "w")
                flight.writelines(r for _, r in pre)
                armed_since, disarmed_at = now, None
                print(f"armed: recording {name}", flush=True)
            elif armed:
                disarmed_at = None
            elif flight is not None and disarmed_at is None:
                disarmed_at = now

        if flight is not None:
            flight.write(line)
            if disarmed_at is not None and now - disarmed_at > POST_S:
                flight.close()
                print(f"disarmed: flight closed after {now - armed_since:.0f} s", flush=True)
                flight = None
        else:
            idle.write(line)
            pre.append((now, line))
            while pre and now - pre[0][0] > PRE_S:
                pre.popleft()
            if idle.tell() > IDLE_MAX:
                idle.close()
                os.replace(out / "idle.jsonl", out / "idle.prev.jsonl")
                idle = open(out / "idle.jsonl", "a")


if __name__ == "__main__":
    main()
