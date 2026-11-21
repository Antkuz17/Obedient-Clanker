"""Summarise a blackbox flight: how it flew, in plain numbers.

  python3 -m tools.flight_report                       # newest logs/blackbox/flight_*.jsonl
  python3 -m tools.flight_report logs/blackbox/flight_20261121_204500.jsonl
  python -m tools.flight_report --pi                   # from the laptop: fetch the newest from the Pi
"""
from __future__ import annotations

import argparse
import json
import math
import statistics
from collections import Counter
from pathlib import Path

MODES = {0: "STABILIZE", 2: "ALT_HOLD", 3: "AUTO", 4: "GUIDED", 5: "LOITER", 6: "RTL", 9: "LAND"}


def load(path: Path) -> list[dict]:
    out = []
    for line in path.open():
        try:
            out.append(json.loads(line))
        except ValueError:
            pass
    return out


def report(recs: list[dict]) -> str:
    by = lambda t: [r for r in recs if r.get("mavpackettype") == t]
    hb = [r for r in by("HEARTBEAT") if r.get("type") != 6]
    armed_t = [r["_t"] for r in hb if r.get("base_mode", 0) & 128]
    if not armed_t:
        return "no armed period in this file"
    t0, t1 = min(armed_t), max(armed_t)
    inflight = lambda rs: [r for r in rs if t0 <= r["_t"] <= t1]
    lines = [f"armed for {t1 - t0:.0f} s"]

    modes = []
    for r in inflight(hb):
        m = MODES.get(r.get("custom_mode"), str(r.get("custom_mode")))
        if not modes or modes[-1] != m:
            modes.append(m)
    lines.append("modes: " + " -> ".join(modes))

    ds = [r["current_distance"] / 100 for r in inflight(by("DISTANCE_SENSOR"))]
    if ds:
        valid = [d for d in ds if d <= 2.0]
        lines.append(f"rangefinder: {min(valid, default=0):.2f}-{max(valid, default=0):.2f} m, "
                     f"dropouts {100 * (len(ds) - len(valid)) / len(ds):.0f}% of {len(ds)}")
    lp = inflight(by("LOCAL_POSITION_NED"))
    if lp:
        alts = [-r["z"] for r in lp]
        n0, e0 = lp[0]["x"], lp[0]["y"]
        drift = [math.hypot(r["x"] - n0, r["y"] - e0) for r in lp]
        lines.append(f"EKF height {min(alts):.2f}-{max(alts):.2f} m; horizontal wander max "
                     f"{max(drift):.2f} m (estimate)")
    of = inflight(by("OPTICAL_FLOW"))
    if of:
        q = [r["quality"] for r in of]
        lines.append(f"optical flow quality mean {statistics.mean(q):.0f}, below 100 "
                     f"{100 * sum(x < 100 for x in q) / len(q):.0f}% of the time")
    bat = inflight(by("SYS_STATUS"))
    if bat:
        v = [r["voltage_battery"] / 1000 for r in bat if 0 < r["voltage_battery"] < 65535]
        if v:
            lines.append(f"battery {max(v):.2f} V -> min {min(v):.2f} V under load "
                         f"(sag {max(v) - min(v):.2f} V)")
    vib = inflight(by("VIBRATION"))
    if vib:
        vmax = max(max(r["vibration_x"], r["vibration_y"], r["vibration_z"]) for r in vib)
        clip = max(r.get("clipping_0", 0) for r in vib)
        lines.append(f"vibration max {vmax:.0f} m/s/s ({'OK' if vmax < 30 else 'HIGH: check mounts/props'}),"
                     f" accel clipping {clip}")
    ekf = inflight(by("EKF_STATUS_REPORT"))
    if ekf:
        lost = sum(1 for r in ekf if not (r["flags"] & 8))
        lines.append(f"EKF relative position lost in {100 * lost / len(ekf):.0f}% of samples; "
                     f"max velocity variance {max(r['velocity_variance'] for r in ekf):.2f}")
    att = inflight(by("ATTITUDE"))
    if att:
        tilt = max(math.degrees(max(abs(r["roll"]), abs(r["pitch"]))) for r in att)
        lines.append(f"max tilt {tilt:.0f} deg")
    texts = Counter(r["text"] for r in by("STATUSTEXT") if t0 - 5 <= r["_t"] <= t1 + 10)
    if texts:
        lines.append("FC messages: " + "; ".join(f"{t} (x{n})" if n > 1 else t
                                                for t, n in texts.most_common(8)))
    return "\n".join(lines)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("path", nargs="?")
    ap.add_argument("--dir", default="logs/blackbox")
    ap.add_argument("--pi", action="store_true", help="fetch the newest flight from the Pi first")
    a = ap.parse_args()
    if a.pi:
        import os
        import paramiko
        c = paramiko.SSHClient()
        c.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        c.connect(os.environ.get("QNX_HOST", "192.168.68.131"), username="qnxuser",
                  password=os.environ.get("QNX_PASS", "qnxuser"), look_for_keys=False,
                  allow_agent=False, timeout=10)
        sf = c.open_sftp()
        remote = "/data/home/qnxuser/hope/logs/blackbox"
        names = sorted(n for n in sf.listdir(remote) if n.startswith("flight_"))
        if not names:
            raise SystemExit("no flights recorded on the Pi yet")
        Path(a.dir).mkdir(parents=True, exist_ok=True)
        sf.get(f"{remote}/{names[-1]}", str(Path(a.dir) / names[-1]))
        a.path = str(Path(a.dir) / names[-1])
    path = Path(a.path) if a.path else max(Path(a.dir).glob("flight_*.jsonl"),
                                          key=lambda p: p.stat().st_mtime)
    print(f"== {path.name}")
    print(report(load(path)))


if __name__ == "__main__":
    main()
