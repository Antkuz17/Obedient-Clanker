"""QNX demo: cause one failure on the running drone stack and measure how it recovers.

Runs ON THE PI (tools/voice_flights calls it over SSH; the drone must be disarmed):
  python3 -m tools.qnx_faults kill_vision          # SIGKILL the person detector
  python3 -m tools.qnx_faults freeze_core          # SIGSTOP the safety core for 3 s
  sudo python3 -m tools.qnx_faults kill_camera_service   # slay QNX's camera service (root)

Prints one JSON object: what was done and the measured recovery.
- kill_vision: our supervisor (tools/supervisor.py) notices the death and respawns it; reports
  the respawn time it measured.
- freeze_core: the C heartbeat watchdog (edge/hbwd, top SCHED_FIFO priority) notices the frozen
  core; meanwhile we count the optical flow and rangefinder messages the FC keeps receiving
  (they go to the FC directly, not through the core). Disarmed, hbwd doesn't land (nothing to
  land); armed in GUIDED it would send LAND 250 ms in.
- kill_camera_service: tools/sensor_keeper restarts QNX's sensor service; the camgrabs come back.
"""
from __future__ import annotations

import json
import os
import signal
import subprocess
import sys
import threading
import time
from pathlib import Path

HOME = Path("/data/home/qnxuser/hope")
EVENTS = HOME / "logs" / "supervisor_events.jsonl"
HBWD_LOG = HOME / "logs" / "hbwd.log"


def pid_of(pattern: str) -> int | None:
    out = subprocess.run(["pidin", "-F", "%I %a", "ar"], capture_output=True, text=True).stdout
    for line in out.splitlines():
        if pattern in line and "qnx_faults" not in line and "grep" not in line:
            return int(line.split()[0].split("-")[0])
    return None


def events_after(t: float, name: str | None = None) -> list[dict]:
    out = []
    for line in EVENTS.read_text(errors="replace").splitlines()[-200:]:
        try:
            e = json.loads(line)
        except ValueError:
            continue
        if e.get("t", 0) >= t and (name is None or e.get("name") == name):
            out.append(e)
    return out


def wait_restart(name: str, t: float, secs: float = 8.0) -> dict | None:
    end = time.time() + secs
    while time.time() < end:
        for e in events_after(t, name):
            if e.get("kind") == "restart" and (e.get("ready_ms") is not None or
                                               "ready_ok" not in e):
                return e
        time.sleep(0.1)
    rs = [e for e in events_after(t, name) if e.get("kind") == "restart"]
    return rs[-1] if rs else None


def kill_vision() -> dict:
    pid = pid_of("edge.vision")
    if pid is None:
        return {"fault": "kill_vision", "ok": False, "error": "person detector not running"}
    t = time.time()
    os.kill(pid, signal.SIGKILL)
    e = wait_restart("vision", t)
    return {"fault": "kill_vision", "ok": e is not None, "pid": pid,
            "new_pid": e and e.get("pid"), "respawn_ms": e and e.get("spawn_ms")}


def count_fc_inputs(secs: float, out: dict) -> None:
    """OPTICAL_FLOW / DISTANCE_SENSOR the FC reports while the core is frozen (relay tap)."""
    try:
        from tools import fclink
        m = fclink.connect(fclink.TAP, timeout=5)
        for mid, hz in ((fclink.M.MAVLINK_MSG_ID_OPTICAL_FLOW, 10),
                        (fclink.M.MAVLINK_MSG_ID_DISTANCE_SENSOR, 10)):
            fclink.stream(m, mid, hz)
        n = {"OPTICAL_FLOW": 0, "DISTANCE_SENSOR": 0}
        out["ready"] = True
        end = time.monotonic() + secs
        while time.monotonic() < end:
            msg = m.recv_match(type=list(n), blocking=True, timeout=0.3)
            if msg is not None and msg.get_srcSystem() == 1:
                n[msg.get_type()] += 1
        out["flow_hz"] = round(n["OPTICAL_FLOW"] / secs, 1)
        out["range_hz"] = round(n["DISTANCE_SENSOR"] / secs, 1)
    except Exception as e:  # never leave the core frozen because of the counter
        out["error"] = repr(e)
        out["ready"] = True


def freeze_core(secs: float = 3.0) -> dict:
    pid = pid_of("-m edge ")
    if pid is None:
        return {"fault": "freeze_core", "ok": False, "error": "edge core not running"}
    counts: dict = {}
    th = threading.Thread(target=count_fc_inputs, args=(secs, counts), daemon=True)
    th.start()
    end = time.time() + 5
    while not counts.get("ready") and time.time() < end:
        time.sleep(0.05)
    t = time.time()
    os.kill(pid, signal.SIGSTOP)
    try:
        time.sleep(secs)
    finally:
        os.kill(pid, signal.SIGCONT)
    th.join(5)
    time.sleep(1.0)
    lost = back = None
    for line in HBWD_LOG.read_text(errors="replace").splitlines()[-400:]:
        try:
            e = json.loads(line)
        except ValueError:
            continue
        if e.get("t", 0) >= t - 0.5:
            if e.get("kind") == "core_lost" and lost is None:
                lost = e
            elif e.get("kind") == "core_back":
                back = e
    return {"fault": "freeze_core", "ok": lost is not None, "frozen_s": secs,
            "watchdog_noticed_ms": lost and lost.get("gap_ms"),
            "core_back_after_ms": back and back.get("gap_ms"),
            "land_sent": bool(lost and lost.get("armed")), "restarted": False,
            "flow_hz": counts.get("flow_hz"), "range_hz": counts.get("range_hz")}


def kill_camera_service() -> dict:
    t = time.time()
    subprocess.run(["slay", "-f", "sensor"], capture_output=True)
    gone = time.time()
    back = None
    end = time.time() + 20
    while time.time() < end:
        if Path("/dev/sensor/camera5").exists() and time.time() - gone > 0.5:
            back = time.time()
            break
        time.sleep(0.1)
    cams = {}
    end = time.time() + 15
    while time.time() < end and len(cams) < 2:
        for name in ("camgrab_fwd", "camgrab_down"):
            if name not in cams and (e := wait_restart(name, t, 0.2)) is not None:
                cams[name] = round((e["t"] - t) * 1000)
        time.sleep(0.2)
    return {"fault": "kill_camera_service", "ok": back is not None,
            "service_back_ms": back and round((back - t) * 1000),
            "cameras_back_ms": cams}


def fc_armed() -> bool | None:
    """The FC's armed flag from its heartbeat (relay tap); None if it can't be read."""
    try:
        from tools import fclink
        m = fclink.connect(fclink.TAP, timeout=5)
        end = time.monotonic() + 4
        while time.monotonic() < end:
            hb = m.recv_match(type="HEARTBEAT", blocking=True, timeout=0.5)
            if hb is not None and hb.get_srcSystem() == 1 and hb.type != fclink.M.MAV_TYPE_GCS:
                return bool(hb.base_mode & fclink.M.MAV_MODE_FLAG_SAFETY_ARMED)
    except Exception:
        pass
    return None


if __name__ == "__main__":
    if fc_armed() is not False:  # armed, or can't tell: these faults are for a drone on the ground
        print(json.dumps({"fault": sys.argv[1] if len(sys.argv) > 1 else "?", "ok": False,
                          "error": "the drone is armed (or its state can't be read): "
                                   "faults only run on the ground"}), flush=True)
        raise SystemExit(0)
    fn = {"kill_vision": kill_vision, "freeze_core": freeze_core,
          "kill_camera_service": kill_camera_service}.get(sys.argv[1] if len(sys.argv) > 1 else "")
    if fn is None:
        raise SystemExit("usage: python3 -m tools.qnx_faults kill_vision|freeze_core|kill_camera_service")
    print(json.dumps(fn()), flush=True)
