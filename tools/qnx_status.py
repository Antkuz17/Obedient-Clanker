"""QNX proof for the dashboard: what is running on the drone's QNX computer, at what real-time
priority, how the hard real-time loop is keeping time, and which QNX AI packages do the work.

  sudo python3 -m tools.qnx_status          # on the Pi (root: to see every process); one JSON line
  sudo python3 -m tools.qnx_status --full   # + package and module versions (slower: imports cv2)

tools/voice_flights polls it over SSH every few seconds for its "Running on QNX" panel.
"""
from __future__ import annotations

import json
import re
import subprocess
import sys
from pathlib import Path

HOME = Path("/data/home/qnxuser/hope")
# supervisor child -> what it does (the order is the order of importance on the page)
ROLES = {
    "hbwd": "C heartbeat watchdog: lands the drone if the core hangs",
    "usb_relay": "USB link to the flight controller",
    "edge": "safety core: every command passes its gate",
    "rangefinder": "ultrasonic height → flight controller",
    "flow": "optical flow (QNX OpenCV) → flight controller",
    "camgrab_down": "down camera (QNX camera API, C)",
    "camgrab_fwd": "forward camera (QNX camera API, C)",
    "vision": "person detector (QNX TensorFlow Lite)",
    "slog2d": "drone events → QNX system log (slog2)",
    "video": "live video to the laptop",
    "blackbox": "flight recorder",
    "camrec": "camera recorder",
    "sensor_keeper": "restarts QNX's camera service if it dies",
}
POLICY = {"f": "FIFO", "r": "RR", "o": "OTHER", "s": "SPORADIC"}


def sh(cmd: str, timeout: float = 10) -> str:
    try:
        return subprocess.run(cmd, shell=True, capture_output=True, text=True,
                              timeout=timeout).stdout
    except subprocess.TimeoutExpired:
        return ""


def supervisor_children() -> dict[str, dict]:
    """name -> {pid, restarts, state} from the supervisor's own event log."""
    kids: dict[str, dict] = {}
    path = HOME / "logs" / "supervisor_events.jsonl"
    lines = path.read_text(errors="replace").splitlines()[-3000:] if path.exists() else []
    for line in lines:
        try:
            e = json.loads(line)
        except ValueError:
            continue
        name = e.get("name")
        if not name:
            continue
        k = kids.setdefault(name, {"pid": None, "restarts": 0, "state": "?"})
        if e.get("kind") == "start":  # a fresh stack start resets the count
            k.update(pid=e.get("pid"), restarts=0, state="up")
        elif e.get("kind") == "restart":
            k.update(pid=e.get("pid"), restarts=k["restarts"] + 1, state="up")
        elif e.get("kind") in ("exit", "spawn_failed"):
            k["state"] = "down"
    return kids


def thread_prios() -> dict[int, list[str]]:
    """pid -> ['60f', '10o', ...] (every thread's priority + policy letter)."""
    out: dict[int, list[str]] = {}
    for line in sh("pidin -F '%I %N %p'").splitlines():
        m = re.match(r"\s*(\d+)-(\d+)\s+(\S+)\s+(\d+)([a-z])", line)
        if m:
            out.setdefault(int(m.group(1)), []).append(m.group(4) + m.group(5))
    return out


def driver_prios(prios: dict[int, list[str]]) -> dict[str, int]:
    names: dict[str, int] = {}
    for line in sh("pidin -F '%I %N'").splitlines():
        m = re.match(r"\s*(\d+)-01\s+(\S+)", line)
        if m and m.group(2) in ("io-usb-otg", "io-sock", "sensor"):
            ps = [int(p[:-1]) for p in prios.get(int(m.group(1)), [])]
            if ps:  # the working threads' priority (the most common); 254 = interrupt threads
                names[m.group(2)] = max(set(ps), key=ps.count)
    return names


def hbwd() -> dict:
    path = HOME / "logs" / "hbwd.log"
    stats = []
    for line in (path.read_text(errors="replace").splitlines()[-200:] if path.exists() else []):
        try:
            e = json.loads(line)
        except ValueError:
            continue
        if e.get("kind") == "stats":
            stats.append(e)
    if not stats:
        return {}
    last = stats[-1]
    win = stats[-60:]
    return {"hz": last.get("n"), "max_ms_1s": last.get("max_ms"),
            "max_ms_60s": max(s.get("max_ms", 0) for s in win),
            "late_60s": sum(s.get("over_2ms", 0) for s in win),
            "trace": [round(s.get("max_ms", 0), 3) for s in win],
            "pets": last.get("pets"), "lands": last.get("lands"), "core": last.get("core")}


def vision() -> dict:
    path = HOME / "logs" / "vision.log"
    for line in reversed(path.read_text(errors="replace").splitlines()[-80:] if path.exists() else []):
        m = re.search(r"running: ([\d.]+) Hz, infer (\d+) ms", line)
        if m:
            return {"hz": float(m.group(1)), "infer_ms": int(m.group(2)), "at": line[:19]}
    return {}


def slog(n: int = 8) -> list[str]:
    out = []
    for line in sh("slog2info -b hope").splitlines()[-n:]:
        m = re.match(r"\w+\s+\d+\s+([\d:.]+)\s+\S+\s+\S+\s+\d+\s+(.*)", line)
        out.append(f"{m.group(1)[:12]}  {m.group(2)}" if m else line.strip())
    return out


def packages() -> dict:
    pk = {}
    for line in sh("apk info -v 2>/dev/null").splitlines():
        for name in ("python3-tflite-runtime", "tflite-runtime", "python3-opencv",
                     "libopencv_optflow", "libopencv_video", "python3-numpy", "libusb"):
            if line.startswith(name + "-") and line[len(name) + 1:len(name) + 2].isdigit():
                pk[name] = line[len(name) + 1:]
    mods = sh("python3 -c \"import tflite_runtime, cv2, sys; print(tflite_runtime.__version__, "
              "cv2.__version__, sys.version.split()[0])\"", timeout=30).split()
    if len(mods) == 3:
        pk.update(tflite_module=mods[0], opencv_module=mods[1], python=mods[2])
    return pk


def main() -> None:
    prios = thread_prios()
    kids = supervisor_children()
    procs = []
    for name, role in ROLES.items():
        k = kids.get(name, {})
        ps = prios.get(k.get("pid") or -1, [])
        main_t = ps[0] if ps else None
        procs.append({"name": name, "role": role, "pid": k.get("pid"), "state": k.get("state", "?"),
                      "restarts": k.get("restarts", 0),
                      "prio": int(main_t[:-1]) if main_t else None,
                      "policy": POLICY.get(main_t[-1], main_t[-1]) if main_t else None})
    boot = re.search(r"BootTime:(.*)", sh("pidin info"))
    snap = {"os": sh("uname -a").strip(), "boot": boot.group(1).strip() if boot else None,
            "cpus": len(re.findall(r"^Processor\d+:", sh("pidin info"), re.M)),
            "procs": procs, "drivers": driver_prios(prios), "hbwd": hbwd(), "vision": vision(),
            "slog": slog()}
    if "--full" in sys.argv:
        snap["packages"] = packages()
    print(json.dumps(snap), flush=True)


if __name__ == "__main__":
    main()
