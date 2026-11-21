"""SITL end-to-end for edge/hbwd: the on-drone process layout, then a hung core.

    SITL :5760 <-rawtcp- usb_relay --aux :14556 <-udp- edge core (--hbwd :14560)
                                         ^-------------- hbwd (C) <-pets-'
    observer: SITL's second MAVLink port (:5762) watches the FC mode.

The edge is frozen with SIGSTOP while hovering (the closest thing to a hung core: no crash, no
socket close). hbwd must notice the missing pets and put the FC in LAND within ~0.3 s; the FC's
own GCS failsafe is OFF here, so the LAND can only come from hbwd.

  docker run --rm -d --name hope-sitl-hbwd -p 5770:5760 -p 5772:5762 hope-sitl
  cc -O2 -o edge/hbwd/hbwd edge/hbwd/hbwd.c -lpthread
  python -m tools.sitl_hbwd
"""
from __future__ import annotations

import asyncio
import json
import os
import signal
import subprocess
import sys
import threading
import time
from pathlib import Path

from pymavlink import mavutil

from shared import protocol as P
from shared.client import CommandRejected, EdgeClient

ROOT = Path(__file__).resolve().parent.parent
HBWD = ROOT / "edge" / "hbwd" / "hbwd"
SITL, OBSERVER, EDGE_PORT = "127.0.0.1:5770", "tcp:127.0.0.1:5772", 7100
results: list[tuple[str, bool, str]] = []


def check(name: str, ok: bool, detail: str = "") -> None:
    results.append((name, ok, detail))
    print(f"[{'PASS' if ok else 'FAIL'}] {name} {detail}", flush=True)


class Observer:
    def __init__(self, url: str):
        # SITL opens :5762 only after something connects to :5760, so start the relay first
        for _ in range(10):
            self.m = mavutil.mavlink_connection(url, source_system=254)
            if self.m.wait_heartbeat(timeout=3) is not None:
                break
            self.m.close()
        else:
            raise SystemExit(f"observer: no FC heartbeat on {url}")
        self.mode, self.armed, self.changes = "?", False, []
        threading.Thread(target=self._run, daemon=True).start()

    def _run(self):
        while True:
            msg = self.m.recv_match(type="HEARTBEAT", blocking=True, timeout=1)
            if msg is None or msg.get_srcSystem() != 1:
                continue
            mode = mavutil.mode_string_v10(msg)
            armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            if mode != self.mode:
                self.changes.append((time.monotonic(), mode))
            self.mode, self.armed = mode, armed

    def set_param(self, name: str, val: float):
        self.m.mav.param_set_send(1, 1, name.encode(), val, 9)


async def main() -> int:
    if not HBWD.exists():
        print(f"build it first: cc -O2 -o {HBWD.relative_to(ROOT)} edge/hbwd/hbwd.c -lpthread")
        return 2
    py = sys.executable
    procs = {
        "relay": subprocess.Popen([py, "-m", "tools.usb_relay", "--source", f"rawtcp:{SITL}",
                                   "--aux", "127.0.0.1:14556"], cwd=ROOT),
        "hbwd": subprocess.Popen([str(HBWD), "-t", "250", "-s", "5"], cwd=ROOT,
                                 stdout=subprocess.PIPE, text=True),
        "edge": subprocess.Popen([py, "-m", "edge", "--mav", "udpin:127.0.0.1:14555", "--sitl",
                                  "--hbwd", "127.0.0.1:14560", "--port", str(EDGE_PORT),
                                  "--no-video", "--log", "", "--no-person-check"], cwd=ROOT),
    }
    hb_events: list[dict] = []
    def read_hbwd():
        for line in procs["hbwd"].stdout:
            ev = json.loads(line)
            hb_events.append(ev)
            if ev["kind"] != "stats":
                print("   hbwd:", line.strip(), flush=True)
    threading.Thread(target=read_hbwd, daemon=True).start()
    try:
        await asyncio.sleep(3)
        obs = Observer(OBSERVER)
        obs.set_param("FS_GCS_ENABLE", 0)  # so a LAND can only come from hbwd
        edge = await EdgeClient("127.0.0.1", EDGE_PORT).connect()
        t0, took_off = time.monotonic(), False
        while time.monotonic() - t0 < 120 and not took_off:
            try:
                d = await edge.run("takeoff", alt=1.0, timeout=40)
                took_off = d["status"] == P.OK
                if not took_off:
                    print("   takeoff:", d["status"], d.get("reason"), flush=True)
            except CommandRejected as e:
                print(f"   takeoff not yet: {e}", flush=True)
            if not took_off:
                await asyncio.sleep(5)
        check("takeoff through relay + edge + hbwd", took_off and obs.mode == "GUIDED",
              f"mode {obs.mode}")
        if not took_off:
            return 1
        await asyncio.sleep(3)
        check("hbwd sees the core petting", any(e.get("core") == "ok" for e in hb_events))

        t_stop = time.monotonic()
        procs["edge"].send_signal(signal.SIGSTOP)
        print("   edge frozen (SIGSTOP)", flush=True)
        while time.monotonic() - t_stop < 5 and obs.mode != "LAND":
            await asyncio.sleep(0.01)
        land_t = next((t for t, m in obs.changes if t >= t_stop and m == "LAND"), None)
        dt = (land_t - t_stop) if land_t else None
        check("hung core -> FC in LAND, from hbwd (FC failsafe off)",
              dt is not None and dt < 1.0, f"{dt * 1000:.0f} ms after SIGSTOP" if dt else
              f"mode still {obs.mode}")
        t1 = time.monotonic()
        while obs.armed and time.monotonic() - t1 < 30:
            await asyncio.sleep(0.2)
        check("...landed and disarmed", obs.mode != "?" and not obs.armed, f"mode {obs.mode}")
        procs["edge"].send_signal(signal.SIGCONT)
        await asyncio.sleep(1.0)
        check("core back -> hbwd resumes", bool([e for e in hb_events if e["kind"] == "core_back"]))
        await edge.close()
    finally:
        for p in procs.values():
            try:
                p.send_signal(signal.SIGCONT)
                p.terminate()
                p.wait(5)
            except Exception:
                p.kill()
    ok = all(r[1] for r in results)
    print("ALL PASS" if ok else "SOME FAILED")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(asyncio.run(main()))
