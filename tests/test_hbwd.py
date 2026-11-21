"""edge/hbwd (C): built here, then driven over UDP with pymavlink decoding what it sends."""
import json
import shutil
import socket
import subprocess
import threading
import time
from pathlib import Path

import pytest
from pymavlink.dialects.v20 import ardupilotmega as mav

ROOT = Path(__file__).resolve().parent.parent
SRC = ROOT / "edge" / "hbwd" / "hbwd.c"
CC = shutil.which("cc") or shutil.which("gcc") or shutil.which("clang")
pytestmark = pytest.mark.skipif(CC is None, reason="no C compiler")


@pytest.fixture(scope="module")
def binary(tmp_path_factory):
    out = tmp_path_factory.mktemp("hbwd") / "hbwd"
    subprocess.run([CC, "-O2", "-Wall", "-Werror", "-o", str(out), str(SRC), "-lpthread"],
                   check=True)
    return out


def free_udp() -> int:
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


class Rig:
    """Fake FC (decodes MAVLink) + pet sender + hbwd process with its JSON events."""

    def __init__(self, binary, timeout_ms=150):
        self.fc = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.fc.bind(("127.0.0.1", 0))
        self.fc.settimeout(0.05)
        self.pet_port = free_udp()
        self.proc = subprocess.Popen(
            [str(binary), "-l", f"127.0.0.1:{self.pet_port}",
             "-f", f"127.0.0.1:{self.fc.getsockname()[1]}", "-t", str(timeout_ms), "-s", "0.5"],
            stdout=subprocess.PIPE, text=True)
        self.msgs: list[tuple[float, object]] = []
        self.events: list[dict] = []
        self._stop = threading.Event()
        self._pet = threading.Event()
        self.armed, self.ours = 1, 1
        self._parser = mav.MAVLink(None)
        for fn in (self._rx_fc, self._rx_out, self._petter):
            threading.Thread(target=fn, daemon=True).start()
        self.wait_event("start")

    def _rx_fc(self):
        while not self._stop.is_set():
            try:
                data = self.fc.recv(256)
            except socket.timeout:
                continue
            for m in self._parser.parse_buffer(data) or []:
                self.msgs.append((time.monotonic(), m))

    def _rx_out(self):
        for line in self.proc.stdout:
            self.events.append({**json.loads(line), "_mono": time.monotonic()})

    def _petter(self):
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        seq = 0
        while not self._stop.is_set():
            if self._pet.is_set():
                seq += 1
                s.sendto(f"PET {seq} {self.armed} {self.ours}\n".encode(),
                         ("127.0.0.1", self.pet_port))
            time.sleep(0.02)

    def pet(self, on: bool):
        (self._pet.set if on else self._pet.clear)()

    def of(self, kind, since=0.0):
        return [m for t, m in self.msgs if t >= since and m.get_type() == kind]

    def ev(self, kind):
        return [e for e in self.events if e["kind"] == kind]

    def wait_event(self, kind, timeout=3.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self.ev(kind):
                return self.ev(kind)[-1]
            time.sleep(0.01)
        raise AssertionError(f"no {kind} event; got {[e['kind'] for e in self.events]}")

    def close(self):
        self._stop.set()
        self.proc.terminate()
        self.proc.wait(3)
        time.sleep(0.05)


@pytest.fixture
def rig(binary):
    r = Rig(binary)
    yield r
    r.close()


def test_no_heartbeat_before_the_core_pets(rig):
    time.sleep(0.8)
    assert rig.of("HEARTBEAT") == []


def test_heartbeats_while_petted_then_land_when_the_core_stops(rig):
    rig.pet(True)
    time.sleep(1.2)
    hbs = rig.of("HEARTBEAT")
    assert len(hbs) >= 2   # 2 Hz
    hb = hbs[0]
    assert hb.get_srcSystem() == 255 and hb.type == mav.MAV_TYPE_GCS
    assert hb.autopilot == mav.MAV_AUTOPILOT_INVALID

    t_stop = time.monotonic()
    rig.pet(False)                       # the core hangs
    lost = rig.wait_event("core_lost")
    assert lost["action"] == "land" and lost["armed"] and lost["ours"]
    time.sleep(0.6)
    lands = rig.of("SET_MODE", since=t_stop)
    assert len(lands) == 3 and all(m.custom_mode == 9 and m.target_system == 1 for m in lands)
    first_land = next(t for t, m in rig.msgs if t >= t_stop and m.get_type() == "SET_MODE")
    assert first_land - t_stop < 0.35    # 150 ms timeout + a tick or two, not the FC's 2 s
    assert rig.of("HEARTBEAT", since=t_stop + 0.25) == []   # heartbeat dropped too

    rig.pet(True)                        # the core comes back (e.g. supervisor restart)
    rig.wait_event("core_back")
    t_back = time.monotonic()
    time.sleep(0.8)
    assert rig.of("HEARTBEAT", since=t_back)
    assert len(rig.of("SET_MODE")) == 3  # no more LANDs


def test_pilot_mode_is_left_alone(rig):
    rig.ours = 0                         # armed, but the pilot switched to AltHold
    rig.pet(True)
    time.sleep(0.5)
    rig.pet(False)
    lost = rig.wait_event("core_lost")
    assert lost["action"].startswith("none (pilot mode)")
    time.sleep(0.5)
    assert rig.of("SET_MODE") == []      # the FC's own GCS failsafe is the backstop


def test_disarmed_core_loss_sends_nothing(rig):
    rig.armed = rig.ours = 0
    rig.pet(True)
    time.sleep(0.4)
    rig.pet(False)
    rig.wait_event("core_lost")
    time.sleep(0.4)
    assert rig.of("SET_MODE") == []


def test_stats_report_its_own_lateness(rig):
    rig.pet(True)
    time.sleep(1.2)
    st = rig.ev("stats")[-1]
    assert st["n"] >= 20 and sum(st["counts"]) == st["n"] and st["core"] == "ok"
    assert st["heartbeats"] >= 1
