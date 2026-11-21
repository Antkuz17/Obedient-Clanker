import os
import signal
import socket
import sys
import threading
import time

import pytest

from tools import supervisor as S
from tools.supervisor import Child, Supervisor

SLEEPER = [sys.executable, "-c", "import time; time.sleep(60)"]


def run_bg(sup: Supervisor) -> threading.Thread:
    t = threading.Thread(target=sup.run, daemon=True)
    t.start()
    return t


def wait_for(pred, timeout=5.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if pred():
            return True
        time.sleep(0.01)
    return False


def kinds(sup, kind, name=None):
    return [e for e in list(sup.events) if e["kind"] == kind and (name is None or e["name"] == name)]


def test_killed_child_is_restarted_and_timed():
    sup = Supervisor([Child("a", SLEEPER), Child("b", SLEEPER)])
    t = run_bg(sup)
    try:
        assert wait_for(lambda: len(kinds(sup, "start")) == 2)
        a, b = sup.children
        old_pid = a.proc.pid
        os.kill(old_pid, signal.SIGKILL)        # like `slay`
        assert wait_for(lambda: kinds(sup, "restart", "a"))
        ev = kinds(sup, "restart", "a")[0]
        assert ev["signal"] == signal.SIGKILL and ev["pid"] != old_pid
        assert ev["spawn_ms"] < 200             # noticed within a few 20 ms polls
        assert b.restarts == 0 and b.proc.poll() is None   # the other child untouched
    finally:
        sup.stop(); t.join(5)
    assert all(c.proc.poll() is not None for c in sup.children)   # shutdown took them down
    assert len(kinds(sup, "stopped")) == 2


def test_ready_ms_waits_for_tcp_port():
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0)); port = s.getsockname()[1]
    server = [sys.executable, "-c",
              f"import socket,time; time.sleep(0.3); s=socket.socket(); "
              f"s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1); "
              f"s.bind(('127.0.0.1',{port})); s.listen(); time.sleep(60)"]
    sup = Supervisor([Child("srv", server, ready=f"tcp:127.0.0.1:{port}")])
    t = run_bg(sup)
    try:
        assert wait_for(lambda: sup.children[0].proc is not None)
        time.sleep(0.5)
        os.kill(sup.children[0].proc.pid, signal.SIGTERM)
        assert wait_for(lambda: kinds(sup, "restart"), timeout=8)
        ev = kinds(sup, "restart")[0]
        assert ev["ready_ok"] and ev["ready_ms"] >= 300 > ev["spawn_ms"]
    finally:
        sup.stop(); t.join(5)


def test_crash_loop_backs_off(monkeypatch):
    sup = Supervisor([Child("crash", [sys.executable, "-c", "raise SystemExit(3)"])])
    t = run_bg(sup)
    try:
        assert wait_for(lambda: len(kinds(sup, "exit")) >= 4, timeout=8)
        exits = kinds(sup, "exit")
        assert exits[0]["code"] == 3
        delays = [e["restart_in_s"] for e in exits[:4]]
        assert delays == [0.0, 0.25, 0.5, 1.0]
    finally:
        sup.stop(); t.join(5)


def test_restart_false_and_missing_binary():
    sup = Supervisor([Child("once", [sys.executable, "-c", "pass"], restart=False),
                      Child("missing", ["/nonexistent/binary"])])
    t = run_bg(sup)
    try:
        assert wait_for(lambda: kinds(sup, "exit", "once"))
        assert wait_for(lambda: len(kinds(sup, "spawn_failed", "missing")) >= 2, timeout=5)
        time.sleep(0.2)
        assert not kinds(sup, "restart", "once")
    finally:
        sup.stop(); t.join(5)


def test_config_and_validation(tmp_path):
    p = tmp_path / "c.json"
    p.write_text('[{"name": "x", "cmd": ["{python}", "-V"], "prio": 10, "ready": "tcp:h:1"}]')
    (c,) = S.load_config(str(p))
    assert c.cmd[0] == sys.executable and c.prio == 10 and c.restart
    with pytest.raises(ValueError):
        Supervisor([Child("x", ["a"]), Child("x", ["b"])])
    with pytest.raises(ValueError):
        S.check_ready("bogus:1", time.time(), timeout=0.1)
    for cfg in ("tools/supervisor_pi.json", "tools/supervisor_laptop.json"):
        assert S.load_config(cfg)


def test_port_owned_by_someone_else_is_not_ready():
    """macOS AirPlay holds :7000; a child that dies on bind must not be reported ready."""
    squatter = socket.socket()
    squatter.bind(("127.0.0.1", 0)); squatter.listen()
    port = squatter.getsockname()[1]
    try:
        assert S.check_ready(f"tcp:127.0.0.1:{port}", time.time(), timeout=0.5) is True
        assert S.check_ready(f"tcp:127.0.0.1:{port}", time.time(), timeout=0.5,
                             alive=lambda: False) is False
    finally:
        squatter.close()
