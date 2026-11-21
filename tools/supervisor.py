"""Process supervisor for the drone (no HAM on the QNX image; PRD D14, demo "slay + restart").

Starts the onboard processes, restarts any that die, and logs how long each restart took:

  python3 -m tools.supervisor tools/supervisor_pi.json
  python3 -m tools.supervisor tools/supervisor_laptop.json --events-udp 127.0.0.1:7010

  slay camgrab        # (QNX) or kill <pid>: the supervisor logs
  {"kind": "restart", "name": "camgrab_fwd", "signal": 15, "spawn_ms": 1.9, "ready_ms": 143.0, ...}

Config: a JSON list of children:
  {"name": "camgrab_fwd",                       # unique
   "cmd": ["./edge/camgrab/camgrab", "-u", "5", "-n", "camgrab_fwd"],
   "prio": 40,                                  # optional SCHED_FIFO priority (needs root on QNX)
   "ready": "shm:camgrab_fwd",                  # optional: shm:<name> (a new frame after spawn),
                                                #   tcp:<host>:<port> (accepts a connection)
   "restart": true,                             # optional, default true
   "user": "qnxuser",                           # optional: drop to this user (supervisor as root)
   "env": {"PYTHONPATH": "."}}                  # optional extra environment
"{python}" in cmd is replaced by the interpreter running the supervisor.

Timing: `spawn_ms` = death noticed -> new process started; `ready_ms` = death noticed -> the
ready check passed (None if the child has no check). Death is noticed within POLL_S.
A child that keeps dying is restarted with backoff (0, 0.25, 0.5 ... up to 5 s) so a broken
binary doesn't spin the CPU; the backoff resets after it stays up STABLE_S.

Stdlib only (runs on the Pi's Python 3.14); the shm check imports edge.camera (numpy) lazily.
"""
from __future__ import annotations

import argparse
import json
import logging
import os
import signal
import socket
import subprocess
import sys
import threading
import time
from dataclasses import dataclass, field
from typing import Callable

log = logging.getLogger("supervisor")

POLL_S = 0.02          # 50 Hz death check
STABLE_S = 10.0        # up this long -> crash backoff resets
BACKOFF_MAX_S = 5.0
READY_TIMEOUT_S = 10.0
STOP_GRACE_S = 2.0


@dataclass
class Child:
    name: str
    cmd: list[str]
    prio: int | None = None
    ready: str | None = None
    restart: bool = True
    env: dict[str, str] = field(default_factory=dict)
    user: str | None = None
    # runtime
    proc: subprocess.Popen | None = None
    started_at: float = 0.0
    restarts: int = 0
    fails_in_a_row: int = 0
    next_start: float = 0.0
    died_at: float | None = None
    last_signal: int | None = None
    last_code: int | None = None

    @classmethod
    def from_dict(cls, d: dict) -> "Child":
        if not d.get("name") or not d.get("cmd"):
            raise ValueError(f"child needs name and cmd: {d}")
        # "{python}" -> this interpreter, so configs work from a venv
        cmd = [sys.executable if a == "{python}" else a for a in d["cmd"]]
        return cls(name=d["name"], cmd=cmd, prio=d.get("prio"), ready=d.get("ready"),
                   restart=d.get("restart", True), env=dict(d.get("env", {})),
                   user=d.get("user"))


def _realtime(prio: int | None):
    if prio is None or not hasattr(os, "sched_setscheduler"):
        return None
    def apply() -> None:  # runs in the child between fork and exec
        try:
            os.sched_setscheduler(0, os.SCHED_FIFO, os.sched_param(prio))
        except OSError:
            pass  # not root: run at normal priority rather than not at all
    return apply


def check_ready(spec: str, since: float, timeout: float = READY_TIMEOUT_S,
                stop: threading.Event | None = None,
                alive: "Callable[[], bool] | None" = None) -> bool:
    """True once the child is serving again. `since` is time.time() at spawn. `alive` (the
    child is still the same running process) is re-checked after a pass: a tcp port can be
    answered by someone else (macOS AirPlay holds :7000), so a dead child is never "ready"."""
    ok = _probe(spec, since, timeout, stop)
    return ok and (alive is None or alive())


def _probe(spec: str, since: float, timeout: float, stop: threading.Event | None) -> bool:
    kind, _, arg = spec.partition(":")
    deadline = time.monotonic() + timeout
    if kind == "shm":
        from edge.camera import ShmFrameReader
        reader = None
        try:
            while time.monotonic() < deadline and not (stop and stop.is_set()):
                try:
                    if reader is None:
                        reader = ShmFrameReader(arg, open_timeout=0.2)
                    f = reader.wait_new(timeout=0.2)
                    if f is not None and f.t >= since:
                        return True
                except (FileNotFoundError, OSError, ValueError, TimeoutError):
                    reader = None
                    time.sleep(0.01)
        finally:
            if reader is not None:
                reader.close()
        return False
    if kind == "tcp":
        host, _, port = arg.rpartition(":")
        while time.monotonic() < deadline and not (stop and stop.is_set()):
            try:
                with socket.create_connection((host or "127.0.0.1", int(port)), timeout=0.2):
                    return True
            except OSError:
                time.sleep(0.01)
        return False
    raise ValueError(f"unknown ready check {spec!r} (use shm:<name> or tcp:<host>:<port>)")


class Supervisor:
    def __init__(self, children: list[Child], events_udp: tuple[str, int] | None = None,
                 log_path: str | None = None):
        names = [c.name for c in children]
        if len(set(names)) != len(names):
            raise ValueError(f"duplicate child names: {names}")
        self.children = children
        self.events: list[dict] = []
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self._udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) if events_udp else None
        self._udp_addr = events_udp
        self._log = open(log_path, "a", buffering=1) if log_path else None

    # --- events ------------------------------------------------------------------------------
    def emit(self, kind: str, **kw) -> dict:
        ev = {"t": time.time(), "kind": kind, **kw}
        with self._lock:
            self.events.append(ev)
        line = json.dumps(ev)
        print(line, flush=True)
        if self._log:
            self._log.write(line + "\n")
        if self._udp:
            try:
                self._udp.sendto(line.encode(), self._udp_addr)
            except OSError:
                pass
        return ev

    # --- lifecycle ---------------------------------------------------------------------------
    def _spawn(self, c: Child) -> None:
        env = {**os.environ, **c.env}
        kw = {"user": c.user} if c.user and os.geteuid() == 0 else {}
        c.proc = subprocess.Popen(c.cmd, env=env, preexec_fn=_realtime(c.prio),
                                  start_new_session=True, **kw)
        c.started_at = time.monotonic()
        wall = time.time()
        if c.died_at is None:
            self.emit("start", name=c.name, pid=c.proc.pid, prio=c.prio)
            return
        died, c.died_at = c.died_at, None
        spawn_ms = round((c.started_at - died) * 1000, 1)
        c.restarts += 1
        info = dict(name=c.name, pid=c.proc.pid, restarts=c.restarts, signal=c.last_signal,
                    code=c.last_code, spawn_ms=spawn_ms)
        if not c.ready:
            self.emit("restart", ready_ms=None, **info)
            return
        proc = c.proc
        def wait_ready() -> None:
            ok = check_ready(c.ready, wall, stop=self._stop,
                             alive=lambda: c.proc is proc and proc.poll() is None)
            if self._stop.is_set():
                return
            self.emit("restart", ready_ms=round((time.monotonic() - died) * 1000, 1)
                      if ok else None, ready_ok=ok, **info)
        threading.Thread(target=wait_ready, daemon=True, name=f"ready-{c.name}").start()

    def _reap(self, c: Child, now: float) -> None:
        code = c.proc.poll()
        if code is None:
            if c.fails_in_a_row and now - c.started_at > STABLE_S:
                c.fails_in_a_row = 0
            return
        c.died_at = now
        c.last_signal, c.last_code = (-code, None) if code < 0 else (None, code)
        c.proc = None
        up_s = now - c.started_at
        c.fails_in_a_row = c.fails_in_a_row + 1 if up_s < STABLE_S else 1
        delay = 0.0 if c.fails_in_a_row <= 1 else min(BACKOFF_MAX_S,
                                                      0.25 * 2 ** (c.fails_in_a_row - 2))
        c.next_start = now + delay
        self.emit("exit", name=c.name, signal=c.last_signal, code=c.last_code,
                  up_s=round(up_s, 2), restart_in_s=delay if c.restart else None)
        if not c.restart:
            c.died_at = None

    def step(self) -> None:
        now = time.monotonic()
        for c in self.children:
            if c.proc is not None:
                self._reap(c, now)
            if c.proc is None and c.restart and c.died_at is not None and now >= c.next_start:
                try:
                    self._spawn(c)
                except OSError as e:  # binary missing etc.: keep trying with backoff
                    c.fails_in_a_row += 1
                    c.next_start = now + min(BACKOFF_MAX_S, 0.25 * 2 ** c.fails_in_a_row)
                    self.emit("spawn_failed", name=c.name, error=str(e))

    def start(self) -> None:
        for c in self.children:
            try:
                self._spawn(c)
            except OSError as e:
                c.died_at, c.next_start = time.monotonic(), time.monotonic() + 1.0
                self.emit("spawn_failed", name=c.name, error=str(e))

    def run(self) -> None:
        self.start()
        try:
            while not self._stop.is_set():
                self.step()
                self._stop.wait(POLL_S)
        finally:
            self.shutdown()

    def stop(self) -> None:
        self._stop.set()

    def shutdown(self) -> None:
        self._stop.set()
        live = [c for c in self.children if c.proc is not None and c.proc.poll() is None]
        for c in live:
            try:
                os.killpg(c.proc.pid, signal.SIGTERM)
            except (ProcessLookupError, PermissionError):
                c.proc.terminate()
        deadline = time.monotonic() + STOP_GRACE_S
        for c in live:
            try:
                c.proc.wait(timeout=max(0.0, deadline - time.monotonic()))
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(c.proc.pid, signal.SIGKILL)
                except (ProcessLookupError, PermissionError):
                    c.proc.kill()
                c.proc.wait()
        for c in live:
            self.emit("stopped", name=c.name)
        if self._log:
            self._log.close()
            self._log = None


def load_config(path: str) -> list[Child]:
    with open(path) as f:
        return [Child.from_dict(d) for d in json.load(f)]


def main(argv: list[str] | None = None) -> None:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("config", help="JSON list of children")
    ap.add_argument("--events-udp", help="host:port to send each event to as JSON")
    ap.add_argument("--log", help="append events (JSON lines) to this file")
    a = ap.parse_args(argv)
    logging.basicConfig(level=logging.INFO, format="%(levelname)s:%(name)s:%(message)s")
    udp = None
    if a.events_udp:
        h, _, p = a.events_udp.rpartition(":")
        udp = (h, int(p))
    sup = Supervisor(load_config(a.config), events_udp=udp, log_path=a.log)
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: sup.stop())
    sup.run()


if __name__ == "__main__":
    main(sys.argv[1:])
