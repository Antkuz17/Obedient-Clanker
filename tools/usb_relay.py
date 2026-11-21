"""Root-only USB <-> UDP byte pump for the flight controller (review A9: a user-space USB driver).

libusb on QNX only works as root, and the edge core shouldn't run as root (it serves a network
port). So this tiny process owns the USB device and nothing else:

    FC  <--USB CDC-ACM-->  usb_relay (root)  <--UDP 127.0.0.1:14555-->  edge core (qnxuser)

  sudo env PYTHONPATH=$HOME/.local/lib/python3.14/site-packages python3 -m tools.usb_relay
  python3 -m edge --mav udpin:127.0.0.1:14555 --station-ip <laptop ip> ...

No MAVLink parsing here: bytes in, bytes out. If USB drops, it reopens the device (review A5);
the edge notices the gap (no FC heartbeat) and re-runs its stream/origin setup.
For testing without hardware: --source rawtcp:127.0.0.1:5760 (ArduPilot SITL).
"""
from __future__ import annotations

import argparse
import logging
import os
import socket
import threading
import time

from edge.mavlink_usb import UsbCdc

log = logging.getLogger("usb_relay")
SILENT_REOPEN_S = 3.0  # FC silent this long -> reopen the USB device
SILENT_EXIT_S = 5.0
TAP_PEER_TTL_S = 30.0  # a tap tool must send something at least this often to keep getting data    # still silent (read stuck inside libusb) -> exit; the supervisor restarts us


class TcpStream:
    def __init__(self, host: str, port: int):
        self.s = socket.create_connection((host, port))

    def read(self, timeout_s: float) -> bytes:
        self.s.settimeout(timeout_s)
        try:
            data = self.s.recv(4096)
        except socket.timeout:
            return b""
        if not data:
            raise ConnectionError("source closed")
        return data

    def write(self, buf: bytes) -> None:
        self.s.sendall(buf)

    def close(self) -> None:
        self.s.close()


def open_source(spec: str):
    if spec == "usb":
        return UsbCdc()
    if spec.startswith("rawtcp:"):
        _, host, port = spec.split(":")
        return TcpStream(host, int(port))
    raise SystemExit(f"unknown source {spec!r} (usb or rawtcp:HOST:PORT)")


class Relay:
    def __init__(self, source: str, core_addr: tuple[str, int],
                 aux_addr: tuple[str, int] | None = None,
                 tap_addr: tuple[str, int] | None = None):
        self.source_spec, self.core_addr = source, core_addr
        # tap: a two-way port for tools (tools/fc_profile.py, preflight.py, blackbox.py, ...)
        # while the core runs. FC traffic is copied to every tool that sent to it in the last
        # TAP_PEER_TTL_S seconds. Tools must use
        # their own sysid (254) and never send heartbeats, so they can't mask the core's GCS
        # failsafe. Off unless --tap is given.
        self.tap = None
        self.tap_peers: dict = {}  # addr -> last time it sent us something (several tools at once)
        if tap_addr:
            self.tap = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.tap.bind(tap_addr)
            self.tap.settimeout(0.2)
        # aux: a write-only inlet to the FC for edge/hbwd (the C heartbeat watchdog). It gets no
        # FC traffic back; off unless --aux is given.
        self.aux = None
        if aux_addr:
            self.aux = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.aux.bind(aux_addr)
            self.aux.settimeout(0.2)
        self.udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp.bind(("127.0.0.1", 0))
        self.udp.settimeout(0.2)
        self.src = None
        self.src_lock = threading.Lock()
        self.write_lock = threading.Lock()  # core + aux writers: never interleave two frames
        self.stop = threading.Event()
        self.bytes_up = self.bytes_down = 0
        self.last_rx = time.monotonic()

    def _ensure_source(self):
        while not self.stop.is_set():
            with self.src_lock:
                if self.src is not None:
                    return self.src
                try:
                    self.src = open_source(self.source_spec)
                    log.info("source %s open", self.source_spec)
                    return self.src
                except Exception as e:
                    log.error("open %s failed: %r; retrying", self.source_spec, e)
            time.sleep(1.0)

    def _drop_source(self, why: Exception) -> None:
        log.error("source error: %r; reopening", why)
        with self.src_lock:
            if self.src is not None:
                try:
                    self.src.close()
                except Exception:
                    pass
            self.src = None

    def watchdog(self) -> None:
        """After an FC reboot, libusb on QNX can block inside the stale read forever, so the read
        loop never sees the silence (seen on the Pi). Exit and let the supervisor restart a fresh
        relay; the edge re-runs its stream/origin setup when the FC heartbeat returns."""
        while not self.stop.is_set():
            time.sleep(0.5)
            silent = time.monotonic() - self.last_rx
            if silent > SILENT_EXIT_S:
                log.error("no FC bytes for %.1f s (USB read stuck?); exiting for a clean restart",
                          silent)
                os._exit(3)

    def fc_to_core(self) -> None:
        last_rx = time.monotonic()
        while not self.stop.is_set():
            src = self._ensure_source()
            try:
                data = src.read(0.05)
            except Exception as e:
                self._drop_source(e)
                last_rx = time.monotonic()
                continue
            if not data and time.monotonic() - last_rx > SILENT_REOPEN_S:
                # After an FC reboot / USB re-enumeration, libusb on QNX keeps timing out on the
                # stale handle instead of raising, so the relay never reopened (seen on the Pi).
                # The FC streams many messages a second: silence means a dead handle.
                self._drop_source(TimeoutError(f"no FC bytes for {SILENT_REOPEN_S:.0f} s"))
                last_rx = time.monotonic()
                continue
            if data:
                last_rx = self.last_rx = time.monotonic()
                self.udp.sendto(data, self.core_addr)
                self.bytes_down += len(data)
                if self.tap_peers:
                    now = time.monotonic()
                    for peer, seen in list(self.tap_peers.items()):
                        if now - seen > TAP_PEER_TTL_S:
                            self.tap_peers.pop(peer, None)  # tool gone quiet: stop copying
                            continue
                        try:
                            self.tap.sendto(data, peer)
                        except OSError:
                            self.tap_peers.pop(peer, None)

    def core_to_fc(self, sock: socket.socket | None = None) -> None:
        sock = sock or self.udp
        while not self.stop.is_set():
            try:
                data, peer = sock.recvfrom(4096)
            except socket.timeout:
                continue
            if sock is self.tap:
                self.tap_peers[peer] = time.monotonic()
            src = self._ensure_source()
            try:
                with self.write_lock:
                    src.write(data)
                self.bytes_up += len(data)
            except Exception as e:
                self._drop_source(e)

    def run(self, exit_on_silence: bool = True) -> None:
        if exit_on_silence:
            threading.Thread(target=self.watchdog, daemon=True, name="relay-watchdog").start()
        threading.Thread(target=self.core_to_fc, daemon=True, name="relay-up").start()
        if self.aux:
            threading.Thread(target=self.core_to_fc, args=(self.aux,), daemon=True,
                             name="relay-aux").start()
        if self.tap:
            threading.Thread(target=self.core_to_fc, args=(self.tap,), daemon=True,
                             name="relay-tap").start()
        self.fc_to_core()


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--source", default="usb", help="usb (default) or rawtcp:HOST:PORT")
    ap.add_argument("--core", default="127.0.0.1:14555", help="edge core's udpin address")
    ap.add_argument("--aux", help="extra write-only inlet to the FC, e.g. 127.0.0.1:14556 (hbwd)")
    ap.add_argument("--tap", help="two-way port for tools while the core runs, e.g. 127.0.0.1:14557")
    a = ap.parse_args()
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(name)s %(message)s")
    host, port = a.core.split(":")
    aux = None
    if a.aux:
        ah, ap_ = a.aux.rsplit(":", 1)
        aux = (ah, int(ap_))
    tap = None
    if a.tap:
        th, tp = a.tap.rsplit(":", 1)
        tap = (th, int(tp))
    r = Relay(a.source, (host, int(port)), aux, tap)
    try:
        r.run()
    except KeyboardInterrupt:
        r.stop.set()


if __name__ == "__main__":
    main()
