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
import socket
import threading
import time

from edge.mavlink_usb import UsbCdc

log = logging.getLogger("usb_relay")


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
    def __init__(self, source: str, core_addr: tuple[str, int]):
        self.source_spec, self.core_addr = source, core_addr
        self.udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp.bind(("127.0.0.1", 0))
        self.udp.settimeout(0.2)
        self.src = None
        self.src_lock = threading.Lock()
        self.stop = threading.Event()
        self.bytes_up = self.bytes_down = 0

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

    def fc_to_core(self) -> None:
        while not self.stop.is_set():
            src = self._ensure_source()
            try:
                data = src.read(0.05)
            except Exception as e:
                self._drop_source(e)
                continue
            if data:
                self.udp.sendto(data, self.core_addr)
                self.bytes_down += len(data)

    def core_to_fc(self) -> None:
        while not self.stop.is_set():
            try:
                data, _ = self.udp.recvfrom(4096)
            except socket.timeout:
                continue
            src = self._ensure_source()
            try:
                src.write(data)
                self.bytes_up += len(data)
            except Exception as e:
                self._drop_source(e)

    def run(self) -> None:
        threading.Thread(target=self.core_to_fc, daemon=True, name="relay-up").start()
        self.fc_to_core()


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--source", default="usb", help="usb (default) or rawtcp:HOST:PORT")
    ap.add_argument("--core", default="127.0.0.1:14555", help="edge core's udpin address")
    a = ap.parse_args()
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(name)s %(message)s")
    host, port = a.core.split(":")
    r = Relay(a.source, (host, int(port)))
    try:
        r.run()
    except KeyboardInterrupt:
        r.stop.set()


if __name__ == "__main__":
    main()
