"""MAVLink over raw USB CDC-ACM for QNX, which has no devc-serusb (see docs/qnx.md).

The USB part is lifted from tools/fc_usb_probe.py (verified on the Pi). It must run as root;
libusb segfaults as qnxuser.

`StreamMav` is a pymavlink mavfile over any (read, write) pair. `UsbCdc` supplies those for the
flight controller; tests drive the same class over a TCP socket to SITL, so only the libusb
calls themselves are hardware-only.
"""
from __future__ import annotations

import socket
import threading
from typing import Callable

from pymavlink import mavutil

ARDUPILOT_VID, MATEK_H743_PID = 0x1209, 0x5740
LIBUSB_PATH = "/usr/lib/libusb-1.0.so.0"


class StreamMav(mavutil.mavfile):
    """mavfile over read(timeout_s) -> bytes and write(bytes). Thread-safe writes."""

    def __init__(self, read: Callable[[float], bytes], write: Callable[[bytes], None],
                 address: str, source_system: int = 255, source_component: int = 191,
                 close: Callable[[], None] | None = None):
        self._read, self._write, self._close = read, write, close
        self._wlock = threading.Lock()
        self._buf = bytearray()
        super().__init__(None, address, source_system=source_system,
                         source_component=source_component)

    def recv(self, n=None) -> bytes:
        # Hand pymavlink only the bytes it asks for. It parses one message per recv(), so
        # returning a whole 512-byte chunk would leave messages queued behind a fresh 50 ms
        # read each, and telemetry would fall further and further behind.
        if not self._buf:
            self._buf += self._read(0.05)
        n = len(self._buf) if not n else n
        out = bytes(self._buf[:n])
        del self._buf[:n]
        return out

    def write(self, buf) -> None:
        with self._wlock:
            self._write(bytes(buf))

    def select(self, timeout) -> bool:
        return True  # recv() already blocks for up to 50 ms; don't let mavfile sleep 0.5 s

    def close(self) -> None:
        if self._close:
            self._close()


class UsbCdc:
    """The flight controller's CDC-ACM data endpoints via pyusb + libusb."""

    def __init__(self, vid: int = ARDUPILOT_VID, pid: int = MATEK_H743_PID,
                 libusb_path: str = LIBUSB_PATH):
        import usb.backend.libusb1 as libusb1
        import usb.core
        import usb.util

        self._usb = usb
        be = libusb1.get_backend(find_library=lambda _: libusb_path)
        dev = usb.core.find(idVendor=vid, idProduct=pid, backend=be)
        if dev is None:
            raise RuntimeError(f"USB device {vid:04x}:{pid:04x} not found (unplugged, data-less "
                               "cable, shorted board? running as root?)")
        cfg = dev.get_active_configuration()
        comm = next(i for i in cfg if i.bInterfaceClass == 0x02)
        data = next(i for i in cfg if i.bInterfaceClass == 0x0A)
        is_in = lambda e: (usb.util.endpoint_direction(e.bEndpointAddress)
                           == usb.util.ENDPOINT_IN)
        self.ep_in = next(e for e in data if is_in(e))
        self.ep_out = next(e for e in data if not is_in(e))
        for i in (comm, data):
            try:
                usb.util.claim_interface(dev, i.bInterfaceNumber)
            except Exception:  # already claimed / no kernel driver on QNX: fine
                pass
        # SET_CONTROL_LINE_STATE: DTR + RTS, or ArduPilot won't talk
        dev.ctrl_transfer(0x21, 0x22, 0x0003, comm.bInterfaceNumber, None)
        self.dev = dev

    def read(self, timeout_s: float) -> bytes:
        try:
            return bytes(self.ep_in.read(512, timeout=max(1, int(timeout_s * 1000))))
        except self._usb.core.USBTimeoutError:
            return b""

    def write(self, buf: bytes) -> None:
        self.ep_out.write(buf, timeout=200)

    def close(self) -> None:
        self._usb.util.dispose_resources(self.dev)


def connect(device: str, source_system: int = 255, source_component: int = 191):
    """Extra device strings on top of pymavlink's:
       usb                    flight controller over raw USB (QNX)
       usb:VID:PID            e.g. usb:1209:5740
       rawtcp:HOST:PORT       StreamMav over a plain socket (tests the same path as usb)
    Returns None for anything else (caller falls back to mavutil.mavlink_connection).
    """
    if device == "usb" or device.startswith("usb:"):
        parts = device.split(":")
        vid, pid = ((int(parts[1], 16), int(parts[2], 16)) if len(parts) == 3
                    else (ARDUPILOT_VID, MATEK_H743_PID))
        u = UsbCdc(vid, pid)
        return StreamMav(u.read, u.write, device, source_system, source_component, u.close)
    if device.startswith("rawtcp:"):
        _, host, port = device.split(":")
        s = socket.create_connection((host, int(port)))

        def read(timeout_s: float) -> bytes:
            s.settimeout(timeout_s)
            try:
                return s.recv(4096)
            except socket.timeout:
                return b""
        return StreamMav(read, s.sendall, device, source_system, source_component, s.close)
    return None
