"""HC-SR04 (on an Arduino, tools/hcsr04/hcsr04.ino) -> edge -> FC as MAVLink DISTANCE_SENSOR.

The Arduino prints "d <mm>" lines over USB. This process reads them and sends
{"range_m": m} (or -1 for no echo) to the edge's local UDP port 7002; the edge forwards it to
the flight controller (FC params: RNGFND1_TYPE=10, RNGFND1_ORIENT=25, low RNGFND1_MIN).
SITL showed a rangefinder is required for flow navigation and must read valid on the ground.

On the QNX Pi (no USB-serial driver: raw CDC-ACM via pyusb, so run as root):
  sudo env PYTHONPATH=$HOME/hope:$HOME/.local/lib/python3.14/site-packages \\
      python3 -m tools.rangefinder --usb
On a laptop (serial port via pyserial):
  python -m tools.rangefinder --serial COM8
"""
from __future__ import annotations

import argparse
import json
import logging
import socket
import struct
import time

log = logging.getLogger("rangefinder")
ARDUINO_VID, UNO_PID = 0x2341, 0x0043


def lines_usb():
    from edge.mavlink_usb import UsbCdc
    while True:
        try:
            cdc = UsbCdc(ARDUINO_VID, UNO_PID)
            # The Uno's USB-serial chip sends nothing until it's told the baud rate
            # (SET_LINE_CODING: 115200 8N1 on comm interface 0). Verified on the QNX Pi.
            cdc.dev.ctrl_transfer(0x21, 0x20, 0, 0, struct.pack("<IBBB", 115200, 0, 0, 8))
            time.sleep(2.0)  # the Uno resets when the port opens
            log.info("Arduino open over USB")
            buf = b""
            while True:
                buf += cdc.read(0.2)
                *done, buf = buf.split(b"\n")
                yield from done
        except Exception as e:  # unplugged or not there yet: retry
            log.warning("Arduino USB: %s; retrying", e)
            time.sleep(1.0)


def lines_serial(port: str):
    import serial  # laptop only
    with serial.Serial(port, 115200, timeout=1) as s:
        log.info("Arduino open on %s", port)
        while True:
            yield s.readline()


def main() -> None:
    ap = argparse.ArgumentParser()
    src = ap.add_mutually_exclusive_group(required=True)
    src.add_argument("--usb", action="store_true", help="raw USB (QNX Pi, root)")
    src.add_argument("--serial", help="serial port, e.g. COM8 or /dev/ttyACM0")
    ap.add_argument("--edge", default="127.0.0.1:7002")
    ap.add_argument("--print", action="store_true", help="print each reading")
    a = ap.parse_args()
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(name)s %(message)s")
    host, port = a.edge.rsplit(":", 1)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    n, t0 = 0, time.monotonic()
    for raw in (lines_usb() if a.usb else lines_serial(a.serial)):
        parts = raw.decode(errors="replace").split()
        if len(parts) != 2 or parts[0] != "d":
            continue
        try:
            mm = int(parts[1])
        except ValueError:
            continue
        m = -1.0 if mm < 0 else mm / 1000.0
        sock.sendto(json.dumps({"range_m": m}).encode(), (host, int(port)))
        n += 1
        if a.print:
            print(f"range {m:.3f} m", flush=True)
        if time.monotonic() - t0 >= 10:
            log.info("%.1f readings/s, last %.3f m", n / (time.monotonic() - t0), m)
            n, t0 = 0, time.monotonic()


if __name__ == "__main__":
    main()
