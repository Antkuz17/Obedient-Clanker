"""Two HC-SR04s (on an Arduino, tools/hcsr04/hcsr04.ino) -> edge (and the FC).

The Arduino prints "d <mm>" (down) and "f <mm>" (forward) lines over USB, -1 = no echo.
- down: {"range_m": m} to the edge's local UDP port 7002; the edge (or --fc directly) sends it to
  the flight controller as DISTANCE_SENSOR (FC params: RNGFND1_TYPE=10, RNGFND1_ORIENT=25, low
  RNGFND1_MIN). SITL showed a rangefinder is required for flow navigation and must read valid
  on the ground.
- forward: {"fwd_m": m} to the same port; the edge's safety gate refuses forward moves that
  would end within obstacle_close_m of the echo and stops a running one. Never sent to the FC.

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


def parse_line(raw: bytes) -> tuple[str, float] | None:
    """b"d 412" -> ("d", 0.412); b"f -1" -> ("f", -1.0); anything else -> None."""
    parts = raw.decode(errors="replace").split()
    if len(parts) != 2 or parts[0] not in ("d", "f"):
        return None
    try:
        mm = int(parts[1])
    except ValueError:
        return None
    return parts[0], (-1.0 if mm < 0 else mm / 1000.0)


class SpikeFilter:
    """Throws out HC-SR04 readings the drone can't physically have made.

    With the props spinning the sensor hears them: on the floor (true 0.12-0.19 m) both
    autonomous tries logged no-echo dropouts and fake readings of 0.3-1.99 m, up to 3 in 5
    (logs/blackbox, 2026-11-21). With the rangefinder as the FC's height source, one of those
    is a fake climb. Rule: a reading is accepted if it is within `slack` + `max_rate` x (time
    since the last accepted one) of it; otherwise the last good value is repeated. A real step
    (flying over a table) gets through once `confirm` readings in a row agree with each other.
    No echo is held the same way, and reported (-1) after `confirm` in a row.
    """

    def __init__(self, max_rate: float = 2.0, slack: float = 0.08, confirm: int = 5):
        self.max_rate, self.slack, self.confirm = max_rate, slack, confirm
        self.est: float | None = None
        self.t_est = 0.0
        self.cand: list[float] = []
        self.misses = 0
        self.rejected = 0  # running count, for the log

    def update(self, m: float, t: float) -> float:
        """m in metres (-1 = no echo), t in seconds. Returns the filtered value (-1 = no echo)."""
        if m < 0:
            self.misses += 1
            if self.est is not None and self.misses < self.confirm:
                self.rejected += 1
                return self.est
            self.est, self.cand = None, []
            return -1.0
        self.misses = 0
        if self.est is None or abs(m - self.est) <= self.slack + self.max_rate * (t - self.t_est):
            self.est, self.t_est, self.cand = m, t, []
            return m
        self.cand = (self.cand + [m])[-self.confirm:]
        if len(self.cand) == self.confirm and max(self.cand) - min(self.cand) <= 2 * self.slack:
            self.est, self.t_est = sorted(self.cand)[self.confirm // 2], t
            self.cand = []
            return self.est
        self.rejected += 1
        return self.est


def main() -> None:
    ap = argparse.ArgumentParser()
    src = ap.add_mutually_exclusive_group(required=True)
    src.add_argument("--usb", action="store_true", help="raw USB (QNX Pi, root)")
    src.add_argument("--serial", help="serial port, e.g. COM8 or /dev/ttyACM0")
    ap.add_argument("--edge", default="127.0.0.1:7002")
    ap.add_argument("--print", action="store_true", help="print each reading")
    ap.add_argument("--no-filter", action="store_true",
                    help="send raw readings (no prop-noise spike filter; see SpikeFilter)")
    ap.add_argument("--fc", help="also send DISTANCE_SENSOR straight to the FC through the USB "
                    "relay's aux inlet, e.g. 127.0.0.1:14556 (survives an edge core crash)")
    a = ap.parse_args()
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(name)s %(message)s")
    host, port = a.edge.rsplit(":", 1)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    fc = None
    if a.fc:
        from edge.fcdirect import RANGE_COMPONENT, FcDirect
        fc = FcDirect(a.fc, RANGE_COMPONENT)
    filt = None if a.no_filter else SpikeFilter()
    n, t0 = 0, time.monotonic()
    for raw in (lines_usb() if a.usb else lines_serial(a.serial)):
        parsed = parse_line(raw)
        if parsed is None:
            continue
        tag, m = parsed
        if tag == "f":
            sock.sendto(json.dumps({"fwd_m": m}).encode(), (host, int(port)))
            if a.print:
                print(f"forward {m:.3f} m", flush=True)
            continue
        raw_m = m
        if filt:
            m = filt.update(m, time.monotonic())
        if fc:
            fc.distance(m)
        sock.sendto(json.dumps({"range_m": m, "direct": fc is not None}).encode(),
                    (host, int(port)))
        n += 1
        if a.print:
            print(f"range {m:.3f} m" + (f"  (raw {raw_m:.3f}, spike held)" if m != raw_m else ""),
                  flush=True)
        if time.monotonic() - t0 >= 10:
            log.info("%.1f readings/s, last %.3f m, spikes held %d", n / (time.monotonic() - t0),
                     m, filt.rejected if filt else 0)
            n, t0 = 0, time.monotonic()
            if filt:
                filt.rejected = 0


if __name__ == "__main__":
    main()
