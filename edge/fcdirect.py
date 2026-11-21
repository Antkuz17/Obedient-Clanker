"""Sensor MAVLink straight to the flight controller, not through the edge core (review3 #8).

The flow and rangefinder readings used to reach the FC only via the Python core, so a core
crash removed position hold at the same moment the watchdog started a landing: a blind,
drifting descent. With `--fc 127.0.0.1:14556` the flow and rangefinder processes encode
OPTICAL_FLOW / DISTANCE_SENSOR themselves and send them to the USB relay's aux inlet (the same
write-only inlet edge/hbwd uses). They still tell the core about each sample (with
"direct": true) so the safety gate keeps seeing flow quality, but the core no longer forwards.

    fc = FcDirect("127.0.0.1:14556", component=193)
    fc.optical_flow(fx, fy, quality)
    fc.distance(range_m)

MAVLink 2 is required: ArduPilot reads flow_rate_x/y, which only exist in the v2 extension.
"""
from __future__ import annotations

import socket
import time

# the MAVLink 2 dialect directly, whatever pymavlink's default is in this process
from pymavlink.dialects.v20 import ardupilotmega as M

FLOW_COMPONENT, RANGE_COMPONENT = 193, 194  # core is 191, hbwd 190; all sysid 255
RANGE_MIN_CM, RANGE_MAX_CM = 2, 200          # same as edge/mav.py MavVehicle


class FcDirect:
    def __init__(self, addr: str, component: int, system: int = 255):
        host, port = addr.rsplit(":", 1)
        self.addr = (host, int(port))
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.mav = M.MAVLink(None, srcSystem=system, srcComponent=component)

    def _send(self, msg) -> None:
        try:
            self.sock.sendto(msg.pack(self.mav), self.addr)  # one frame per datagram
        except OSError:
            pass  # relay not up yet; the next sample tries again

    def optical_flow(self, flow_x: float, flow_y: float, quality: int) -> None:
        """flow_x/y in rad/s, as edge/mav.py MavVehicle.send_optical_flow."""
        self._send(self.mav.optical_flow_encode(
            int(time.time() * 1e6), 0, 0, 0, 0.0, 0.0, max(0, min(255, int(quality))), -1.0,
            float(flow_x), float(flow_y)))

    def distance(self, range_m: float) -> None:
        """Downward range in metres; < 0 = no echo (sent as out of range)."""
        cm = RANGE_MAX_CM + 1 if range_m < 0 else int(round(range_m * 100))
        self._send(self.mav.distance_sensor_encode(
            int(time.monotonic() * 1000) & 0xFFFFFFFF, RANGE_MIN_CM, RANGE_MAX_CM,
            max(0, min(65535, cm)), M.MAV_DISTANCE_SENSOR_ULTRASOUND, 0,
            M.MAV_SENSOR_ROTATION_PITCH_270, 0))
