"""MAVLink vehicle (ArduPilot Copter 4.7, GUIDED). Works against SITL and the real Matek H743.

Device examples:
  /dev/ser1                 QNX Pi 5 GPIO UART -> FC SERIAL1 (115200)
  /dev/serusb1              QNX USB fallback
  tcp:127.0.0.1:5760        ArduPilot SITL
  udpin:0.0.0.0:14550       SITL / MAVProxy output

We heartbeat as a GCS (sysid 255), so ArduPilot's own GCS failsafe (FS_GCS_ENABLE) lands the
drone if this whole process dies: a second watchdog that lives inside the flight controller.
We never switch the FC *into* GUIDED except on takeoff: if the pilot flips SwA to Stabilize or
Land, our targets are ignored, which is exactly what we want.
"""
from __future__ import annotations

import logging
import math
import threading
import time

from pymavlink import mavutil

from .vehicle import Vehicle, VehicleState, wrap360

log = logging.getLogger("edge.mav")

COPTER_GUIDED, COPTER_LAND = 4, 9
# position + yaw only: ignore vel(8,16,32), acc(64,128,256), yaw_rate(2048)
MASK_POS_YAW = 8 | 16 | 32 | 64 | 128 | 256 | 2048
# Western University, London ON; any fixed point works, the EKF just needs an origin
ORIGIN_LAT, ORIGIN_LON, ORIGIN_ALT_MM = 430096000, -812737000, 250000

STREAMS = {  # message id -> Hz
    mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED: 20,
    mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE: 30,
    mavutil.mavlink.MAVLINK_MSG_ID_SYS_STATUS: 2,
    mavutil.mavlink.MAVLINK_MSG_ID_DISTANCE_SENSOR: 10,
    mavutil.mavlink.MAVLINK_MSG_ID_EXTENDED_SYS_STATE: 5,
}


class MavVehicle(Vehicle):
    def __init__(self, device: str, baud: int = 115200, speed_xy: float = 0.5):
        self.device, self.baud, self.speed_xy = device, baud, speed_xy
        self.m = None
        self.s = VehicleState()
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._last_fc_hb = 0.0
        self._last_flow_q = 0
        self._last_flow_t = 0.0
        self._t0 = time.monotonic()

    # --- lifecycle ---------------------------------------------------------------------------
    async def start(self) -> None:
        self.m = mavutil.mavlink_connection(self.device, baud=self.baud, source_system=255,
                                            source_component=191, autoreconnect=True)
        threading.Thread(target=self._reader, daemon=True, name="mav-rx").start()
        threading.Thread(target=self._heartbeat, daemon=True, name="mav-hb").start()
        threading.Thread(target=self._setup, daemon=True, name="mav-setup").start()

    async def close(self) -> None:
        self._stop.set()

    def _setup(self) -> None:
        if not self.m.wait_heartbeat(timeout=30):
            log.error("no heartbeat from flight controller on %s", self.device)
            return
        log.info("FC heartbeat: sys %d comp %d", self.m.target_system, self.m.target_component)
        for msg_id, hz in STREAMS.items():
            self._cmd(mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, msg_id, int(1e6 / hz))
        self.set_origin()

    def set_origin(self) -> None:
        self.m.mav.set_gps_global_origin_send(self.m.target_system, ORIGIN_LAT, ORIGIN_LON,
                                              ORIGIN_ALT_MM)

    def _cmd(self, command: int, *params: float) -> None:
        p = list(params) + [0.0] * (7 - len(params))
        self.m.mav.command_long_send(self.m.target_system, self.m.target_component, command, 0,
                                     *p)

    def _heartbeat(self) -> None:
        while not self._stop.is_set():
            self.m.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                                      mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
            with self._lock:
                self.s.connected = time.monotonic() - self._last_fc_hb < 3.0
            time.sleep(1.0)

    def _reader(self) -> None:
        mav = mavutil.mavlink
        while not self._stop.is_set():
            msg = self.m.recv_match(blocking=True, timeout=0.5)
            if msg is None:
                continue
            t = msg.get_type()
            with self._lock:
                s = self.s
                if t == "HEARTBEAT":
                    if msg.type == mav.MAV_TYPE_GCS or msg.get_srcSystem() != self.m.target_system:
                        continue
                    self._last_fc_hb = time.monotonic()
                    s.connected = True
                    s.armed = bool(msg.base_mode & mav.MAV_MODE_FLAG_SAFETY_ARMED)
                    s.mode = mavutil.mode_string_v10(msg)
                elif t == "LOCAL_POSITION_NED":
                    s.n, s.e, s.alt = msg.x, msg.y, -msg.z
                    s.vn, s.ve, s.vd = msg.vx, msg.vy, msg.vz
                elif t == "ATTITUDE":
                    s.yaw = wrap360(math.degrees(msg.yaw))
                elif t == "SYS_STATUS":
                    if msg.voltage_battery not in (0, 65535):
                        s.battery_v = msg.voltage_battery / 1000.0
                elif t == "DISTANCE_SENSOR":
                    s.range_m = msg.current_distance / 100.0
                elif t == "EXTENDED_SYS_STATE":
                    s.landed = msg.landed_state in (mav.MAV_LANDED_STATE_ON_GROUND,
                                                    mav.MAV_LANDED_STATE_UNDEFINED)
                elif t == "STATUSTEXT":
                    log.info("FC: %s", msg.text)
                    if msg.text.startswith(("PreArm", "Arm", "Takeoff")):
                        self.last_error = msg.text
                # flow quality is what *we* last sent; stale flow = 0
                s.flow_quality = (self._last_flow_q
                                  if time.monotonic() - self._last_flow_t < 0.5 else 0)

    def state(self) -> VehicleState:
        with self._lock:
            s = VehicleState(**self.s.as_dict())
        if not s.armed:
            s.landed = True
        return s

    # --- primitives --------------------------------------------------------------------------
    def takeoff(self, alt: float) -> None:
        threading.Thread(target=self._takeoff, args=(alt,), daemon=True).start()

    def _takeoff(self, alt: float) -> None:
        self.set_origin()
        self.m.set_mode(COPTER_GUIDED)
        self._cmd(mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED, 1, self.speed_xy, -1)
        self._cmd(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 1)
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline and not self.state().armed:
            time.sleep(0.1)
        if not self.state().armed:
            log.error("arming failed: %s", self.last_error or "no reason from FC")
            return
        self._cmd(mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, alt)

    def set_target(self, n: float, e: float, alt: float, yaw: float) -> None:
        self.m.mav.set_position_target_local_ned_send(
            int((time.monotonic() - self._t0) * 1000) & 0xFFFFFFFF,
            self.m.target_system, self.m.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED, MASK_POS_YAW,
            n, e, -alt, 0, 0, 0, 0, 0, 0, math.radians(yaw), 0)

    def land(self) -> None:
        self.m.set_mode(COPTER_LAND)

    def send_optical_flow(self, flow_x: float, flow_y: float, quality: int, dt_us: int,
                          ground_m: float | None) -> None:
        """flow_x/flow_y in rad/s (see docs/flight-setup.md)."""
        self._last_flow_q, self._last_flow_t = quality, time.monotonic()
        self.m.mav.optical_flow_send(
            int(time.time() * 1e6), 0, 0, 0, 0.0, 0.0, max(0, min(255, quality)),
            -1.0 if ground_m is None else float(ground_m), float(flow_x), float(flow_y))
