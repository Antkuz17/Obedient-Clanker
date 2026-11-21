"""MAVLink vehicle (ArduPilot Copter 4.7, GUIDED). Works against SITL and the real Matek H743.

Device examples:
  udpin:127.0.0.1:14555     QNX Pi: through tools/usb_relay.py (root) -> USB -> FC  (primary)
  usb                       QNX Pi: USB directly from this process (needs root)
  /dev/ser1                 QNX Pi 5 GPIO UART -> FC SERIAL1 (115200), if we solder later
  rawtcp:127.0.0.1:5760     SITL through the same StreamMav path the USB link uses
  tcp:127.0.0.1:5760        ArduPilot SITL

Safety notes (docs/review.md section A):
- The GCS heartbeat to the FC is sent from tick(), i.e. from the edge's safety loop, not from
  a thread of its own. If the safety loop hangs, the FC stops hearing us and its own GCS
  failsafe (FS_GCS_ENABLE=5) lands it. (A3)
- Every send goes through one lock. (A11)
- Reader errors mark the link dead and reconnect; the setup is re-run after a reconnect. (A5)
- pos_ok comes from EKF_STATUS_REPORT; stale or bad means "no position". (A6)
- `landed` is inferred from armed + altitude when EXTENDED_SYS_STATE goes stale. (A7)
- land() is retried until the FC reports LAND or disarms. (A2)
- cancel_takeoff() stops a takeoff that is still arming: no NAV_TAKEOFF, and a disarm if it
  already armed on the ground. (A1)
We never switch the FC *into* GUIDED except on takeoff: if the pilot flips to AltHold or Land,
our targets are ignored, which is what we want.
"""
from __future__ import annotations

import logging
import math
import threading
import time

from pymavlink import mavutil

from . import mavlink_usb
from .vehicle import Vehicle, VehicleState, wrap360

log = logging.getLogger("edge.mav")

COPTER_GUIDED, COPTER_LAND = 4, 9
# position + yaw only: ignore vel(8,16,32), acc(64,128,256), yaw_rate(2048)
MASK_POS_YAW = 8 | 16 | 32 | 64 | 128 | 256 | 2048
# Western University, London ON; any fixed point works, the EKF just needs an origin
ORIGIN_LAT, ORIGIN_LON, ORIGIN_ALT_MM = 430096000, -812737000, 250000

M = mavutil.mavlink
STREAMS = {  # message id -> Hz
    M.MAVLINK_MSG_ID_LOCAL_POSITION_NED: 20,
    M.MAVLINK_MSG_ID_ATTITUDE: 30,
    M.MAVLINK_MSG_ID_SYS_STATUS: 2,
    M.MAVLINK_MSG_ID_DISTANCE_SENSOR: 10,
    M.MAVLINK_MSG_ID_EXTENDED_SYS_STATE: 5,
    M.MAVLINK_MSG_ID_EKF_STATUS_REPORT: 5,
}
EKF_NEEDED = M.EKF_VELOCITY_HORIZ | M.EKF_POS_HORIZ_REL
STALE_S = 1.0          # EKF / EXTENDED_SYS_STATE older than this = unknown
FC_LINK_TIMEOUT_S = 3.0
LAND_RETRY_S = 0.5
HEARTBEAT_PERIOD_S = 0.5   # 2 Hz: 4 beats inside FS_GCS_TIMEOUT=2 (review2 #10)


class MavVehicle(Vehicle):
    def __init__(self, device: str, baud: int = 115200, speed_xy: float = 0.5,
                 speed_z: float = 0.3):
        self.device, self.baud, self.speed_xy, self.speed_z = device, baud, speed_xy, speed_z
        # GUIDED position targets and takeoff fly at these FC params, not at anything we send
        # per command, so the edge sets them on every connect and reads them back (review2 #2)
        # ArduPilot 4.7 renamed these and switched to m/s (WPNAV_SPEED cm/s -> WP_SPD m/s);
        # SITL 4.7 defaults are WP_SPD=10 m/s, WP_SPD_UP=2.5. Older firmware: the cm/s names.
        self.limit_param_sets = [
            {"WP_SPD": speed_xy, "WP_SPD_UP": speed_z, "WP_SPD_DN": speed_z},
            {"WPNAV_SPEED": speed_xy * 100, "WPNAV_SPEED_UP": speed_z * 100,
             "WPNAV_SPEED_DN": speed_z * 100},
        ]
        self.limit_params: dict[str, float] = {}  # the set the FC confirmed
        self.params: dict[str, float] = {}
        self.m = None
        self.s = VehicleState(pos_ok=False, limits_ok=False)
        self._lock = threading.Lock()      # state
        self._tx = threading.Lock()        # all MAVLink sends (A11)
        self._stop = threading.Event()
        self._abort_takeoff = threading.Event()
        self._land_req = False
        self._last_land_send = 0.0
        self._last_hb_sent = 0.0
        self._last_fc_hb = 0.0
        self._ess_t = 0.0
        self._ekf_t = 0.0
        self._ekf_flags = 0
        self._last_flow_q = 0
        self._last_flow_t = 0.0
        self._t0 = time.monotonic()
        self.reconnects = 0
        self._last_setup = 0.0

    # --- lifecycle ---------------------------------------------------------------------------
    def _open(self) -> None:
        self.m = mavlink_usb.connect(self.device) or mavutil.mavlink_connection(
            self.device, baud=self.baud, source_system=255, source_component=191,
            autoreconnect=True)

    async def start(self) -> None:
        self._open()
        threading.Thread(target=self._reader, daemon=True, name="mav-rx").start()
        # setup runs when the first FC heartbeat arrives (see _handle)

    async def close(self) -> None:
        self._stop.set()

    def _send(self, fn, *args) -> None:
        with self._tx:
            fn(*args)

    def _setup(self) -> None:
        """Run when the FC heartbeat (re)appears: stream rates and the EKF origin."""
        time.sleep(0.05)  # let _handle finish recording the heartbeat
        self._last_setup = time.monotonic()
        log.info("FC link up: sys %d comp %d", self.m.target_system, self.m.target_component)
        for msg_id, hz in STREAMS.items():
            self._cmd(M.MAV_CMD_SET_MESSAGE_INTERVAL, msg_id, int(1e6 / hz))
        self.set_origin()
        self._apply_limits()

    def _apply_limits(self) -> None:
        """Set the FC speed limits and confirm them from PARAM_VALUE echoes.

        A re-setup after a telemetry gap keeps limits_ok if they were already confirmed:
        param_set is persisted by the FC, and dropping it here made a takeoff that landed in the
        2 s re-confirm window get rejected (intermittent SITL re-takeoff failure)."""
        if not self.limit_params:
            with self._lock:
                self.s.limits_ok = False
        def confirmed(ps: dict) -> bool:
            return all(k in self.params and abs(self.params[k] - v) <= 0.01 * max(1.0, abs(v))
                       for k, v in ps.items())
        for attempt in range(3):
            for ps in self.limit_param_sets:  # unknown names are ignored (no echo) by the FC
                for name, val in ps.items():
                    self.params.pop(name, None)
                    self._send(self.m.mav.param_set_send, self.m.target_system,
                               self.m.target_component, name.encode(), float(val),
                               M.MAV_PARAM_TYPE_REAL32)
            deadline = time.monotonic() + 2.0
            while time.monotonic() < deadline:
                for ps in self.limit_param_sets:
                    if confirmed(ps):
                        self.limit_params = dict(ps)
                        with self._lock:
                            self.s.limits_ok = True
                        log.info("FC speed limits confirmed: %s", ps)
                        return
                time.sleep(0.05)
        want = " or ".join(str(ps) for ps in self.limit_param_sets)
        self.last_error = f"FC did not confirm speed limits ({want})"
        log.error(self.last_error)

    def set_origin(self) -> None:
        self._send(self.m.mav.set_gps_global_origin_send, self.m.target_system, ORIGIN_LAT,
                   ORIGIN_LON, ORIGIN_ALT_MM)

    def _cmd(self, command: int, *params: float) -> None:
        p = list(params) + [0.0] * (7 - len(params))
        self._send(self.m.mav.command_long_send, self.m.target_system,
                   self.m.target_component, command, 0, *p)

    def _set_mode(self, mode: int) -> None:
        self._send(self.m.mav.set_mode_send, self.m.target_system,
                   M.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode)

    # --- called from the edge's safety loop (50 Hz) -----------------------------------------
    def tick(self, now: float) -> None:
        if self.m is None:
            return
        if now - self._last_hb_sent >= HEARTBEAT_PERIOD_S:  # A3: only while the safety loop runs
            self._last_hb_sent = now
            try:
                self._send(self.m.mav.heartbeat_send, M.MAV_TYPE_GCS, M.MAV_AUTOPILOT_INVALID,
                           0, 0, 0)
            except Exception as e:  # link broken; the reader thread reconnects
                log.warning("heartbeat send failed: %s", e)
        with self._lock:
            self.s.connected = now - self._last_fc_hb < FC_LINK_TIMEOUT_S
            armed, mode = self.s.armed, self.s.mode
        # FC alive but telemetry silent: it rebooted faster than the link timeout (brownout,
        # USB reset) and forgot our stream rates and origin. Ask again, at most every 5 s. (A5)
        if (self.s.connected and now - self._ekf_t > 3.0 and now - self._last_setup > 5.0
                and now - self._last_fc_hb < 1.5):
            self._last_setup = now
            self.reconnects += self._ekf_t > 0
            log.warning("FC telemetry silent; re-sending stream rates and origin")
            threading.Thread(target=self._setup, daemon=True, name="mav-setup").start()
        if self._land_req:  # A2: keep asking for LAND until the FC is in it (or disarmed)
            if not armed or mode not in ("GUIDED", "LAND"):
                # disarmed, or the pilot chose another mode: the pilot always wins (review2 #1)
                self._land_req = False
            elif mode == "GUIDED" and now - self._last_land_send >= LAND_RETRY_S:
                self._last_land_send = now
                try:
                    self._set_mode(COPTER_LAND)
                except Exception as e:
                    log.warning("LAND send failed: %s", e)

    # --- receive -----------------------------------------------------------------------------
    def _reader(self) -> None:
        while not self._stop.is_set():
            try:
                msg = self.m.recv_match(blocking=True, timeout=0.5)
                if msg is not None:
                    self._handle(msg)
            except Exception as e:  # A3/A5: never freeze silently; mark dead, reconnect
                log.error("MAVLink link error: %r; reconnecting", e)
                with self._lock:
                    self.s.connected = False
                self._last_fc_hb = 0.0
                self._reconnect()

    def _reconnect(self) -> None:
        while not self._stop.is_set():
            try:
                try:
                    self.m.close()
                except Exception:
                    pass
                time.sleep(1.0)
                self._open()
                self.reconnects += 1
                return  # setup re-runs on the next FC heartbeat
            except Exception as e:
                log.error("reconnect failed: %r", e)

    def _handle(self, msg) -> None:
        t = msg.get_type()
        now = time.monotonic()
        with self._lock:
            s = self.s
            if t == "HEARTBEAT":
                if msg.type == M.MAV_TYPE_GCS or msg.get_srcSystem() != self.m.target_system:
                    return
                if now - self._last_fc_hb > FC_LINK_TIMEOUT_S:
                    # first heartbeat, or the FC came back after a gap (USB drop, FC reboot,
                    # relay restart): streams and the EKF origin must be set again (A5)
                    self.reconnects += self._last_fc_hb > 0
                    threading.Thread(target=self._setup, daemon=True, name="mav-setup").start()
                self._last_fc_hb = now
                s.connected = True
                s.armed = bool(msg.base_mode & M.MAV_MODE_FLAG_SAFETY_ARMED)
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
                self._ess_t = now
                s.landed = msg.landed_state in (M.MAV_LANDED_STATE_ON_GROUND,
                                                M.MAV_LANDED_STATE_UNDEFINED)
            elif t == "PARAM_VALUE":
                self.params[msg.param_id] = msg.param_value
            elif t == "EKF_STATUS_REPORT":
                self._ekf_t, self._ekf_flags = now, msg.flags
            elif t == "STATUSTEXT":
                log.info("FC: %s", msg.text)
                if msg.text.startswith(("PreArm", "Arm", "Takeoff", "EKF")):
                    self.last_error = msg.text

    def state(self) -> VehicleState:
        now = time.monotonic()
        with self._lock:
            s = VehicleState(**self.s.as_dict())
        if now - self._ess_t > STALE_S:  # A7: no landed_state -> infer it
            s.landed = not (s.armed and s.alt > 0.15)
        if not s.armed:
            s.landed = True
        f = self._ekf_flags
        s.pos_ok = (now - self._ekf_t <= STALE_S and (f & EKF_NEEDED) == EKF_NEEDED
                    and not f & M.EKF_CONST_POS_MODE and not f & M.EKF_UNINITIALIZED)
        # flow quality is what *we* last forwarded; stale flow = 0
        s.flow_quality = self._last_flow_q if now - self._last_flow_t < 0.5 else 0
        return s

    def pos_diag(self) -> str:
        """Why pos_ok is false, for the failsafe event: report age and EKF flags."""
        return f"EKF report age {time.monotonic() - self._ekf_t:.1f} s, flags 0x{self._ekf_flags:x}"

    # --- primitives --------------------------------------------------------------------------
    def takeoff(self, alt: float) -> None:
        self._abort_takeoff.clear()
        self._land_req = False
        threading.Thread(target=self._takeoff, args=(alt,), daemon=True,
                         name="mav-takeoff").start()

    def cancel_takeoff(self) -> None:
        self._abort_takeoff.set()

    def _takeoff(self, alt: float) -> None:
        ab = self._abort_takeoff
        if ab.is_set():
            return
        self.set_origin()
        self._set_mode(COPTER_GUIDED)
        self._cmd(M.MAV_CMD_DO_CHANGE_SPEED, 1, self.speed_xy, -1)
        if ab.is_set():
            return
        self._cmd(M.MAV_CMD_COMPONENT_ARM_DISARM, 1)
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline and not self.state().armed and not ab.is_set():
            time.sleep(0.05)
        if ab.is_set():  # A1: aborted while arming: make sure we end up disarmed on the ground
            if self.state().armed and self.state().alt < 0.2:
                self._cmd(M.MAV_CMD_COMPONENT_ARM_DISARM, 0)
            log.warning("takeoff aborted before NAV_TAKEOFF")
            return
        if not self.state().armed:
            log.error("arming failed: %s", self.last_error or "no reason from FC")
            return
        self._cmd(M.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, alt)

    def set_target(self, n: float, e: float, alt: float, yaw: float) -> None:
        self._send(self.m.mav.set_position_target_local_ned_send,
                   int((time.monotonic() - self._t0) * 1000) & 0xFFFFFFFF,
                   self.m.target_system, self.m.target_component,
                   M.MAV_FRAME_LOCAL_NED, MASK_POS_YAW,
                   n, e, -alt, 0, 0, 0, 0, 0, 0, math.radians(yaw), 0)

    def land(self) -> None:
        self.cancel_takeoff()
        self._land_req = True
        self._last_land_send = time.monotonic()
        self._set_mode(COPTER_LAND)

    def send_optical_flow(self, flow_x: float, flow_y: float, quality: int, dt_us: int,
                          ground_m: float | None) -> None:
        """flow_x/flow_y in rad/s (see docs/flight-setup.md). ground_m is ignored by ArduPilot;
        height must come from DISTANCE_SENSOR (review C)."""
        self._last_flow_q, self._last_flow_t = quality, time.monotonic()
        self._send(self.m.mav.optical_flow_send,
                   int(time.time() * 1e6), 0, 0, 0, 0.0, 0.0, max(0, min(255, quality)),
                   -1.0 if ground_m is None else float(ground_m), float(flow_x), float(flow_y))
