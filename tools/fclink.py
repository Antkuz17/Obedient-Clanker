"""Shared helpers for the flight-test tools: talk to the FC through the USB relay's tap port.

The relay copies FC traffic to the tap (`--tap 127.0.0.1:14557`) while the edge core keeps
running. Tools use sysid 254 and never send heartbeats, so the FC's GCS failsafe still watches
the core alone. Without a relay (SITL), pass a pymavlink address such as tcp:127.0.0.1:5761.
"""
from __future__ import annotations

import time

from pymavlink import mavutil

TAP = "udpout:127.0.0.1:14557"
M = mavutil.mavlink

EKF_FLAGS = {1: "att", 2: "vel_h", 4: "vel_v", 8: "pos_h_rel", 16: "pos_h_abs",
             32: "pos_v_abs", 64: "pos_v_agl", 128: "const_pos", 256: "pred_h_rel",
             512: "pred_h_abs"}


def connect(addr: str = TAP, timeout: float = 10.0):
    m = mavutil.mavlink_connection(addr, source_system=254, source_component=200)
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        # any packet registers us with the relay tap; the FC ignores a request for msg 0
        m.mav.command_long_send(1, 1, M.MAV_CMD_REQUEST_MESSAGE, 0, 0, 0, 0, 0, 0, 0, 0)
        hb = m.recv_match(type="HEARTBEAT", blocking=True, timeout=1)
        if hb and hb.get_srcSystem() == 1 and hb.type != M.MAV_TYPE_GCS:
            m.target_system, m.target_component = 1, 1
            return m
    raise TimeoutError(f"no flight controller heartbeat via {addr} (relay running with --tap?)")


def is_armed(addr: str = TAP, timeout: float = 5.0) -> bool | None:
    """True / False from the FC's own heartbeat; None when the FC can't be reached (relay or
    FC down)."""
    try:
        m = connect(addr, timeout=timeout)
    except Exception:
        return None
    t0 = time.monotonic()
    while time.monotonic() - t0 < 3.0:
        hb = m.recv_match(type="HEARTBEAT", blocking=True, timeout=1)
        if hb and hb.get_srcSystem() == 1 and hb.type != M.MAV_TYPE_GCS:
            return bool(hb.base_mode & M.MAV_MODE_FLAG_SAFETY_ARMED)
    return None


def get_param(m, name: str, tries: int = 6) -> float | None:
    for _ in range(tries):
        m.mav.param_request_read_send(1, 1, name.encode(), -1)
        t0 = time.monotonic()
        while time.monotonic() - t0 < 1.0:
            p = m.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.3)
            if p and p.param_id.strip("\x00") == name:
                return float(p.param_value)
    return None


def set_param(m, name: str, value: float, tries: int = 5) -> float | None:
    got = None
    for _ in range(tries):
        m.mav.param_set_send(1, 1, name.encode(), float(value), M.MAV_PARAM_TYPE_REAL32)
        time.sleep(0.3)
        got = get_param(m, name)
        if got is not None and abs(got - value) < 1e-3 * max(1.0, abs(value)):
            return got
    return got


def stream(m, msg_id: int, hz: float) -> None:
    m.mav.command_long_send(1, 1, M.MAV_CMD_SET_MESSAGE_INTERVAL, 0, msg_id,
                            int(1e6 / hz) if hz > 0 else -1, 0, 0, 0, 0, 0)


def ekf_flags(flags: int) -> list[str]:
    return [n for b, n in EKF_FLAGS.items() if flags & b]
