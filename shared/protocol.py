"""Edge protocol v1: the contract between the station (Person B) and the edge (Person A).

Frozen. Changing anything here needs both people. Pure stdlib so it runs on QNX unchanged.

Transport: TCP, edge listens on EDGE_PORT, one JSON object per line (UTF-8).
Frame: local NED from the takeoff point. Metres, degrees. Yaw 0 = north, clockwise positive.
"""
from __future__ import annotations

import json
from typing import Any

PROTO_VERSION = 1
EDGE_PORT = 7000
VIDEO_PORT = 8080

HEARTBEAT_HZ = 5
TELEMETRY_HZ = 10

# --- message types -------------------------------------------------------------------------
# station -> edge
HEARTBEAT = "heartbeat"   # {t}
CMD = "cmd"               # {id, op, args}
# edge -> station
ACK = "ack"               # {id, ok, reason}
DONE = "done"             # {id, status, reason}
TELEMETRY = "telemetry"   # see TELEMETRY_FIELDS
EVENT = "event"           # {kind, detail}

# --- ops: name -> {arg: (type, required)} ---------------------------------------------------
OPS: dict[str, dict[str, tuple[type, bool]]] = {
    "takeoff": {"alt": (float, True)},
    "move_body": {"fwd": (float, False), "right": (float, False), "up": (float, False)},
    "goto": {"n": (float, True), "e": (float, True), "alt": (float, False)},
    "yaw": {"deg": (float, True), "relative": (bool, False)},
    "scan": {"rate_dps": (float, False)},
    "hover": {},
    "stop": {},
    "return_home": {},
    "land": {},
}
ALWAYS_ALLOWED_OPS = {"stop", "land", "hover"}

# done.status
OK, ABORTED, FAILED = "ok", "aborted", "failed"

# telemetry.state
STATES = ("DISCONNECTED", "IDLE", "TAKING_OFF", "BUSY", "HOVER", "LANDING", "FAILSAFE")

TELEMETRY_FIELDS = (
    "t", "state", "armed", "mode", "n", "e", "alt", "yaw", "vn", "ve", "vd",
    "battery_v", "flow_quality", "range_m", "active_cmd",
)

# event.kind
EVENT_KINDS = ("safety_reject", "watchdog", "person_close", "palm_stop", "restart", "battery")


class ProtocolError(ValueError):
    pass


def encode(msg: dict[str, Any]) -> bytes:
    return (json.dumps(msg, separators=(",", ":")) + "\n").encode()


def decode(line: bytes | str) -> dict[str, Any]:
    try:
        msg = json.loads(line)
    except json.JSONDecodeError as e:
        raise ProtocolError(f"bad json: {e}") from None
    if not isinstance(msg, dict) or "type" not in msg:
        raise ProtocolError("message must be an object with a 'type'")
    return msg


def validate_cmd(msg: dict[str, Any]) -> tuple[str, str, dict[str, Any]]:
    """Return (id, op, normalized args) or raise ProtocolError."""
    cid = msg.get("id")
    op = msg.get("op")
    args = msg.get("args") or {}
    if not isinstance(cid, str) or not cid:
        raise ProtocolError("cmd needs a string 'id'")
    if op not in OPS:
        raise ProtocolError(f"unknown op {op!r}")
    if not isinstance(args, dict):
        raise ProtocolError("'args' must be an object")
    spec = OPS[op]
    unknown = set(args) - set(spec)
    if unknown:
        raise ProtocolError(f"unknown args for {op}: {sorted(unknown)}")
    out: dict[str, Any] = {}
    for name, (typ, required) in spec.items():
        if name not in args:
            if required:
                raise ProtocolError(f"{op} needs '{name}'")
            continue
        v = args[name]
        if typ is float:
            if isinstance(v, bool) or not isinstance(v, (int, float)):
                raise ProtocolError(f"{op}.{name} must be a number")
            v = float(v)
            if v != v or v in (float("inf"), float("-inf")):
                raise ProtocolError(f"{op}.{name} must be finite")
        elif typ is bool and not isinstance(v, bool):
            raise ProtocolError(f"{op}.{name} must be a bool")
        out[name] = v
    return cid, op, out


# --- constructors (use these instead of hand-writing dicts) ----------------------------------
def heartbeat(t: float) -> dict:
    return {"type": HEARTBEAT, "t": t}


def cmd(cid: str, op: str, **args: Any) -> dict:
    return {"type": CMD, "id": cid, "op": op, "args": args}


def ack(cid: str, ok: bool, reason: str = "") -> dict:
    return {"type": ACK, "id": cid, "ok": ok, "reason": reason}


def done(cid: str, status: str, reason: str = "") -> dict:
    return {"type": DONE, "id": cid, "status": status, "reason": reason}


def event(kind: str, detail: str = "") -> dict:
    return {"type": EVENT, "kind": kind, "detail": detail}
