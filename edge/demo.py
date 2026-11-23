"""Demo mode (`python -m edge --demo`): the whole pipeline runs, nothing flies.

For demos where flying isn't allowed (indoors, props off). Every flight command still goes
through the real safety gate with the real battery, flow, rangefinder and person detector;
then it is refused, and nothing is ever sent to the flight controller:
  - the gate refuses it -> the gate's real reason ("move of 3.0 m exceeds the 2.0 m limit");
  - the gate clears it  -> a sarcastic "I would, but..." line the station reads out.
A takeoff is judged from the real (landed) state. Once one is cleared, the edge pretends to
hover until `land`: moves are judged as if it were DEMO_ALT_M up where it is, and telemetry
shows that pretend hover (armed, DEMO_ALT_M, mode GUIDED-DEMO) so the planner doesn't put a
takeoff in front of every move. The edge's own safety logic and the C watchdog keep using the
real state. stop / hover / land keep their normal path (on the ground they touch nothing).
"""
from __future__ import annotations

import itertools
from dataclasses import replace

from .vehicle import VehicleState

DEMO_ALT_M = 1.0
DEMO_MODE = "GUIDED-DEMO"   # the dashboard treats *GUIDED* as ours (no "pilot has control")
REFUSED_OPS = ("takeoff", "move_body", "goto", "yaw", "scan", "return_home")

# {what} is a plain description: "turn right 90 degrees", "fly back 1 metre"
_LINES = itertools.cycle([
    "I would {what}. Every safety check passed. Sadly, someone decided drones don't fly "
    "indoors, so I'll be doing it in my imagination",
    "{what}? Cleared by every safety check, and I'm not allowed to anyway. We're indoors. "
    "Consider it done, spiritually",
    "I'd love to {what}, and my safety checks agree, but the house rules say I stay put. "
    "Tragic",
    "Safety checks: all green. House rules: very red. So no, I won't {what}, but I really "
    "wanted to",
])


def _metres(d: float) -> str:
    d = abs(d)
    return f"{d:g} metre" + ("" if d == 1 else "s")


def describe(op: str, args: dict) -> str:
    """'take off to 1 metre', 'fly back 2 metres and right 1 metre', 'turn left 90 degrees'."""
    if op == "takeoff":
        return f"take off to {_metres(args.get('alt', DEMO_ALT_M))}"
    if op == "move_body":
        parts = []
        for key, pos, neg in (("fwd", "forward", "back"), ("right", "right", "left"),
                              ("up", "up", "down")):
            v = float(args.get(key, 0) or 0)
            if abs(v) >= 0.01:
                parts.append(f"{pos if v > 0 else neg} {_metres(v)}")
        return "fly " + " and ".join(parts) if parts else "stay exactly where I am"
    if op == "yaw":
        deg = float(args.get("deg", 0))
        if not args.get("relative", True):
            return f"turn to face {deg:g} degrees"
        if abs(abs(deg) - 180) < 1:
            return "turn around"
        return f"turn {'right' if deg > 0 else 'left'} {abs(deg):g} degrees"
    if op == "scan":
        return "spin a full circle and look around"
    if op == "goto":
        return f"fly to {args.get('n', 0):.1f} north, {args.get('e', 0):.1f} east"
    if op == "return_home":
        return "fly back to where I took off"
    return op.replace("_", " ")


def sarcastic(op: str, args: dict) -> str:
    return next(_LINES).format(what=describe(op, args))


def pretend_hover(s: VehicleState) -> VehicleState:
    """The state moves are judged against: armed in GUIDED, DEMO_ALT_M up, where it is."""
    return replace(s, armed=True, mode="GUIDED", landed=False, alt=max(s.alt, DEMO_ALT_M))
