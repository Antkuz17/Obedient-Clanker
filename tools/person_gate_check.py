"""What the person check does and doesn't protect, run against the edge's real safety gate.

Runs the simulated drone with the real drone's settings (person detector required, 2 m per
move, slow-and-low speeds). A fake detector places people around the drone and sends their
distance over UDP the way edge/vision does: it reports only people inside the forward camera's
field of view and range. Each scenario prints PASS / FAIL. Scenarios marked GAP show a known
blind spot. They are not failures, but you should know about them before flying near people.

    .venv/bin/python -m tools.person_gate_check            # exits 0 if every check passed
    .venv/bin/python -m tools.person_gate_check --only walk_in

The sim drone stops instantly and the fake detector never misses, so this checks the gate's
logic; real stopping distance and detector accuracy are field tests.
"""
from __future__ import annotations

import argparse
import asyncio
import json
import logging
import math
import socket
import sys

from edge.config import SafetyConfig, WatchdogConfig
from edge.server import EdgeServer
from edge.sim import SimVehicle
from edge.vehicle import body_to_ned, wrap360, yaw_err
from shared.client import CommandRejected, EdgeClient

HFOV_DEG = 45.0     # forward C270, as assumed by edge/vision
RANGE_M = 4.0       # SafetyConfig.person_range_m: the detector's usable range
DETECTOR_HZ = 10.0  # edge/vision's rate

results: list[tuple[str, str, str]] = []  # (verdict, name, info)


def report(verdict: str, name: str, info: str = "") -> None:
    results.append((verdict, name, info))
    print(f"[{verdict}] {name}" + (f"  ({info})" if info else ""), flush=True)


def real_drone_cfg() -> SafetyConfig:
    """The limits `python -m edge --mav usb --gps` flies with (edge/__main__.py)."""
    c = SafetyConfig()
    c.flow_quality_min = 0
    c.require_person_detector = True
    c.alt_max, c.max_takeoff_alt, c.max_step = 1.5, 1.2, 2.0
    c.speed_xy, c.speed_z, c.speed_dn = 0.5, 1.0, 0.5
    return c


class FakeDetector:
    """People at fixed world positions (n, e). Sends what the forward camera would see."""

    def __init__(self, sim: SimVehicle, port: int):
        self.sim, self.addr = sim, ("127.0.0.1", port)
        self.people: list[tuple[float, float]] = []
        self.on = True
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def place(self, fwd: float, right: float = 0.0) -> None:
        """Put one person (replacing any others) at a spot relative to the drone right now."""
        s = self.sim.state()
        dn, de = body_to_ned(fwd, right, s.yaw)
        self.people = [(s.n + dn, s.e + de)]

    def nearest_seen(self) -> float | None:
        s = self.sim.state()
        best = None
        for n, e in self.people:
            d = math.hypot(n - s.n, e - s.e)
            bearing = wrap360(math.degrees(math.atan2(e - s.e, n - s.n)))
            if d <= RANGE_M and abs(yaw_err(bearing, s.yaw)) <= HFOV_DEG / 2:
                best = d if best is None else min(best, d)
        return best

    def nearest_any(self) -> float | None:
        s = self.sim.state()
        return min((math.hypot(n - s.n, e - s.e) for n, e in self.people), default=None)

    async def run(self) -> None:
        while True:
            if self.on:
                d = self.nearest_seen()
                msg = {"person_m": None if d is None else round(d, 2), "palm": False}
                self.sock.sendto(json.dumps(msg).encode(), self.addr)
            await asyncio.sleep(1 / DETECTOR_HZ)


async def attempt(edge: EdgeClient, op: str, timeout: float = 30, **args) -> tuple[str, str]:
    """('refused', reason) | ('ok'|'aborted'|'failed', reason)."""
    try:
        done = await edge.run(op, timeout=timeout, **args)
    except CommandRejected as e:
        return "refused", str(e)
    return done["status"], done.get("reason", "")


# --- scenarios: each starts airborne, hovering, with no people placed -------------------------
async def s_detector_off(sim, edge, det: FakeDetector):
    det.on = False
    await asyncio.sleep(1.5)  # the edge marks it dead after 1 s of silence
    st, why = await attempt(edge, "move_body", fwd=1.0)
    det.on = True
    await asyncio.sleep(0.5)
    report("PASS" if st == "refused" else "FAIL", "detector not running: forward 1 m refused",
           f"{st}: {why}")


async def s_clear(sim, edge, det):
    st, why = await attempt(edge, "move_body", fwd=1.0)
    report("PASS" if st == "ok" else "FAIL", "nobody ahead: forward 1 m flies", f"{st} {why}")


async def s_person_ahead(sim, edge, det):
    det.place(fwd=1.5)
    await asyncio.sleep(0.3)
    for fwd in (1.0, 0.3):
        st, why = await attempt(edge, "move_body", fwd=fwd)
        report("PASS" if st == "refused" else "FAIL",
               f"person 1.5 m ahead: forward {fwd} m refused", f"{st}: {why}")
    st, why = await attempt(edge, "goto", n=sim.state().n + body_to_ned(1.0, 0, sim.state().yaw)[0],
                            e=sim.state().e + body_to_ned(1.0, 0, sim.state().yaw)[1])
    report("PASS" if st != "ok" else "FAIL", "person 1.5 m ahead: goto through them refused",
           f"{st}: {why}")


async def s_person_far(sim, edge, det):
    det.place(fwd=4.0)
    await asyncio.sleep(0.3)
    st, why = await attempt(edge, "move_body", fwd=1.0)
    left = det.nearest_any()
    report("PASS" if st == "ok" else "FAIL", "person 4 m ahead: forward 1 m allowed",
           f"{st} {why}; ends {left:.2f} m from them")


async def s_walk_in(sim, edge, det):
    cid, ack = await edge.send("move_body", fwd=2.0)
    if not ack["ok"]:
        report("FAIL", "person walks in mid-move: move stopped", f"move refused: {ack['reason']}")
        return
    await asyncio.sleep(1.0)
    det.place(fwd=1.6)  # steps into the path 1.6 m ahead
    done = await edge.wait_done(cid, 30)
    await asyncio.sleep(0.5)
    gap = det.nearest_any()
    report("PASS" if done["status"] != "ok" and gap >= 1.0 else "FAIL",
           "person walks in mid-move: move stopped, >= 1 m clearance kept",
           f"{done['status']}: {done.get('reason', '')}; stopped {gap:.2f} m away")


async def s_detector_dies(sim, edge, det):
    cid, ack = await edge.send("move_body", fwd=2.0)
    if not ack["ok"]:
        report("FAIL", "detector dies mid-move: move stopped", f"refused: {ack['reason']}")
        return
    await asyncio.sleep(0.8)
    det.on = False
    done = await edge.wait_done(cid, 30)
    det.on = True
    await asyncio.sleep(0.5)
    report("PASS" if done["status"] != "ok" else "FAIL", "detector dies mid-move: move stopped",
           f"{done['status']}: {done.get('reason', '')}")


async def s_back_away(sim, edge, det):
    det.place(fwd=1.5)
    await asyncio.sleep(0.3)
    st, why = await attempt(edge, "move_body", fwd=-1.0)
    gap = det.nearest_any()
    report("PASS" if st == "ok" and gap > 1.5 else "FAIL",
           "person 1.5 m ahead: backing away 1 m allowed (turns to face the path first)",
           f"{st} {why}; now {gap:.2f} m away")


async def s_side_long(sim, edge, det):
    det.place(fwd=0.0, right=1.5)
    await asyncio.sleep(0.3)
    st, why = await attempt(edge, "move_body", right=1.0)
    gap = det.nearest_any()
    report("PASS" if st != "ok" else "FAIL",
           "person 1.5 m to the right: right 1 m stopped (turns, sees them)",
           f"{st}: {why}; {gap:.2f} m away")


async def s_side_short(sim, edge, det):
    """Moves of 0.5 m or less keep their heading (face_travel_min_m), so a person beside or
    behind the drone is never seen."""
    det.place(fwd=0.0, right=0.8)
    await asyncio.sleep(0.3)
    st, why = await attempt(edge, "move_body", right=0.4)
    gap = det.nearest_any()
    report("GAP" if st == "ok" else "PASS",
           "person 0.8 m to the right: right 0.4 m is NOT checked",
           f"{st} {why}; ends {gap:.2f} m from them")


async def s_vertical(sim, edge, det):
    """Nothing looks up or down: climbs and descents are never checked for people."""
    det.place(fwd=1.0)
    await asyncio.sleep(0.3)
    st, why = await attempt(edge, "move_body", up=0.3)
    report("GAP" if st == "ok" else "PASS",
           "person 1 m ahead: climbing / descending is NOT checked (no up/down sensor)",
           f"{st} {why}")


async def s_turn(sim, edge, det):
    det.place(fwd=0.0, right=1.0)
    await asyncio.sleep(0.3)
    st, why = await attempt(edge, "yaw", deg=90.0, relative=True)
    report("PASS" if st == "ok" else "FAIL",
           "person beside the drone: turning on the spot allowed (it doesn't move toward them)",
           f"{st} {why}")


SCENARIOS = {
    "detector_off": s_detector_off, "clear": s_clear, "person_ahead": s_person_ahead,
    "person_far": s_person_far, "walk_in": s_walk_in, "detector_dies": s_detector_dies,
    "back_away": s_back_away, "side_long": s_side_long, "side_short": s_side_short,
    "vertical": s_vertical, "turn": s_turn,
}


def free_udp_port() -> int:
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind(("127.0.0.1", 0))
    p = s.getsockname()[1]
    s.close()
    return p


async def main(a) -> int:
    cfg = real_drone_cfg()
    sim = SimVehicle(cfg, speedup=a.speedup)
    pport = free_udp_port()
    srv = EdgeServer(sim, cfg, WatchdogConfig(), host="127.0.0.1", port=0,
                     perception_port=pport, flow_port=None, log_path=None)
    await srv.start()
    edge = await EdgeClient("127.0.0.1", srv.port).connect()
    det = FakeDetector(sim, pport)
    det_task = asyncio.create_task(det.run())
    await asyncio.sleep(0.5)

    st, why = await attempt(edge, "takeoff", alt=1.0)
    if st != "ok":
        print(f"takeoff failed: {st} {why}")
        return 2
    names = a.only or list(SCENARIOS)
    for name in names:
        det.people = []
        await asyncio.sleep(0.3)
        await SCENARIOS[name](sim, edge, det)
        det.people = []
        await attempt(edge, "hover")
    print("\nNot covered here (no sensor at all): people below the drone on landing, people "
          "behind it on a short backward step.")

    await attempt(edge, "land")
    det_task.cancel()
    await edge.close()
    await srv.close()
    fails = [r for r in results if r[0] == "FAIL"]
    gaps = [r for r in results if r[0] == "GAP"]
    print(f"\n{'ALL PASS' if not fails else f'{len(fails)} FAIL'}"
          f" ({len(results) - len(fails) - len(gaps)} pass, {len(gaps)} known gaps)")
    return 1 if fails else 0


if __name__ == "__main__":
    ap = argparse.ArgumentParser(prog="tools.person_gate_check", description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--only", nargs="+", choices=list(SCENARIOS), help="run just these")
    ap.add_argument("--speedup", type=float, default=1.0,
                    help="sim speed (above 1 the fake detector lags in sim time: pessimistic)")
    ap.add_argument("-v", "--verbose", action="store_true")
    args = ap.parse_args()
    logging.basicConfig(level=logging.INFO if args.verbose else logging.WARNING)
    sys.exit(asyncio.run(main(args)))
