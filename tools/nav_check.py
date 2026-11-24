"""Fly spoken phrases through the whole chain on the simulated drone and check where it ends up.

phrase -> brain (rules, or the real LLM with --llm) -> planner -> edge safety gate -> sim drone.
The edge uses the real drone's limits (2 m per move, 1.5 m ceiling) and the planner holds height
on the rangefinder, as on the real drone (--no-rng for the FC's height, as in station.dev). For each phrase it prints
what the brain planned, what the gate said, and the drone's position and heading after it.

    .venv/bin/python -m tools.nav_check                       # built-in suite, PASS / FAIL
    .venv/bin/python -m tools.nav_check --llm                 # same, with the Vultr LLM (.env key)
    .venv/bin/python -m tools.nav_check --say "take off" "fly in a square" "come back" "land"
    .venv/bin/python -m tools.nav_check --file my_phrases.txt  # one phrase per line, '#' comments

Positions are metres from the takeoff point: n = north, e = east; yaw in degrees (0 = north).
"""
from __future__ import annotations

import argparse
import asyncio
import logging
import math
import sys
from typing import Callable

from edge.config import SafetyConfig, WatchdogConfig
from edge.server import EdgeServer
from edge.sim import SimVehicle
from edge.video import intrinsics
from edge.vehicle import yaw_err
from shared.client import EdgeClient
from station import localize_stub
from station.brain import Brain
from station.camera import Frame, Pose
from station.planner import Planner, PlannerConfig

POS_TOL = 0.4   # m
ALT_TOL = 0.2   # m
YAW_TOL = 15.0  # deg

Pos = tuple[float, float, float, float, bool]  # n, e, alt, yaw, armed
Check = Callable[[Pos, Pos], str | None]       # (before, after) -> problem, or None


def at(n: float, e: float) -> Check:
    def f(b, a):
        d = math.hypot(a[0] - n, a[1] - e)
        return None if d <= POS_TOL else f"expected n={n:.1f} e={e:.1f}, {d:.2f} m off"
    return f


def moved(dn: float, de: float) -> Check:
    return lambda b, a: at(b[0] + dn, b[1] + de)(b, a)


def alt(h: float) -> Check:
    return lambda b, a: None if abs(a[2] - h) <= ALT_TOL else f"expected alt {h:.1f}"


def facing(deg: float) -> Check:
    return lambda b, a: (None if abs(yaw_err(deg, a[3])) <= YAW_TOL
                         else f"expected yaw {deg:.0f}")


def stayed() -> Check:
    return moved(0, 0)


def armed(want: bool) -> Check:
    return lambda b, a: None if a[4] == want else ("expected flying" if want else "expected landed")


def all_of(*cs: Check) -> Check:
    def f(b, a):
        probs = [p for c in cs if (p := c(b, a))]
        return "; ".join(probs) or None
    return f


# Each step: (phrase, check). The suite is one flight, so positions build on the last step.
SUITE: list[tuple[str, Check]] = [
    ("take off", all_of(armed(True), at(0, 0))),
    ("go north 2 metres", moved(2, 0)),
    ("face east", all_of(stayed(), facing(90))),
    ("go 1 metre east", moved(0, 1)),
    ("go back to where you were", at(2, 0)),
    ("hover for 2 seconds", stayed()),
    ("go up to 1 metre", alt(1.0)),
    ("a little lower", alt(0.5)),
    ("fly in a square", stayed()),           # a closed shape ends where it started
    ("fly a triangle", stayed()),
    ("fly out and back", stayed()),
    ("move left 1 metre", lambda b, a: None if math.hypot(a[0] - b[0], a[1] - b[1]) > 0.6
     else "did not move"),
    ("do that again", lambda b, a: None if math.hypot(a[0] - b[0], a[1] - b[1]) > 0.6
     else "did not move again"),
    ("come back", at(0, 0)),
    ("go 5 metres north", lambda b, a: None),  # over the 2 m cap: just shows what happens
    ("land", armed(False)),
]


class SimCam:
    def __init__(self, sim: SimVehicle):
        self.sim = sim

    async def K(self):
        return intrinsics()

    async def frame(self):
        s = self.sim.state()
        return Frame(b"\xff\xd8sim", Pose(s.n, s.e, s.alt, s.yaw, 0.0))


def real_drone_cfg() -> SafetyConfig:
    c = SafetyConfig()
    c.flow_quality_min = 0
    c.alt_max, c.max_takeoff_alt, c.max_step = 1.5, 1.2, 2.0
    c.speed_xy, c.speed_z, c.speed_dn = 0.5, 1.0, 0.5
    return c


def pos(sim: SimVehicle) -> Pos:
    s = sim.state()
    return (s.n, s.e, s.alt, s.yaw, s.armed)


def fmt(p: Pos) -> str:
    return (f"n={p[0]:+.2f} e={p[1]:+.2f} alt={p[2]:.2f} yaw={p[3]:3.0f}"
            f" {'flying' if p[4] else 'landed'}")


async def main(a) -> int:
    ai = None
    if a.llm:
        from station.ai import AI
        from station.config import Settings, load_dotenv
        load_dotenv()
        ai = AI(Settings())
    cfg = real_drone_cfg()
    sim = SimVehicle(cfg, speedup=a.speedup)
    srv = EdgeServer(sim, cfg, WatchdogConfig(), host="127.0.0.1", port=0,
                     perception_port=None, flow_port=None, log_path=None)
    await srv.start()
    edge = await EdgeClient("127.0.0.1", srv.port).connect()
    said: list[str] = []

    async def say(t: str) -> None:
        said.append(t)

    planner = Planner(edge, SimCam(sim), locate=localize_stub.locate, ai=ai, say=say,
                      safety=cfg, cfg=PlannerConfig(rng=not a.no_rng))
    brain = Brain(ai)
    await asyncio.sleep(0.5)  # first telemetry: the planner needs the rangefinder reading

    if a.say or a.file:
        phrases = list(a.say or [])
        if a.file:
            with open(a.file) as f:
                phrases += [ln.strip() for ln in f if ln.strip() and not ln.startswith("#")]
        steps: list[tuple[str, Check | None]] = [(p, None) for p in phrases]
    else:
        steps = list(SUITE)

    fails = 0
    for phrase, check in steps:
        before = pos(sim)
        r = await brain.think(phrase, edge.telemetry)
        said.clear()
        log_from = len(planner.log)
        planner.submit(r.actions, r.reply)
        try:
            await asyncio.wait_for(planner.wait(), a.timeout)
        except asyncio.TimeoutError:
            planner.cancel()
            said.append(f"(timed out after {a.timeout:.0f} s)")
        if any(act["kind"] == "land" for act in r.actions):  # land's plan ends before touchdown
            for _ in range(int(30 / 0.1)):
                if not sim.state().armed:
                    break
                await asyncio.sleep(0.1)
        await asyncio.sleep(0.8 / a.speedup)  # let it settle
        after = pos(sim)
        problem = check(before, after) if check else None
        verdict = "" if check is None else ("[FAIL] " if problem else "[PASS] ")
        fails += bool(problem)
        print(f"\n{verdict}\"{phrase}\"  [{r.source}]")
        print(f"   plan:  {r.actions or '(nothing)'}")
        for line in planner.log[log_from:]:
            print(f"   sent:  {line}")
        for t in said or [r.reply]:
            print(f"   says:  {t}")
        print(f"   now:   {fmt(after)}" + (f"   <- {problem}" if problem else ""))

    if pos(sim)[4]:
        planner.submit([{"kind": "land"}])
        await asyncio.wait_for(planner.wait(), 30)
    await edge.close()
    await srv.close()
    if ai:
        await ai.aclose()
    if any(c for _, c in steps):
        print(f"\n{'ALL PASS' if not fails else f'{fails} FAIL'} ({len(steps)} phrases)")
    return 1 if fails else 0


if __name__ == "__main__":
    ap = argparse.ArgumentParser(prog="tools.nav_check", description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--say", nargs="+", metavar="PHRASE", help="fly these phrases, in order")
    ap.add_argument("--file", help="phrases to fly, one per line")
    ap.add_argument("--llm", action="store_true", help="use the Vultr LLM (key in .env)")
    ap.add_argument("--no-rng", action="store_true",
                    help="hold height on the FC's estimate instead of the rangefinder")
    ap.add_argument("--speedup", type=float, default=4.0, help="sim speed (default 4x)")
    ap.add_argument("--timeout", type=float, default=60.0, help="s per phrase")
    ap.add_argument("-v", "--verbose", action="store_true")
    args = ap.parse_args()
    logging.basicConfig(level=logging.INFO if args.verbose else logging.WARNING)
    sys.exit(asyncio.run(main(args)))
