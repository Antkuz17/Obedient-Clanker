"""Planner: brain actions -> protocol v1 primitives, one plan at a time.

    planner = Planner(edge, camera, locate=..., ai=..., memory=..., say=...)
    planner.submit(result.actions)     # cancels whatever plan is running (D7)

- `stop` and `land` pre-empt: the running plan is cancelled and the primitive goes out at once.
- A safety-gate rejection or a failed primitive ends the plan and Jarvis says why.
- mission: look for the target in the current frame, then turn in `scan_step` steps until
  `locate()` finds it (one full turn at most), fly to `approach_m` short of it (clamped inside
  the fence), face it, do the task (photo / read / solve / describe), then stay, come back or
  land.
- recall: search visual memory, say where and when, optionally fly to that pose and face the
  way the camera was facing.

The planner is never trusted either: the edge gates every primitive it sends.
"""
from __future__ import annotations

import asyncio
import logging
import math
import re
import time
from dataclasses import dataclass
from typing import Awaitable, Callable

from edge.config import SafetyConfig
from shared.client import CommandRejected, EdgeClient

from . import vision
from .camera import EdgeCamera, Frame

log = logging.getLogger("planner")

Say = Callable[[str], Awaitable[None]]
Locate = Callable[..., Awaitable[dict | None]]
MemorySearch = Callable[[str], Awaitable[list[dict]]]
OnPhoto = Callable[[Frame, str], Awaitable[None]]


class PlanAborted(Exception):
    pass


@dataclass
class PlannerConfig:
    takeoff_alt: float = 1.0
    approach_m: float = 1.0        # stop this far short of a target
    scan_step: float = 45.0        # deg per look while searching (60 deg HFOV, some overlap)
    fence_margin: float = 0.4      # stay this far inside the edge's geofence
    max_step: float = SafetyConfig.max_step


def compass(dn: float, de: float) -> str:
    names = ["north", "north-east", "east", "south-east", "south", "south-west", "west",
             "north-west"]
    return names[int(((math.degrees(math.atan2(de, dn)) + 22.5) % 360) // 45)]


def ago(t: float, now: float | None = None) -> str:
    s = max(0, int((now or time.time()) - t))
    if s < 60:
        return "a few seconds ago" if s < 15 else f"{s} seconds ago"
    m = s // 60
    return "a minute ago" if m == 1 else f"{m} minutes ago" if m < 60 else f"{m // 60} hours ago"


class Planner:
    def __init__(self, edge: EdgeClient, camera: EdgeCamera | None, *, locate: Locate,
                 ai=None, memory: MemorySearch | None = None, say: Say | None = None,
                 on_photo: OnPhoto | None = None, cfg: PlannerConfig | None = None,
                 safety: SafetyConfig | None = None):
        self.edge, self.camera = edge, camera
        self.locate, self.ai, self.memory = locate, ai, memory
        self._say = say
        self.on_photo = on_photo
        self.cfg = cfg or PlannerConfig()
        self.safety = safety or SafetyConfig()
        self._task: asyncio.Task | None = None
        self.log: list[str] = []  # human-readable trace, for the dashboard and tests

    # --- public ------------------------------------------------------------------------------
    def submit(self, actions: list[dict]) -> asyncio.Task | None:
        if not actions:
            return None
        if actions[0]["kind"] in ("stop", "land"):
            self.cancel()
            return asyncio.create_task(self._run(actions))
        self.cancel()
        self._task = asyncio.create_task(self._run(actions))
        return self._task

    def cancel(self) -> None:
        if self._task and not self._task.done():
            self._task.cancel()
        self._task = None

    @property
    def busy(self) -> bool:
        return bool(self._task and not self._task.done())

    async def wait(self) -> None:
        if self._task:
            try:
                await self._task
            except asyncio.CancelledError:
                pass

    # --- plumbing ----------------------------------------------------------------------------
    async def say(self, text: str) -> None:
        self.log.append(f"say: {text}")
        if self._say:
            try:
                await self._say(text)
            except Exception as e:  # speech must never break a plan
                log.warning("say failed: %s", e)

    async def _run(self, actions: list[dict]) -> None:
        try:
            for a in actions:
                await getattr(self, f"_do_{a['kind']}")(a)
        except asyncio.CancelledError:
            self.log.append("plan cancelled")
            raise
        except CommandRejected as e:
            await self.say(f"Safety says no: {e}.")
        except PlanAborted as e:
            await self.say(str(e))
        except Exception as e:
            log.exception("plan failed")
            await self.say(f"Something went wrong: {e}.")

    async def _prim(self, op: str, **args) -> dict:
        self.log.append(f"{op} {args}" if args else op)
        timeout = 90.0 if op in ("scan", "takeoff", "land") else 60.0
        done = await self.edge.run(op, timeout=timeout, **args)
        if done["status"] != "ok":
            raise PlanAborted(f"That {op.replace('_', ' ')} was {done['status']}"
                              + (f": {done['reason']}." if done.get("reason") else "."))
        return done

    @property
    def tel(self) -> dict:
        return self.edge.telemetry or {}

    def _airborne(self) -> bool:
        return bool(self.tel.get("armed")) and self.tel.get("alt", 0.0) > 0.2

    async def _ensure_airborne(self) -> None:
        if not self._airborne():
            await self._prim("takeoff", alt=self.cfg.takeoff_alt)

    def _clamp(self, n: float, e: float) -> tuple[float, float]:
        lim = self.safety.fence_half - self.cfg.fence_margin
        return max(-lim, min(lim, n)), max(-lim, min(lim, e))

    async def _goto(self, n: float, e: float) -> None:
        """goto in legs of at most max_step (the gate refuses longer single moves)."""
        n, e = self._clamp(n, e)
        while True:
            cn, ce = self.tel.get("n", 0.0), self.tel.get("e", 0.0)
            d = math.hypot(n - cn, e - ce)
            if d < 0.15:
                return
            f = min(1.0, (self.cfg.max_step - 0.1) / d)
            await self._prim("goto", n=round(cn + f * (n - cn), 3), e=round(ce + f * (e - ce), 3))
            if f >= 1.0:
                return

    async def _face(self, bearing: float) -> None:
        await self._prim("yaw", deg=round(bearing % 360, 1), relative=False)

    async def _frame(self) -> Frame:
        if not self.camera:
            raise PlanAborted("I have no camera feed.")
        try:
            return await self.camera.frame()
        except Exception as e:
            raise PlanAborted(f"I can't see anything: the camera feed is down ({e}).") from e

    # --- actions -----------------------------------------------------------------------------
    async def _do_takeoff(self, a: dict) -> None:
        if self._airborne():
            return
        await self._prim("takeoff", alt=a.get("alt") or self.cfg.takeoff_alt)

    async def _do_land(self, a: dict) -> None:
        await self._prim("land")

    async def _do_stop(self, a: dict) -> None:
        await self._prim("stop")

    async def _do_home(self, a: dict) -> None:
        await self._prim("return_home")

    async def _do_move(self, a: dict) -> None:
        await self._ensure_airborne()
        fwd, right, up = a.get("fwd", 0.0), a.get("right", 0.0), a.get("up", 0.0)
        if fwd or right or up:
            await self._prim("move_body", fwd=fwd, right=right, up=up)
        if a.get("yaw_deg"):
            await self._prim("yaw", deg=a["yaw_deg"], relative=True)

    async def find(self, target: str) -> tuple[dict, Frame] | None:
        """Look ahead, then turn step by step for at most one full turn."""
        K = await self.camera.K() if self.camera else {}
        steps = int(round(360 / self.cfg.scan_step))
        for i in range(steps):
            fr = await self._frame()
            hit = await self.locate(self.ai, target, fr.jpeg, fr.pose, K)
            self.log.append(f"look {i}: {'found' if hit else 'nothing'}")
            if hit:
                return hit, fr
            if i < steps - 1:
                await self._prim("yaw", deg=self.cfg.scan_step, relative=True)
        return None

    async def _do_mission(self, a: dict) -> None:
        target = re.sub(r"^(the|a|an|my|that) ", "", a["target"].strip().lower())
        await self._ensure_airborne()
        found = await self.find(target)
        if not found:
            raise PlanAborted(f"I looked all the way round and couldn't find the {target}.")
        hit, _ = found
        cn, ce = self.tel.get("n", 0.0), self.tel.get("e", 0.0)
        d = math.hypot(hit["n"] - cn, hit["e"] - ce)
        if d > self.cfg.approach_m + 0.2:
            f = (d - self.cfg.approach_m) / d
            await self._goto(cn + f * (hit["n"] - cn), ce + f * (hit["e"] - ce))
        cn, ce = self.tel.get("n", 0.0), self.tel.get("e", 0.0)
        await self._face(math.degrees(math.atan2(hit["e"] - ce, hit["n"] - cn)))
        if a.get("action", "none") != "none":
            await self._task_at(a["action"], a.get("question", ""), target)
        else:
            await self.say(f"At the {target}.")
        after = a.get("after", "stay")
        if after == "come_back":
            await self._prim("return_home")
        elif after == "land":
            await self._prim("land")

    async def _task_at(self, action: str, question: str, target: str = "") -> None:
        fr = await self._frame()
        if action == "photo":
            caption = ""
            if self.ai:
                try:
                    caption = await vision.ask(self.ai, "describe", fr.jpeg)
                except Exception as e:
                    log.warning("caption failed: %s", e)
            if self.on_photo:
                await self.on_photo(fr, caption or target)
            await self.say("Got the shot." + (f" {caption}" if caption else ""))
            return
        if not self.ai:
            raise PlanAborted("My vision model is offline, so I can't do that.")
        try:
            answer = await vision.ask(self.ai, action, fr.jpeg, question)
        except Exception as e:
            raise PlanAborted(f"My vision model didn't answer ({e}).") from e
        self.log.append(f"vision {action}: {answer}")
        await self.say(answer or "I can't make anything out.")

    async def _do_look(self, a: dict) -> None:
        await self._task_at(a.get("action", "describe"), a.get("question", ""))

    async def _do_recall(self, a: dict) -> None:
        q = a["query"]
        if not self.memory:
            raise PlanAborted("My memory is offline.")
        try:
            hits = await self.memory(q)
        except Exception as e:
            raise PlanAborted(f"I can't reach my memory ({e}).") from e
        if not hits:
            raise PlanAborted(f"I don't remember seeing any {q}.")
        h = hits[0]
        dist = math.hypot(h["n"], h["e"])
        where = ("right by where we took off" if dist < 0.5 else
                 f"about {dist:.0f} metre{'s' if round(dist) != 1 else ''} "
                 f"{compass(h['n'], h['e'])} of the takeoff spot")
        await self.say(f"I saw {q} {where}, {ago(h['t'])}.")
        if a.get("fly_there"):
            await self._ensure_airborne()
            await self._goto(h["n"], h["e"])
            await self._face(h.get("yaw", 0.0))
            await self.say("This is where I saw it.")
