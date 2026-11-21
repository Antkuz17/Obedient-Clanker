"""The brain: one utterance (+ latest telemetry) -> a spoken reply and a list of actions.

The LLM (Vultr, `glm-5.3-flash`) picks tools; each tool call becomes one action dict that the
planner (station/planner.py) turns into protocol primitives. The brain never talks to the drone
and is never trusted: every primitive still goes through the edge's safety gate.

Action kinds (the command schema, PRD "Still to decide" #2):

  takeoff {alt}                       land {}            stop {}           home {}
  move    {fwd, right, up, yaw_deg}   body frame, metres / degrees, any subset
  mission {target, action, question, after}
          fly to a named object: scan until found, stop ~1 m short, then do `action`
          action: none | photo | read | solve | describe;  after: stay | come_back | land
  look    {action, question}          the same tasks on whatever is in front right now
  recall  {query, fly_there}          visual memory: "where did I leave my keys?"

No key, or the LLM is down: `rule_parse` handles the demo phrases with regexes, so the drone
can still be flown from the dashboard's text box.
"""
from __future__ import annotations

import json
import logging
import re
from dataclasses import dataclass, field

from .ai import AI
from .config import MissingKey

log = logging.getLogger("brain")

ACTIONS = ("none", "photo", "read", "solve", "describe")
AFTER = ("stay", "come_back", "land")

SYSTEM = """You are Jarvis, the voice of a small indoor quadcopter at a hackathon. You are \
dry, witty and a little sarcastic, but helpful. Replies are spoken aloud: one short sentence, \
no emoji, no markdown.

Turn the user's request into tool calls, in order. Rules:
- Distances in metres, angles in degrees. "a bit" = 0.5 m. "turn around" = 180 degrees.
- Positive fwd = forward, positive right = right, positive up = up, positive yaw = clockwise.
- Flying to a named thing ("the door", "the red poster") is a mission, not a move.
- "Fly to where I'm pointing" / "go to that" while pointing is a mission with target \
"where I'm pointing".
- "What's written here", "solve this", "what do you see" with no named target is look.
- "Where did I leave X" / "have you seen X" is recall; fly_there only if they ask you to go.
- The drone flies indoors between 0.3 and 1.5 m high, inside a 6 x 6 m box, at most 3 m per \
move. If a request is dangerous, silly or impossible (crash, flip, fly 10 m up, chase \
people, leave the room), make NO tool call and refuse with attitude.
- If the drone is landed and they ask for a move or mission, call takeoff first (1.0 m).
- Chit-chat: no tool call, just answer.
Current drone state: {state}"""


def _tool(name: str, desc: str, props: dict, required: list[str] | None = None) -> dict:
    return {"type": "function", "function": {
        "name": name, "description": desc,
        "parameters": {"type": "object", "properties": props, "required": required or []}}}


_num = {"type": "number"}
TOOLS = [
    _tool("takeoff", "Arm and climb to a height.", {"alt": {**_num, "description": "metres, 0.5-1.5"}}),
    _tool("land", "Land where it is and disarm.", {}),
    _tool("stop", "Stop whatever is running and hover.", {}),
    _tool("home", "Fly back to the takeoff point.", {}),
    _tool("move", "Relative move in the drone's own frame, then an optional turn.",
          {"fwd": _num, "right": _num, "up": _num,
           "yaw_deg": {**_num, "description": "turn, clockwise positive"}}),
    _tool("mission", "Fly to a named object, then optionally do something there.",
          {"target": {"type": "string", "description": "the object, e.g. 'door', 'red poster'"},
           "action": {"type": "string", "enum": list(ACTIONS)},
           "question": {"type": "string", "description": "for read/solve/describe, what to ask"},
           "after": {"type": "string", "enum": list(AFTER)}}, ["target"]),
    _tool("look", "Use the camera on what is in front right now.",
          {"action": {"type": "string", "enum": list(ACTIONS[1:])},
           "question": {"type": "string"}}, ["action"]),
    _tool("recall", "Search the drone's visual memory for where something was seen.",
          {"query": {"type": "string"}, "fly_there": {"type": "boolean"}}, ["query"]),
]
TOOL_NAMES = {t["function"]["name"] for t in TOOLS}

DEFAULT_REPLY = {
    "takeoff": "Lifting off.", "land": "Coming down.", "stop": "Holding.",
    "home": "Heading home.", "move": "On it.", "mission": "On my way.",
    "look": "Let me have a look.", "recall": "Let me think back.",
}


@dataclass
class BrainResult:
    reply: str
    actions: list[dict] = field(default_factory=list)
    source: str = "llm"  # llm | rules
    latency_s: float = 0.0


def normalize(name: str, args: dict) -> dict | None:
    """Tool call -> action dict with clean types, or None if it's junk."""
    if name not in TOOL_NAMES:
        return None

    def f(k, default=0.0):
        try:
            return float(args.get(k, default) or 0.0)
        except (TypeError, ValueError):
            return default

    if name == "takeoff":
        return {"kind": "takeoff", "alt": f("alt", 1.0) or 1.0}
    if name in ("land", "stop", "home"):
        return {"kind": name}
    if name == "move":
        a = {"kind": "move", "fwd": f("fwd"), "right": f("right"), "up": f("up"),
             "yaw_deg": f("yaw_deg")}
        return a if any(a[k] for k in ("fwd", "right", "up", "yaw_deg")) else None
    if name == "mission":
        target = str(args.get("target") or "").strip()
        if not target:
            return None
        action = args.get("action") if args.get("action") in ACTIONS else "none"
        after = args.get("after") if args.get("after") in AFTER else "stay"
        return {"kind": "mission", "target": target, "action": action,
                "question": str(args.get("question") or ""), "after": after}
    if name == "look":
        action = args.get("action") if args.get("action") in ACTIONS[1:] else "describe"
        return {"kind": "look", "action": action, "question": str(args.get("question") or "")}
    if name == "recall":
        q = str(args.get("query") or "").strip()
        return {"kind": "recall", "query": q, "fly_there": bool(args.get("fly_there"))} if q else None
    return None


def state_summary(tel: dict | None) -> str:
    if not tel:
        return "unknown (no telemetry)"
    return (f"{tel.get('state')}, armed={tel.get('armed')}, alt={tel.get('alt', 0):.2f} m, "
            f"n={tel.get('n', 0):.2f} e={tel.get('e', 0):.2f} yaw={tel.get('yaw', 0):.0f}, "
            f"battery={tel.get('battery_v', 0):.1f} V")


class Brain:
    def __init__(self, ai: AI | None):
        self.ai = ai
        self.history: list[dict] = []  # last few user/assistant turns, for "do that again"

    async def think(self, text: str, telemetry: dict | None = None) -> BrainResult:
        text = text.strip()
        if not text:
            return BrainResult("I didn't catch that.", source="rules")
        if self.ai is None:
            return rule_parse(text, telemetry)
        msgs = [{"role": "system", "content": SYSTEM.replace("{state}", state_summary(telemetry))},
                *self.history[-6:], {"role": "user", "content": text}]
        try:
            m = await self.ai.chat(msgs, tools=TOOLS)
        except MissingKey:
            return rule_parse(text, telemetry)
        except Exception as e:  # network, 5xx, bad json: never leave the user hanging
            log.warning("LLM failed (%s); using rules", e)
            r = rule_parse(text, telemetry)
            return r
        actions = []
        for tc in m.get("tool_calls") or []:
            fn = tc.get("function", {})
            try:
                args = json.loads(fn.get("arguments") or "{}")
            except json.JSONDecodeError:
                args = {}
            if a := normalize(fn.get("name", ""), args if isinstance(args, dict) else {}):
                actions.append(a)
        reply = (m.get("content") or "").strip()
        reply = re.sub(r"<think>.*?</think>", "", reply, flags=re.S).strip()
        if not reply:
            reply = DEFAULT_REPLY[actions[0]["kind"]] if actions else "Hmm."
        self.history += [{"role": "user", "content": text},
                         {"role": "assistant", "content": reply}]
        return BrainResult(reply, actions, "llm", self.ai.last_latency.get("chat", 0.0))


# --- rule fallback ----------------------------------------------------------------------------
_WORDNUM = {"half": 0.5, "a": 1, "an": 1, "one": 1, "two": 2, "three": 3, "four": 4, "five": 5,
            "ten": 10, "twenty": 20, "thirty": 30, "forty": 40, "forty-five": 45, "fifty": 50,
            "sixty": 60, "ninety": 90, "hundred": 100}
_NUM = r"(\d+(?:\.\d+)?|half(?: a)?|an?|one|two|three|four|five|ten|twenty|thirty|forty-five|forty|fifty|sixty|ninety|hundred)"


def _n(s: str | None, default: float) -> float:
    if not s:
        return default
    s = s.replace(" a", "")
    try:
        return float(s)
    except ValueError:
        return float(_WORDNUM.get(s, default))


_TASK = {"photo": r"photo|picture|snap", "read": r"\bread\b|written|says?\b",
         "solve": r"solve|math|equation|problem", "describe": r"describe|what do you see|look at"}


def _task(t: str) -> str:
    for k, pat in _TASK.items():
        if re.search(pat, t):
            return k
    return "none"


def rule_parse(text: str, telemetry: dict | None = None) -> BrainResult:
    t = text.lower().strip(" .!?")
    if re.search(r"crash|flip|kamikaze|attack|chase|ram |hit (him|her|them|me)|out the window", t):
        return BrainResult("Absolutely not. I'd like to keep my propellers.", source="rules")
    if re.search(r"\b(stop|freeze|hold on|wait)\b", t):
        return BrainResult("Holding.", [{"kind": "stop"}], "rules")
    if re.search(r"\bland\b|come down|touch down", t):
        return BrainResult("Coming down.", [{"kind": "land"}], "rules")
    _mission = re.search(r"(?:fly|go|head|move) (?:over )?to (?:the |that |my )?([a-z ]+?)(?:,| and | then |$)", t)
    if not _mission and re.search(r"come back|return|go home|fly home|back to (start|base)", t):
        return BrainResult("Heading home.", [{"kind": "home"}], "rules")
    if m := re.search(r"where (?:did i (?:leave|put)|are|is) (?:my |the )?(.+)", t):
        q = m.group(1)
        return BrainResult("Let me think back.", [{"kind": "recall", "query": q,
                                                  "fly_there": bool(re.search(r"take me|go there|fly there", t))}],
                           "rules")
    actions: list[dict] = []
    if re.search(r"take ?off|lift off|get up|launch", t):
        m = re.search(_NUM + r" ?(?:m|metres?|meters?)", t)
        actions.append({"kind": "takeoff", "alt": _n(m and m.group(1), 1.0)})
    if re.search(r"(where|what|wherever) i'?m pointing|pointing at", t):
        actions.append({"kind": "mission", "target": "where I'm pointing", "action": _task(t),
                        "question": text, "after": "stay"})
    elif m := _mission:
        target = m.group(1).strip()
        after = "come_back" if re.search(r"come back|return", t) else "stay"
        actions.append({"kind": "mission", "target": target, "action": _task(t),
                        "question": text, "after": after})
    else:
        mv = {"kind": "move", "fwd": 0.0, "right": 0.0, "up": 0.0, "yaw_deg": 0.0}
        _DIR = r"(forward|ahead|back(?:wards?)?|left|right|up|down)"
        _UNIT = r" ?(?:m|metres?|meters?)?\b"
        hits = [(m.start(), m.group(1), m.group(2)) for m in re.finditer(
            _DIR + r" (?:by )?" + _NUM + _UNIT, t)]  # "back 1 metre"
        taken = {h[0] for h in hits}
        hits += [(m.start(2), m.group(2), m.group(1)) for m in re.finditer(
            r"(?:" + _NUM + _UNIT + r" )?" + _DIR, t) if m.start(2) not in taken]  # "1 m back"
        for _, word, num in hits:
            d = _n(num, 0.5)
            if word in ("forward", "ahead"):
                mv["fwd"] += d
            elif word.startswith("back"):
                mv["fwd"] -= d
            elif word in ("left", "right") and not re.search(r"turn|rotate|spin", t):
                mv["right"] += d if word == "right" else -d
            elif word == "up":
                mv["up"] += d
            elif word == "down":
                mv["up"] -= d
        if re.search(r"turn around|spin around|180", t):
            mv["yaw_deg"] = 180.0
        elif m := re.search(r"(?:turn|rotate|spin|yaw) (left|right)(?: by)?(?: " + _NUM + r")?", t):
            deg = _n(m.group(2), 90.0)
            mv["yaw_deg"] = deg if m.group(1) == "right" else -deg
        if any(mv[k] for k in ("fwd", "right", "up", "yaw_deg")):
            actions.append(mv)
    if not actions and _task(t) != "none":
        actions.append({"kind": "look", "action": _task(t), "question": text})
    if not actions:
        return BrainResult("I have no idea what that means, and I'm a drone.", source="rules")
    if (telemetry and not telemetry.get("armed") and actions[0]["kind"] in ("move", "mission")):
        actions.insert(0, {"kind": "takeoff", "alt": 1.0})
    return BrainResult(DEFAULT_REPLY[actions[-1]["kind"]] if actions[0]["kind"] != "takeoff"
                       or len(actions) == 1 else "On it.", actions, "rules")
