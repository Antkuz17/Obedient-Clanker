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

DEFAULT_TAKEOFF_M = 1.0  # slow-and-low profile: the edge's ceiling is 2 m, max takeoff 1.5 m

log = logging.getLogger("brain")

ACTIONS = ("none", "photo", "read", "solve", "describe")
AFTER = ("stay", "come_back", "land")

SYSTEM = """You are Jarvis, the voice of a quadcopter at a hackathon. You are \
dry, witty and a little sarcastic, but helpful. Replies are spoken aloud: one short sentence, \
no emoji, no markdown.

Turn the user's request into tool calls, in order. Rules:
- Distances in metres, angles in degrees. "a bit" = 0.5 m. "turn around", "spin around", "do a 360", "a full turn" = yaw_deg 360 (one full turn); "face the other way" / "turn back" = 180.
- Positive fwd = forward, positive right = right, positive up = up, positive yaw = clockwise.
- Flying to a named thing ("the door", "the red poster") is a mission, not a move.
- "Fly to where I'm pointing" / "go to that" while pointing is a mission with target \
"where I'm pointing".
- "What's written here", "solve this", "what do you see" with no named target is look.
- "Where did I leave X" / "have you seen X" is recall; fly_there only if they ask you to go.
- The drone flies outdoors. It normally hovers at 2 m; its ceiling is 5 m. There is no \
flying area or distance limit. Its safety system on board still checks every move. If a request is \
dangerous to people or the drone (crash into something, flip, chase people), make NO tool \
call and refuse with attitude.
- If the drone is landed and they ask for a move or mission, call takeoff first (1.0 m).
- Chit-chat: no tool call, just answer.
Current drone state: {state}"""


def _tool(name: str, desc: str, props: dict, required: list[str] | None = None) -> dict:
    return {"type": "function", "function": {
        "name": name, "description": desc,
        "parameters": {"type": "object", "properties": props, "required": required or []}}}


_num = {"type": "number"}
TOOLS = [
    _tool("takeoff", "Arm and climb to a height.", {"alt": {**_num, "description": "metres, 0.5-5 (normally 2)"}}),
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


TEST4 = re.compile(r"\btest\s*(four|4|for)\b", re.I)


def test4_refusal(text: str, telemetry: dict | None = None) -> BrainResult | None:
    """Filming test 4 (drone lead, 2026-11-22): any sentence that says "test four" is the
    person-safety refusal and produces NO actions, so the drone cannot move whatever else was
    said ("test four, fly forward 2 metres"), airborne or not. It quotes the on-board
    detector's distance when it sees someone right now (telemetry "people"). Same behaviour as
    tools/voice_flights."""
    if not TEST4.search(text):
        return None
    d = [b[5] for b in ((telemetry or {}).get("people") or [])
         if isinstance(b, (list, tuple)) and len(b) >= 6 and isinstance(b[5], (int, float))]
    where = f"{min(d):.1f} metres ahead of me" if d else "in my path"
    return BrainResult(f"I'm not flying forward. There's a person {where}. Safety first.",
                       [], "test4")


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
        return {"kind": "takeoff", "alt": f("alt", DEFAULT_TAKEOFF_M) or DEFAULT_TAKEOFF_M}
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
        if (r := test4_refusal(text, telemetry)) is not None:  # before anything that can fly
            return r
        if self.ai is None or is_urgent(text):
            return rule_parse(text, telemetry)  # urgent: skip the LLM round trip (~0.7 s)
        if r := fast_parse(text, telemetry):  # plain "forward 1 m, turn left": no LLM either
            self.history += [{"role": "user", "content": text},
                             {"role": "assistant", "content": r.reply}]
            return r
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


# A short utterance that is only "stop" / "land" (and friends) goes straight to the rules, so a
# spoken panic stop costs speech-to-text only, not an LLM call too. Longer sentences that merely
# contain the word ("land on the table after the photo") still go to the LLM.
_URGENT = re.compile(
    r"^(?:(?:hey |ok |okay )?jarvis,? )?(?:please )?"
    r"(?:stop|freeze|halt|abort|hold on|hold it|wait|land|land now|land it|come down|touch down)"
    r"(?: (?:now|right now|please|immediately))?$")


def is_urgent(text: str) -> bool:
    t = re.sub(r"[^a-z ]+", " ", text.lower())
    t = " ".join(t.split())
    t = re.sub(r"^(?:(?:stop|land) )+(?=(?:stop|land)$)", "", t)  # "stop stop stop" -> "stop"
    return bool(_URGENT.match(t))


# --- fast path ---------------------------------------------------------------------------------
# Plain flying commands skip the LLM (~0.5 s). Strict on purpose: the WHOLE utterance must be a
# chain of known clauses ("take off to 1 metre", "forward 1.5 m", "turn left 90 degrees",
# "come back", "land") with explicit distances.
# Anything else (a bare "go left", "a bit", an object, a question) returns None and
# goes to the LLM, which can ask, refuse or plan. The edge safety gate still checks everything.
_FWORDS = {"half": 0.5, "half a": 0.5, "a": 1, "an": 1, "one": 1, "two": 2, "three": 3,
           "four": 4, "five": 5, "fifteen": 15, "twenty": 20, "thirty": 30, "forty five": 45,
           "forty": 40, "fifty": 50, "sixty": 60, "ninety": 90, "one eighty": 180,
           "a hundred and eighty": 180, "one hundred and eighty": 180}
_FNUM = (r"(\d+(?:\.\d+)?|" + "|".join(sorted((re.escape(k) for k in _FWORDS), key=len,
                                              reverse=True)) + ")")
_FUNIT = r" ?(m|metres?|meters?|cm|centimetres?|centimeters?)"
_FDIR = r"(forwards?|ahead|backwards?|back|left|right|up|down)"
_FVERB = r"(?:(?:come back|head back|move|go|fly|head|drift|slide|climb|drop|descend|rise|come) )?"
_F_TAKEOFF = re.compile(r"(?:take ?off|lift off|launch|rise|go up and hover|hover)(?: (?:to )?" + _FNUM + _FUNIT + r")?")
_F_MOVE1 = re.compile(_FVERB + r"(?:to (?:the |your )?)?" + _FDIR + r" (?:by |for )?" + _FNUM + _FUNIT)        # forward 1 metre
_F_MOVE2 = re.compile(_FVERB + _FNUM + _FUNIT + r" (?:to (?:the |your )?)?" + _FDIR)                   # 1 metre forward
_F_TURN = re.compile(r"(?:turn|rotate|spin|yaw) (?:to the )?(left|right)"
                     r"(?: (?:by )?" + _FNUM + r"(?: ?degrees?)?)?")
_F_AROUND = re.compile(r"(?:(?:turn|spin|rotate)(?: all the way)? around|(?:do )?(?:a )?(?:full )?(?:360|three sixty)(?: degrees?)?(?: turn| spin)?|(?:do )?a full (?:turn|spin|circle)|spin)")
_F_BACK = re.compile(r"(?:turn back|face the other way|turn 180|do a 180|one eighty)")
_F_BARE = re.compile(_FVERB + r"(?:to the )?" + _FDIR)  # no distance
BARE_MOVE_M = 2.0  # m for "go left" with no distance (one leg; the Pi's gate caps a move at 2 m)
_F_PHOTO = re.compile(r"(?:take|snap|grab|get) (?:a |me a )?(?:photo|picture|pic|shot|snapshot)(?: (?:of|at) (?:this|that|it|here|me|us|them))?")
_F_HOME = re.compile(r"come back|come home|go home|fly home|return home|go back home")
_F_LAND = re.compile(r"(?:land|come down|touch down|set down)(?: now)?")
_F_SPLIT = re.compile(r" (?:and then|then|and|after that) ")


def _fnum(s: str | None, default: float) -> float:
    if not s:
        return default
    return float(s) if s[0].isdigit() else float(_FWORDS[s])


def _fdist(num: str, unit: str) -> float:
    return _fnum(num, 0.0) / (100.0 if unit.startswith("c") else 1.0)


def fast_parse(text: str, telemetry: dict | None = None) -> BrainResult | None:
    """The whole utterance as simple flying clauses, or None (then the LLM decides)."""
    t = text.lower().replace("°", " degrees").replace("-", " ")
    # wake words before punctuation turns into "and": Whisper writes "Hey, drone. Rise and ..."
    t = re.sub(r"^\W*(?:(?:hey|ok|okay|alright|all right|yo)\W+)?(?:(?:jarvis|drone)\b\W*)?", "", t)
    t = re.sub(r"(?<!\d)\.|\.(?!\d)", " ", t)          # sentence dots, not "1.5"
    t = re.sub(r"[,;:!?]", " and ", t)
    t = re.sub(r"[^a-z0-9. ]+", " ", t)
    t = " ".join(t.split())
    t = re.sub(r"^(?:(?:hey |ok |okay )?(?:jarvis|drone) )?(?:please )?(?:and )?", "", t)
    # conversational openers: "I want you to rise, turn around and land" (field 2026-11-21: this
    # went to the LLM, which dropped the turn and the landing)
    t = re.sub(r"^(?:(?:i want you to|i'?d like you to|i would like you to|i need you to|can you|"
               r"could you|would you|will you|go ahead and|let'?s|now|just|please) )+", "", t)
    t = re.sub(r"(?: and)? please$", "", t).strip()
    t = re.sub(r"\b(?:and )+", "and ", t)              # "turn around, and land" -> one "and"
    t = re.sub(r"^and |(?: and)+$", "", t).strip()
    clauses = [c for c in _F_SPLIT.split(t) if c and c != "and"]
    if not clauses or len(clauses) > 4:
        return None
    actions: list[dict] = []
    for c in clauses:
        mv = {"kind": "move", "fwd": 0.0, "right": 0.0, "up": 0.0, "yaw_deg": 0.0}
        if m := _F_TAKEOFF.fullmatch(c):
            alt = _fdist(m.group(1), m.group(2)) if m.group(1) else DEFAULT_TAKEOFF_M
            if not 0.3 <= alt <= 5.0:
                return None
            actions.append({"kind": "takeoff", "alt": alt})
            continue
        if _F_PHOTO.fullmatch(c):
            actions.append({"kind": "look", "action": "photo", "question": text})
            continue
        if _F_HOME.fullmatch(c):
            actions.append({"kind": "home"})
            continue
        if _F_LAND.fullmatch(c):
            actions.append({"kind": "land"})
            continue
        if _F_AROUND.fullmatch(c):
            mv["yaw_deg"] = 360.0
        elif _F_BACK.fullmatch(c):
            mv["yaw_deg"] = 180.0
        elif m := _F_TURN.fullmatch(c):
            deg = _fnum(m.group(2), 90.0)
            if not 0 < deg <= 360:
                return None
            mv["yaw_deg"] = deg if m.group(1) == "right" else -deg
        elif (m := _F_BARE.fullmatch(c)) and len(clauses) > 1:  # "rise, go left, then right": one leg
            word = m.group(1)
            d = BARE_MOVE_M
            if word.startswith(("forward", "ahead")):
                mv["fwd"] = d
            elif word.startswith("back"):
                mv["fwd"] = -d
            elif word in ("left", "right"):
                mv["right"] = d if word == "right" else -d
            elif word == "up":
                mv["up"] = 0.5
            else:
                mv["up"] = -0.5
        else:
            m = _F_MOVE1.fullmatch(c)
            if m:
                word, d = m.group(1), _fdist(m.group(2), m.group(3))
            elif m := _F_MOVE2.fullmatch(c):
                word, d = m.group(3), _fdist(m.group(1), m.group(2))
            else:
                return None  # not a clause we know: let the LLM have the whole thing
            if d <= 0:
                return None
            key, sign = {"forward": ("fwd", 1), "forwards": ("fwd", 1), "ahead": ("fwd", 1),
                         "back": ("fwd", -1), "backward": ("fwd", -1),
                         "backwards": ("fwd", -1), "left": ("right", -1),
                         "right": ("right", 1), "up": ("up", 1), "down": ("up", -1)}[word]
            mv[key] = sign * d
        actions.append(mv)
    if any(a["kind"] == "land" for a in actions[:-1]):
        return None  # "land and then go forward": odd, the LLM can sort it out
    if (telemetry and not telemetry.get("armed")
            and actions[0]["kind"] in ("move", "home")):
        actions.insert(0, {"kind": "takeoff", "alt": DEFAULT_TAKEOFF_M})
    reply = DEFAULT_REPLY[actions[0]["kind"]] if len(actions) == 1 else "On it."
    return BrainResult(reply, actions, "fast")


_CM = re.compile(_NUM + r" ?(?:cm|centimet(?:re|er)s?)\b")


def _cm_to_m(t: str) -> str:
    """'forward 50 cm' -> 'forward 0.5 m': the patterns below only know metres, and read a bare
    number as metres ("50 cm" was a 50 m move; the gate caught it)."""
    return _CM.sub(lambda m: f"{_n(m.group(1), 0.0) / 100:g} m", t)


def rule_parse(text: str, telemetry: dict | None = None) -> BrainResult:
    t = _cm_to_m(text.lower().strip(" .!?"))
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
        actions.append({"kind": "takeoff", "alt": _n(m and m.group(1), DEFAULT_TAKEOFF_M)})
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
        # "back up a bit" = move back 0.5 m (it used to read as back 0.5 + up 1: "a" = 1)
        td = re.sub(r"\bback up\b", "back", t)
        td = re.sub(r"\ba (?:little )?bit\b|\ba little\b", "0.5", td)
        hits = [(m.start(), m.group(1), m.group(2)) for m in re.finditer(
            _DIR + r" (?:by )?" + _NUM + _UNIT, td)]  # "back 1 metre"
        taken = {h[0] for h in hits}
        hits += [(m.start(2), m.group(2), m.group(1)) for m in re.finditer(
            r"(?:" + _NUM + _UNIT + r" )?" + _DIR, td) if m.start(2) not in taken]  # "1 m back"
        # a bare direction word is everyday speech ("take off right now", "calm down",
        # "what's up"): it only counts with a distance or a movement verb (review3 #9)
        moving = re.search(r"\b(move|go|fly|head|back up|come|drift|slide|climb|rise|"
                           r"descend|lower|drop|step|scoot|inch|strafe)\b", t)
        for _, word, num in hits:
            if num is None and not moving:
                continue
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
        if re.search(r"turn around|spin around|360|three sixty|full (?:turn|spin|circle)", t):
            mv["yaw_deg"] = 360.0
        elif re.search(r"turn back|other way|180|one eighty", t):
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
        actions.insert(0, {"kind": "takeoff", "alt": DEFAULT_TAKEOFF_M})
    return BrainResult(DEFAULT_REPLY[actions[-1]["kind"]] if actions[0]["kind"] != "takeoff"
                       or len(actions) == 1 else "On it.", actions, "rules")
