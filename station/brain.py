"""The brain: one utterance (+ latest telemetry) -> a spoken reply and a list of actions.

The LLM (Vultr, `glm-5.3-flash`) picks tools; each tool call becomes one action dict that the
planner (station/planner.py) turns into protocol primitives. The brain never talks to the drone
and is never trusted: every primitive still goes through the edge's safety gate.

Action kinds (the command schema, PRD "Still to decide" #2):

  takeoff {alt}                       land {}            stop {}           home {}
  move    {fwd, right, up, yaw_deg}   body frame, metres / degrees, any subset
  compass {north, east, up, face}     world frame: "go north 3 m", "face east" (face = heading
                                      0-360, or None to keep the heading)
  height  {alt}                       absolute height above the ground: "go up to 1 m"
  wait    {seconds}                   hover in place: "hover for 5 seconds, then land"
  pattern {shape, size, side}         square | triangle | circle | out_and_back, flown as moves
  retrace {}                          back to where the last move started: "undo that"
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
import math
import re
from dataclasses import dataclass, field

from .ai import AI
from .config import MissingKey

DEFAULT_TAKEOFF_M = 1.0  # slow-and-low profile: the edge's ceiling is 2 m, max takeoff 1.5 m

log = logging.getLogger("brain")

ACTIONS = ("none", "photo", "read", "solve", "describe")
AFTER = ("stay", "come_back", "land")
SHAPES = ("square", "triangle", "circle", "out_and_back")
HEADINGS = {"north": 0.0, "north east": 45.0, "northeast": 45.0, "east": 90.0,
            "south east": 135.0, "southeast": 135.0, "south": 180.0, "south west": 225.0,
            "southwest": 225.0, "west": 270.0, "north west": 315.0, "northwest": 315.0}
HIGHER_M = 0.5    # "go higher" / "a bit lower"
MAX_WAIT_S = 60.0  # longest "hover for N seconds"

SYSTEM = """You are Jarvis, the voice of a quadcopter. You are \
dry, witty and a little sarcastic, but helpful. Replies are spoken aloud: one short sentence, \
no emoji, no markdown.

Turn the user's request into tool calls, in order. Make ALL the calls for the whole request \
in this one reply: the drone runs them one after another, and you never get a second turn. Rules:
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
- Compass directions ("go north 3 metres", "head south-west", "face east") are compass, \
never move. North and east are positive; south and west negative. "Go / head / fly <direction>" \
with no distance flies 2 m that way; only "face" / "turn to" just turns.
- A height ("go up to 1 metre", "fly at 1.5 metres", "go to 2 metres high") is height. \
"Higher" / "lower" with no number = move up / down 0.5. Never call takeoff while flying.
- "Hover / wait for N seconds" is wait. A square, triangle or circle is pattern (size = side, \
or diameter for a circle; default 2 m); "go out N metres and come back" is pattern out_and_back.
- retrace returns to where this request started if it has moved, else to where the last \
move started. After moves in the same request, "come back", "return", "go back to where you \
started", "the way you came" is retrace: "fly 2 m forward, 3 m right, then go back to where \
you started" = move, move, retrace. Alone, "go back to where you were" / "undo that" is \
retrace, but a bare "go back" / "back up" means move backwards (fwd -1), and "come home" / \
"return to the start" is home.
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
    _tool("compass", "Move by compass direction and/or turn to face a compass heading.",
          {"north": {**_num, "description": "metres north (negative = south)"},
           "east": {**_num, "description": "metres east (negative = west)"},
           "up": _num,
           "face": {**_num, "description": "heading to face: 0 north, 90 east, 180 south, 270 west"}}),
    _tool("height", "Climb or descend to an absolute height above the ground.",
          {"alt": {**_num, "description": "metres above the ground"}}, ["alt"]),
    _tool("wait", "Hover in place for some seconds before the next step.",
          {"seconds": _num}, ["seconds"]),
    _tool("pattern", "Fly a shape from where it is, ending back at the start.",
          {"shape": {"type": "string", "enum": list(SHAPES)},
           "size": {**_num, "description": "metres: side, circle diameter, or out-and-back distance"},
           "side": {"type": "string", "enum": ["left", "right"],
                    "description": "which way it turns (default right)"}}, ["shape"]),
    _tool("retrace", "Go back to where the drone was before its last move.", {}),
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
    "compass": "On it.", "height": "Changing height.", "wait": "Hovering.",
    "pattern": "Here goes.", "retrace": "Going back.",
}
# plans that fly: "do that again" repeats the last one of these
FLIGHT_KINDS = ("takeoff", "move", "home", "compass", "height", "wait", "pattern", "retrace",
                "mission")


@dataclass
class BrainResult:
    reply: str
    actions: list[dict] = field(default_factory=list)
    source: str = "llm"  # llm | rules
    latency_s: float = 0.0


# "test for" is how Whisper often hears "test four"; not when it is ordinary speech ("a test
# for me", "the test for today")
TEST4 = re.compile(r"\btest\s*(?:four|4|for(?!\s+(?:me|you|us|the|a|an|my|your|this|that|it|"
                   r"today|now)\b))\b", re.I)
# a test-4 sentence that also asks to come down or stop must still do that (review: "that's
# enough of test four, land" and "stop moving forward" used to send nothing)
_T4_LAND = re.compile(r"\b(?:land|come down|touch down|get down)\b", re.I)
_T4_STOP = re.compile(r"\b(?:stop|halt|abort|freeze|hold on|wait)\b", re.I)
# With test 4 armed off camera (Brain.test4_armed, POST /test4), a plain forward request is the
# refusal too, so nobody has to say "test four" on camera (cloud lead, 2026-11-22).
FORWARD = re.compile(r"\b(forwards?|straight ahead|towards?|closer|"
                     r"(?:move|go|fly|head|keep going) ahead(?! and))\b", re.I)


def test4_refusal(text: str, telemetry: dict | None = None,
                  armed: bool = False) -> BrainResult | None:
    """Filming test 4 (drone lead, 2026-11-22): any sentence that says "test four" is the
    person-safety refusal and produces NO actions, so the drone cannot move whatever else was
    said ("test four, fly forward 2 metres"), airborne or not. It quotes the on-board
    detector's distance when it sees someone right now (telemetry "people"), and says so when
    it sees nobody: it never claims a person the detector didn't find. A land or stop in the
    same sentence is still sent, and nothing else (landing beats stop). Same behaviour as
    tools/voice_flights."""
    if not (TEST4.search(text) or (armed and FORWARD.search(text) and not is_urgent(text))):
        return None
    if _T4_LAND.search(text):
        return BrainResult("Coming down.", [{"kind": "land"}], "test4")
    if _T4_STOP.search(text):
        return BrainResult("Holding.", [{"kind": "stop"}], "test4")
    d = [b[5] for b in ((telemetry or {}).get("people") or [])
         if isinstance(b, (list, tuple)) and len(b) >= 6 and isinstance(b[5], (int, float))]
    if d:
        return BrainResult(f"I'm not flying forward. There's a person {min(d):.1f} metres ahead "
                           "of me. Safety first.", [], "test4")
    return BrainResult("I'm not flying forward. Forward flight is locked for this test, and I "
                       "don't see anyone ahead right now.", [], "test4")


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
    if name == "compass":
        face = args.get("face")
        if isinstance(face, str):  # "north" instead of 0
            face = HEADINGS.get(face.strip().lower().replace("-", " "))
        try:
            face = None if face is None or face == "" else float(face) % 360.0
        except (TypeError, ValueError):
            face = None
        a = {"kind": "compass", "north": f("north"), "east": f("east"), "up": f("up"),
             "face": face}
        return a if any(a[k] for k in ("north", "east", "up")) or face is not None else None
    if name == "height":
        alt = f("alt", -1.0)
        return {"kind": "height", "alt": alt} if alt > 0 else None
    if name == "wait":
        s = f("seconds")
        return {"kind": "wait", "seconds": min(s, MAX_WAIT_S)} if s > 0 else None
    if name == "pattern":
        shape = str(args.get("shape") or "").strip().lower().replace(" ", "_").replace("-", "_")
        if shape not in SHAPES:
            return None
        size = f("size") or None
        if size is not None and size <= 0:
            return None
        side = "left" if str(args.get("side") or "").lower() == "left" else "right"
        return {"kind": "pattern", "shape": shape, "size": size, "side": side}
    if name == "retrace":
        return {"kind": "retrace"}
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


def did(actions: list[dict]) -> str:
    """What a plan was, for the LLM's history: '[did: move fwd=2 | wait seconds=5]'. Without it
    the LLM only saw its own spoken lines and couldn't tell what "the same but left" was."""
    if not actions:
        return ""
    parts = []
    for a in actions:
        args = " ".join(f"{k}={v:g}" if isinstance(v, float) else f"{k}={v}"
                        for k, v in a.items() if k != "kind" and v not in (0, 0.0, None, ""))
        parts.append(f"{a['kind']} {args}".strip())
    return " [did: " + " | ".join(parts) + "]"


AGAIN = re.compile(r"(?:(?:do|try) (?:that|it|this|the same(?: thing)?) )?(?:again|one more time|"
                   r"once more)|repeat(?: that| it| the last (?:one|move|command))?|same again|"
                   r"(?:do )?the same(?: thing)? again|encore")


class Brain:
    def __init__(self, ai: AI | None):
        self.ai = ai
        self.history: list[dict] = []  # last few user/assistant turns, for "do that again"
        self.test4_armed = False  # filming test 4: plain "move forward" is refused, nothing sent
        self.last_flight: list[dict] = []  # the last plan that flew: "do that again"

    def _remember(self, text: str, r: BrainResult) -> BrainResult:
        self.history += [{"role": "user", "content": text},
                         {"role": "assistant", "content": r.reply + did(r.actions)}]
        acts = [a for a in r.actions if a["kind"] != "takeoff"]  # "again" while flying: no takeoff
        if acts and all(a["kind"] in FLIGHT_KINDS for a in acts):
            self.last_flight = [dict(a) for a in acts]
        return r

    async def think(self, text: str, telemetry: dict | None = None) -> BrainResult:
        text = text.strip()
        if not text:
            return BrainResult("I didn't catch that.", source="rules")
        if (r := test4_refusal(text, telemetry, self.test4_armed)) is not None:  # before anything that can fly
            return r
        if is_urgent(text):
            return rule_parse(text, telemetry)  # urgent: skip the LLM round trip (~0.7 s)
        if AGAIN.fullmatch(_clean(text)):
            if not self.last_flight:
                return BrainResult("Do what again? I haven't flown anything yet.", source="fast")
            acts = [dict(a) for a in self.last_flight]
            if telemetry and not telemetry.get("armed"):
                acts.insert(0, {"kind": "takeoff", "alt": DEFAULT_TAKEOFF_M})
            return self._remember(text, BrainResult("Once more, then.", acts, "fast"))
        if self.ai is None:  # no key: the strict fast path, then the looser rules
            return self._remember(text, fast_parse(text, telemetry) or rule_parse(text, telemetry))
        if r := fast_parse(text, telemetry):  # plain "forward 1 m, turn left": no LLM either
            return self._remember(text, r)
        msgs = [{"role": "system", "content": SYSTEM.replace("{state}", state_summary(telemetry))},
                *self.history[-6:], {"role": "user", "content": text}]
        for attempt in (1, 2):
            try:
                m = await self.ai.chat(msgs, tools=TOOLS)
            except MissingKey:
                return rule_parse(text, telemetry)
            except Exception as e:  # network, 5xx, bad json: never leave the user hanging
                log.warning("LLM failed (%s); using rules", e)
                r = rule_parse(text, telemetry)
                return r
            actions, reply = self._read_llm(m, telemetry)
            if reply or actions:
                break
            # the reasoning model sometimes returns nothing at all (1 in 2 for "take a little
            # trip west and come back the way you came", 2026-11-23); a second ask usually works
            log.info("LLM returned nothing (attempt %d)", attempt)
        if not reply and not actions:
            # the LLM said nothing and planned nothing ("can you fly?" got "Hmm." in the field):
            # the rules may still know it; if not, say what it can do instead of a shrug
            r = rule_parse(text, telemetry)
            if r.actions:
                return r
            reply = ("I didn't get that. Try 'rise', 'go left two metres', 'do a 360', "
                     "'take a photo' or 'land'.")
        if not reply:
            reply = DEFAULT_REPLY[actions[0]["kind"]]
        return self._remember(text, BrainResult(reply, actions, "llm",
                                                self.ai.last_latency.get("chat", 0.0)))

    @staticmethod
    def _read_llm(m: dict, telemetry: dict | None) -> tuple[list[dict], str]:
        actions = []
        for tc in m.get("tool_calls") or []:
            fn = tc.get("function", {})
            try:
                args = json.loads(fn.get("arguments") or "{}")
            except json.JSONDecodeError:
                args = {}
            args = args if isinstance(args, dict) else {}
            if a := normalize(fn.get("name", ""), args):
                # "go higher" while flying came back as takeoff alt=3, which the planner ignores
                # when airborne: a takeoff to a height while flying means go to that height
                if (a["kind"] == "takeoff" and args.get("alt") and telemetry
                        and telemetry.get("armed")):
                    a = {"kind": "height", "alt": a["alt"]}
                actions.append(a)
        reply = (m.get("content") or "").strip()
        reply = re.sub(r"<think>.*?</think>", "", reply, flags=re.S).strip()
        return actions, reply


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


# --- what Whisper writes -> one plain form -------------------------------------------------------
# Field tests (2026-11-21/22) kept failing on the *wording*, not the meaning: "Hey, drone. Rise.",
# "go five meters to the left", "Um, take off", "land the drone", "one point five meters",
# "go forward to meters" (Whisper's "to" for "two"). Every parser reads `_clean(text)`, so a new
# phrasing is fixed once here instead of once per regex.
_ONES = {"zero": 0, "one": 1, "two": 2, "three": 3, "four": 4, "five": 5, "six": 6, "seven": 7,
         "eight": 8, "nine": 9, "ten": 10, "eleven": 11, "twelve": 12, "thirteen": 13,
         "fourteen": 14, "fifteen": 15, "sixteen": 16, "seventeen": 17, "eighteen": 18,
         "nineteen": 19}
_TENS = {"twenty": 20, "thirty": 30, "forty": 40, "fifty": 50, "sixty": 60, "seventy": 70,
         "eighty": 80, "ninety": 90}
_DIGIT = {**{k: v for k, v in _ONES.items() if v < 10}, "oh": 0}
_UNIT_WORD = r"(?:m|meters?|metres?|cm|centimet(?:er|re)s?|feet|foot|ft|degrees?)\b"


def _num_at(w: list[str], i: int) -> tuple[float, int] | None:
    """A spoken number starting at w[i] -> (value, words used), or None.
    'six', 'forty five', 'a hundred and eighty', 'one eighty', 'two and a half', 'one point five'."""
    def small(j):  # 0-99
        if j < len(w) and w[j] in _TENS:
            v, j = _TENS[w[j]], j + 1
            if j < len(w) and w[j] in _ONES and 0 < _ONES[w[j]] < 10:
                v, j = v + _ONES[w[j]], j + 1
            return v, j
        if j < len(w) and w[j] in _ONES:
            return _ONES[w[j]], j + 1
        if j < len(w) and re.fullmatch(r"\d+", w[j]):
            return int(w[j]), j + 1
        return None
    j = i
    if j < len(w) and w[j] in ("a", "one") and j + 1 < len(w) and w[j + 1] == "hundred":
        v, j = 100, j + 2
        if j + 1 < len(w) and w[j] == "and" and (s := small(j + 1)):
            v, j = v + s[0], s[1]
        elif s := small(j):
            v, j = v + s[0], s[1]
    elif s := small(j):
        v, j = s
        # "one eighty", "three sixty", "two seventy": headings said as two numbers
        if 0 < v < 10 and j < len(w) and w[j] in _TENS and w[i] in _ONES:
            v, j = v * 100 + _TENS[w[j]], j + 1
    else:
        return None
    if j + 1 < len(w) and w[j] == "point" and w[j + 1] in _DIGIT:  # "one point five"
        frac, j = "", j + 1
        while j < len(w) and w[j] in _DIGIT:
            frac, j = frac + str(_DIGIT[w[j]]), j + 1
        v = float(f"{v}.{frac}")
    if w[j:j + 3] == ["and", "a", "half"]:  # "two and a half"
        v, j = v + 0.5, j + 3
    return v, j - i


def _numbers(t: str) -> str:
    """Spoken numbers -> digits: 'go six metres left' -> 'go 6 metres left'. Vague amounts get a
    number too ('a couple of metres' = 2, 'a few' = 3, 'half a metre' = 0.5, 'a metre' = 1), but
    only when a unit follows, so 'do a 360' and 'a bit' keep their meaning."""
    t = re.sub(r"\b(?:a )?couple(?: of)? (?=" + _UNIT_WORD + ")", "2 ", t)
    t = re.sub(r"\ba few (?=" + _UNIT_WORD + ")", "3 ", t)
    t = re.sub(r"(?<!and )(?<!and a )\b(?:a half|half an?|half) (?=" + _UNIT_WORD + ")", "0.5 ", t)
    t = re.sub(r"\b(?:to|too) (?=(?:meters?|metres?|feet)\b)", "2 ", t)    # Whisper: "two"
    t = re.sub(r"\b(?:for) (?=(?:meters?|metres?|feet)\b)", "4 ", t)       # Whisper: "four"
    w, out, i = t.split(), [], 0
    while i < len(w):
        if w[i] in ("a", "an") and i + 1 < len(w) and re.fullmatch(_UNIT_WORD, w[i + 1]):
            out.append("1")
            i += 1
            continue
        n = _num_at(w, i)
        if n and not (w[i] == "one" and n[1] == 1 and not (
                i + 1 < len(w) and re.fullmatch(_UNIT_WORD, w[i + 1]))):
            out.append(f"{n[0]:g}")  # a lone "one" stays a word unless a unit follows
            i += n[1]
            continue
        out.append(w[i])
        i += 1
    t = " ".join(out)
    t = re.sub(r"(\d+(?:\.\d+)?) ?(?:feet|foot|ft)\b",
               lambda m: f"{round(float(m.group(1)) * 0.3048, 2):g} m", t)
    t = re.sub(r"(\d+(?:\.\d+)?) ?(?:meters?|metres?|m)\b", r"\1 m", t)
    t = re.sub(r"(\d+(?:\.\d+)?) ?(?:cm|centimet(?:er|re)s?)\b", r"\1 cm", t)
    t = re.sub(r"(\d+(?:\.\d+)?) ?degrees?\b", r"\1 degrees", t)
    return t


_OPENERS = re.compile(
    r"^(?:(?:hey|hi|ok|okay|alright|all right|yo|so|well|now|thanks|thank you|um|uh|"
    r"i want you to|id like you to|i would like you to|i need you to|can you|could you|"
    r"would you|will you|go ahead and|lets|just|please|and|then) )+")
_CLOSERS = re.compile(r"(?: (?:please|thanks|thank you|for me|real quick))+$")


def _clean(text: str) -> str:
    """Lower case, no wake words / filler / politeness, sentences and commas joined by 'and',
    numbers as digits with unit 'm' / 'cm' / 'degrees'. 'Hey, drone. Um, go one point five
    meters to the left, please.' -> 'go 1.5 m to the left'."""
    t = text.lower().replace("°", " degrees").replace("’", "'").replace("'", "")
    t = re.sub(r"(?<=\d)-(?=\d)", " ", t).replace("-", " ")
    # the drone's names, anywhere: "hey drone", "okay jarvis", "land the drone", "drone, stop"
    t = re.sub(r"\b(?:(?:hey|hi|ok|okay|yo|alright)\W+)?(?:the |my |our )?(?:drone|jarvis)\b",
               " ", t)
    t = re.sub(r"\b(?:um+|uh+|uhm|erm|hmm+)\b", " ", t)
    t = re.sub(r"(?<!\d)\.|\.(?!\d)", " and ", t)          # sentence dots, not "1.5"
    t = re.sub(r"[,;:!?]", " and ", t)
    t = re.sub(r"[^a-z0-9. ]+", " ", t)
    t = " ".join(t.split())
    t = _numbers(t)
    t = re.sub(r"\b(?:and )+", "and ", t)                  # "turn around, and land" -> one "and"
    t = re.sub(r"(?: and)+$", "", t)
    t = _OPENERS.sub("", t)
    t = re.sub(r"(?: and)+$", "", _CLOSERS.sub("", t))     # "take off, please" -> "take off"
    return t.strip()


# A short utterance that is only "stop" / "land" (and friends) goes straight to the rules, so a
# spoken panic stop costs speech-to-text only, not an LLM call too. Longer sentences that merely
# contain the word ("land on the table after the photo") still go to the LLM.
_URGENT = re.compile(
    r"^(?:stop|freeze|halt|abort|hold on|hold it|wait|land|land now|land it|come down|touch down|"
    r"get down|stop it|stop now|emergency stop|stop stop|thats enough|cut it out|kill it)"
    r"(?: (?:now|right now|immediately|right away))?$")


def is_urgent(text: str) -> bool:
    t = _clean(text)
    # "stop, stop, stop" / "stop and land" -> the last word decides (landing beats stop)
    t = re.sub(r"^(?:(?:stop|land|abort|halt)(?: and)? )+(?=(?:stop|land|abort|halt)$)", "", t)
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
_F_TAKEOFF = re.compile(r"(?:take ?off|lift off|launch|rise(?: up)?|get up|go up and hover)(?: (?:to )?" + _FNUM + _FUNIT + r")?")
_F_MOVE1 = re.compile(_FVERB + r"(?:to (?:the |your )?)?" + _FDIR + r" (?:by |for )?" + _FNUM + _FUNIT)        # forward 1 metre
_F_MOVE2 = re.compile(_FVERB + _FNUM + _FUNIT + r" (?:to (?:the |your )?)?" + _FDIR)                   # 1 metre forward
_F_TURN = re.compile(r"(?:turn|rotate|spin|yaw) (?:to (?:the |your )?)?(left|right)"
                     r"(?: (?:by |for )?" + _FNUM + r"(?: ?degrees?)?)?")
_F_TURN2 = re.compile(r"(?:turn|rotate|spin|yaw) (?:by )?" + _FNUM +                     # turn 90 degrees to the left
                      r"(?: ?degrees?)? (?:to (?:the |your )?)?(left|right)")
_F_AROUND = re.compile(r"(?:(?:turn|spin|rotate)(?: all the way)? around|(?:do )?(?:a )?(?:full )?(?:360|three sixty)(?: degrees?)?(?: turn| spin)?|(?:do )?a full (?:turn|spin|circle|rotation)|spin|(?:turn|rotate|spin) 360(?: degrees)?)")
_F_BACK = re.compile(r"(?:turn back|face the other way|turn 180|do a 180|one eighty|(?:turn|rotate|spin) 180(?: degrees)?)")
_F_BARE = re.compile(_FVERB + r"(?:to the )?" + _FDIR)  # no distance
BARE_MOVE_M = 2.0  # m for "go left" with no distance (one leg; the Pi's gate caps a move at 2 m)
BARE_UPDOWN_M = 0.5  # m for a bare "go up" / "go down"
_F_PHOTO = re.compile(r"(?:(?:take|snap|grab|get) (?:a |me a |us a )?(?:photo|picture|pic|shot|snapshot|selfie)|photo|picture|say cheese)(?: (?:of|at|with) (?:this|that|it|here|me|us|them|everyone))?")
_F_HOME = re.compile(r"come back(?: here| to me| home| over here| to the start)?|come here|come over here|come to me|come home|go home|fly home|head home|fly back home|return(?: home| to me| to the start| to base)?|go back home")
_F_LAND = re.compile(r"(?:land|come down|touch down|set down|set it down)(?: now| here| it| right here| please)?")
_F_HOLD = re.compile(r"hover(?: there| here| in place)?|stay(?: there| here| put| still| where you are)?|hold(?: position| still| there| it there)|wait there")
_F_FLY = re.compile(r"fly|start flying|get flying|go fly")
# compass ("go north 3 m", "5 m to the south east", "face west"); _clean has made "north-east"
# "north east"
_F_CDIR = r"(north(?: ?east| ?west)?|south(?: ?east| ?west)?|east|west)(?:wards?)?"
_F_CMOVE1 = re.compile(_FVERB + r"(?:to the |towards? the |due )?" + _F_CDIR + r" (?:by |for )?"
                       + _FNUM + _FUNIT)
_F_CMOVE2 = re.compile(_FVERB + _FNUM + _FUNIT + r" (?:to the |towards? the |due )?" + _F_CDIR)
_F_CBARE = re.compile(_FVERB + r"(?:to the |towards? the |due )?" + _F_CDIR)
_F_FACE = re.compile(r"(?:face|turn to face|turn|point|look|rotate|yaw)(?: (?:to|towards))?"
                     r"(?: the)? " + _F_CDIR)
# an absolute height ("go up to 1 metre", "fly at 1.5 m", "descend to 80 cm", "hover at 1 m")
_F_HEIGHT = re.compile(r"(?:(?:go|fly|climb|rise|descend|drop|come|get|move|hover|stay|hold|"
                       r"level off)(?: (?:up|down|back))? )?(?:to|at) (?:a height of |an "
                       r"altitude of )?" + _FNUM + _FUNIT + r"(?: (?:high|up|off the ground|"
                       r"above the ground|altitude|height|in the air))?")
_F_HIGHER = re.compile(r"(?:(?:go|fly|get|move|come|climb|drop|rise|sink) )?(?:a (?:little )?bit "
                       r"|a little |slightly |(?:" + _FNUM + _FUNIT + r") )?(higher|lower)"
                       r"(?: (?:a (?:little )?bit|a little|slightly))?")
_F_WAIT = re.compile(r"(?:hover|wait|stay|hold(?: position)?|pause|sit tight|stay put|hang)"
                     r"(?: (?:there|here|in place|still|right there))?(?: for)? "
                     r"(\d+(?:\.\d+)?|one|a few|a couple of|a couple) (?:seconds?|secs?|s)")
_F_SHAPE = re.compile(r"(?:(?:fly|do|make|draw|trace|go|move)(?: (?:in|around in|around|round "
                      r"in))? )?(?:a |an )?(?:" + _FNUM + _FUNIT + r" )?(square|box|triangle|"
                      r"circle)(?: (?:of |with sides of |about |with a diameter of |thats )?"
                      + _FNUM + _FUNIT + r"(?: (?:wide|across|big|sides?|on each side|each "
                      r"side|diameter))?)?(?: (?:to|on|towards) (?:the |your )?(left|right))?")
_F_CIRCLE2 = re.compile(r"circle(?: around)?(?: (?:to|on) (?:the |your )?(left|right))?")
_F_RETRACE = re.compile(
    r"(?:go |come |fly |head |get )?back (?:to )?where you (?:were|came from|just were)|"
    r"(?:come|go|fly|head) back the (?:same )?way you came|retrace(?: your steps)?|"
    r"undo(?: that| it| the last (?:one|move))?|take that back|reverse that|"
    r"(?:go |come )?back to your last (?:spot|position|place)")
# after a move in the same sentence, these mean "back to where you started this": "go forward
# 2 m and come back" used to fly to the takeoff point, "forward 3 m and back" flew back 2 m
_F_BACKAGAIN = re.compile(r"(?:come|go|fly|head) back|back(?: again)?")
_SHAPE_NAMES = {"square": "square", "box": "square", "triangle": "triangle", "circle": "circle"}
_WAIT_WORDS = {"one": 1.0, "a few": 3.0, "a couple of": 2.0, "a couple": 2.0}


def _compass_move(word: str, d: float) -> dict:
    b = math.radians(HEADINGS[word])
    return {"kind": "compass", "north": round(d * math.cos(b), 3),
            "east": round(d * math.sin(b), 3), "up": 0.0, "face": None}
_F_SPLIT = re.compile(r" (?:and then|then|and|after that) ")


def _fnum(s: str | None, default: float) -> float:
    if not s:
        return default
    return float(s) if s[0].isdigit() else float(_FWORDS[s])


def _fdist(num: str, unit: str) -> float:
    return _fnum(num, 0.0) / (100.0 if unit.startswith("c") else 1.0)


def fast_parse(text: str, telemetry: dict | None = None) -> BrainResult | None:
    """The whole utterance as simple flying clauses, or None (then the LLM decides)."""
    # _clean: wake words, filler, "I want you to ..." (field 2026-11-21: that went to the LLM,
    # which dropped the turn and the landing), sentence dots as "and", spoken numbers as digits
    t = _clean(text)
    clauses = [_OPENERS.sub("", _CLOSERS.sub("", c) + " ").strip() for c in _F_SPLIT.split(t)]
    clauses = [c for c in clauses if c and c != "and"]
    if not clauses or len(clauses) > 5:
        return None
    landed = bool(telemetry) and not telemetry.get("armed")
    airborne = bool(telemetry) and bool(telemetry.get("armed"))
    actions: list[dict] = []
    for c in clauses:
        mv = {"kind": "move", "fwd": 0.0, "right": 0.0, "up": 0.0, "yaw_deg": 0.0}
        if m := _F_TAKEOFF.fullmatch(c):
            alt = _fdist(m.group(1), m.group(2)) if m.group(1) else DEFAULT_TAKEOFF_M
            if not 0.3 <= alt <= 5.0:
                return None
            if airborne and m.group(1) and not re.match(r"take ?off|lift off|launch", c):
                # "rise to 1 metre" while flying: go to that height
                actions.append({"kind": "height", "alt": alt})
            elif not any(a["kind"] == "takeoff" for a in actions):
                actions.append({"kind": "takeoff", "alt": alt})
            continue
        if m := _F_HEIGHT.fullmatch(c):
            alt = _fdist(m.group(1), m.group(2))
            if not 0.3 <= alt <= 5.0:
                return None  # too low or too high to be a hover height: the LLM can say so
            actions.append({"kind": "takeoff", "alt": alt} if landed and not actions
                           else {"kind": "height", "alt": alt})
            continue
        if m := _F_HIGHER.fullmatch(c):
            d = _fdist(m.group(1), m.group(2)) if m.group(1) else HIGHER_M
            if d <= 0 or (not airborne and not actions):
                return None  # "go higher" on the ground: the LLM can say it's landed
            mv["up"] = d if m.group(3) == "higher" else -d
            actions.append(mv)
            continue
        if m := _F_WAIT.fullmatch(c):
            n = m.group(1)
            s = _WAIT_WORDS[n] if n in _WAIT_WORDS else float(n)
            if not 0 < s <= MAX_WAIT_S:
                return None
            actions.append({"kind": "wait", "seconds": s})
            continue
        moved = any(a["kind"] in ("move", "compass", "height") and any(
            a.get(k) for k in ("fwd", "right", "up", "north", "east", "alt")) for a in actions)
        if _F_RETRACE.fullmatch(c) or (moved and _F_BACKAGAIN.fullmatch(c)):
            actions.append({"kind": "retrace"})
            continue
        if (m := _F_SHAPE.fullmatch(c)) or (m2 := _F_CIRCLE2.fullmatch(c)):
            if m:
                num, unit = (m.group(1), m.group(2)) if m.group(1) else (m.group(4), m.group(5))
                size = _fdist(num, unit) if num else None
                shape, side = _SHAPE_NAMES[m.group(3)], m.group(6) or "right"
            else:
                size, shape, side = None, "circle", m2.group(1) or "right"
            if size is not None and not 0 < size <= 20:
                return None
            actions.append({"kind": "pattern", "shape": shape, "size": size, "side": side})
            continue
        if m := _F_FACE.fullmatch(c):
            actions.append({"kind": "compass", "north": 0.0, "east": 0.0, "up": 0.0,
                            "face": HEADINGS[m.group(1)]})
            continue
        if (m := _F_CMOVE1.fullmatch(c)) or (m2 := _F_CMOVE2.fullmatch(c)):
            word, d = ((m.group(1), _fdist(m.group(2), m.group(3))) if m
                       else (m2.group(3), _fdist(m2.group(1), m2.group(2))))
            if d <= 0:
                return None
            actions.append(_compass_move(word, d))
            continue
        if (m := _F_CBARE.fullmatch(c)) and len(clauses) > 1:  # "go north then land": one leg
            actions.append(_compass_move(m.group(1), BARE_MOVE_M))
            continue
        if _F_HOLD.fullmatch(c):
            # "rise and hover", "go left 2 m and stay there": it holds after a step anyway.
            # Alone: hold here when flying, take off when landed, else let the LLM decide.
            if actions:
                continue
            if len(clauses) == 1 and airborne:
                actions.append({"kind": "stop"})
                continue
            if landed or not telemetry:
                actions.append({"kind": "takeoff", "alt": DEFAULT_TAKEOFF_M})
                continue
            return None
        if _F_FLY.fullmatch(c) and len(clauses) == 1:  # "fly" / "can you fly?" on the ground
            if not landed:
                return None  # already up: fly where? the LLM can ask
            actions.append({"kind": "takeoff", "alt": DEFAULT_TAKEOFF_M})
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
        elif (m := _F_TURN.fullmatch(c)) or (m2 := _F_TURN2.fullmatch(c)):
            side, num = (m.group(1), m.group(2)) if m else (m2.group(2), m2.group(1))
            deg = _fnum(num, 90.0)
            if not 0 < deg <= 360:
                return None
            mv["yaw_deg"] = deg if side == "right" else -deg
        elif (m := _F_BARE.fullmatch(c)) and (len(clauses) > 1 or m.group(1) in ("up", "down")):
            # "rise, go left, then right": one leg each. Alone, only up/down (small and
            # obvious); a bare "go left" goes to the LLM, which can ask how far.
            word = m.group(1)
            d = BARE_MOVE_M
            if word == "up" and not actions and landed:
                actions.append({"kind": "takeoff", "alt": DEFAULT_TAKEOFF_M})  # "hey drone, go up"
                continue
            if word == "down" and not actions and not airborne:
                return None
            if word.startswith(("forward", "ahead")):
                mv["fwd"] = d
            elif word.startswith("back"):
                mv["fwd"] = -d
            elif word in ("left", "right"):
                mv["right"] = d if word == "right" else -d
            elif word == "up":
                mv["up"] = BARE_UPDOWN_M
            else:
                mv["up"] = -BARE_UPDOWN_M
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
    if not actions:
        return None
    if any(a["kind"] == "land" for a in actions[:-1]):
        return None  # "land and then go forward": odd, the LLM can sort it out
    if (telemetry and not telemetry.get("armed")
            and actions[0]["kind"] in ("move", "home", "compass", "pattern")):
        actions.insert(0, {"kind": "takeoff", "alt": DEFAULT_TAKEOFF_M})
    reply = DEFAULT_REPLY[actions[0]["kind"]] if len(actions) == 1 else "On it."
    return BrainResult(reply, actions, "fast")


_CM = re.compile(_NUM + r" ?(?:cm|centimet(?:re|er)s?)\b")


def _cm_to_m(t: str) -> str:
    """'forward 50 cm' -> 'forward 0.5 m': the patterns below only know metres, and read a bare
    number as metres ("50 cm" was a 50 m move; the gate caught it)."""
    return _CM.sub(lambda m: f"{_n(m.group(1), 0.0) / 100:g} m", t)


_NEGATED = re.compile(r"\b(?:dont|do not|doesnt|never|not|no need to|stop trying to)\b")


def rule_parse(text: str, telemetry: dict | None = None) -> BrainResult:
    # _clean: same reading of numbers as the fast path ("six metres" was 0.5 m here, "one point
    # five" was 5 m, "a couple of metres" also climbed 0.5 m)
    t = _cm_to_m(_clean(text))
    # "don't take off" took off, "don't move forward" moved forward, "never mind, don't land"
    # landed: a clause that says not to do something is dropped, never obeyed (_clean has
    # already turned commas and sentence ends into "and" and removed apostrophes)
    parts = t.split(" and ")
    kept = [p for p in parts if not _NEGATED.search(p)]
    negated = len(kept) != len(parts)
    t = " and ".join(kept)
    wont = BrainResult("Understood. I won't.", source="rules")
    if negated and not t:
        return wont
    if re.search(r"crash|flip|kamikaze|attack|chase|ram |hit (him|her|them|me)|out the window", t):
        return BrainResult("Absolutely not. I'd like to keep my propellers.", source="rules")
    # every word is_urgent() sends here must do something ("abort" and "halt" did nothing)
    if re.search(r"\b(stop|freeze|halt|abort|hold on|hold it|wait)\b|thats enough|cut it out|"
                 r"kill it", t):
        return BrainResult("Holding.", [{"kind": "stop"}], "rules")
    if re.search(r"\bland\b|come down|touch down|get down", t):
        return BrainResult("Coming down.", [{"kind": "land"}], "rules")
    # "come back the way you came" isn't home; after moves in the same sentence it comes last
    # ("go west 2 m and come back the way you came" used to drop the move)
    retrace = False
    if m := _F_RETRACE.search(t):
        t = (t[:m.start()] + t[m.end():]).strip()
        if not re.search(r"\b(?:forward|ahead|back|left|right|up|down|north|south|east|west|"
                         r"higher|lower)\b", t):
            return BrainResult("Going back.", [{"kind": "retrace"}], "rules")
        retrace = True
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
    elif not actions and (m := re.search(
            r"\b(?:go|fly|climb|rise|descend|drop|get|come|hover|stay|move)(?: (?:up|down))? "
            r"(?:to|at) " + _NUM + r" ?m\b", t)):
        # "go up to 3 metres" is a height, not "up" + a default 0.5 m (it used to climb 0.5)
        actions.append({"kind": "height", "alt": _n(m.group(1), DEFAULT_TAKEOFF_M)})
    elif m := re.search(r"\b" + _F_CDIR + r" (?:by |for )?" + _NUM + r" ?m\b|" + _NUM +
                        r" ?m (?:to the |due )?" + _F_CDIR, t):
        word, num = (m.group(1), m.group(2)) if m.group(1) else (m.group(4), m.group(3))
        actions.append(_compass_move(word, _n(num, BARE_MOVE_M)))
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
            r"(?:" + _NUM + _UNIT + r" (?:to (?:the |your )?)?)?" + _DIR, td)
            if m.start(2) not in taken]  # "1 m back", "5 m to the left"
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
        if not mv["up"] and (m := re.search(r"\b(higher|lower)\b", td)):
            mv["up"] = HIGHER_M if m.group(1) == "higher" else -HIGHER_M
        if re.search(r"turn around|spin around|360|three sixty|full (?:turn|spin|circle)", t):
            mv["yaw_deg"] = 360.0
        elif re.search(r"turn back|other way|180|one eighty", t):
            mv["yaw_deg"] = 180.0
        elif m := re.search(r"(?:turn|rotate|spin|yaw) (?:to the )?(left|right)(?: by| for)?(?: " + _NUM + r")?", t):
            deg = _n(m.group(2), 90.0)
            mv["yaw_deg"] = deg if m.group(1) == "right" else -deg
        elif m := re.search(r"(?:turn|rotate|spin|yaw) (?:by )?" + _NUM +
                            r"(?: degrees)? (?:to (?:the |your )?)?(left|right)", t):
            deg = _n(m.group(1), 90.0)  # "turn 90 degrees to the left"
            mv["yaw_deg"] = deg if m.group(2) == "right" else -deg
        if any(mv[k] for k in ("fwd", "right", "up", "yaw_deg")):
            actions.append(mv)
    if not actions and _task(t) != "none":
        actions.append({"kind": "look", "action": _task(t), "question": text})
    if not actions:
        return wont if negated else BrainResult(
            "I have no idea what that means, and I'm a drone.", source="rules")
    if retrace:
        actions.append({"kind": "retrace"})
    if (telemetry and not telemetry.get("armed") and actions[0]["kind"] in ("move", "mission")):
        actions.insert(0, {"kind": "takeoff", "alt": DEFAULT_TAKEOFF_M})
    return BrainResult(DEFAULT_REPLY[actions[-1]["kind"]] if actions[0]["kind"] != "takeoff"
                       or len(actions) == 1 else "On it.", actions, "rules")
