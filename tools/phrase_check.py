"""Dry run: what would the brain do with these phrases? Nothing flies, no drone needed.

    .venv/bin/python -m tools.phrase_check "go north 3 m" "fly in a square" "don't take off"
    .venv/bin/python -m tools.phrase_check --file my_phrases.txt     # one per line, '#' comments
    .venv/bin/python -m tools.phrase_check --llm "take a little trip west and come back"
    echo "land" | .venv/bin/python -m tools.phrase_check -

Each phrase prints the source (fast = strict regex path, rules = fallback regexes, llm = the
model), the reply Jarvis would speak and the actions the planner would get. Phrases run in
order through one brain, so "do that again" repeats the phrase before it. The brain assumes
the drone is flying (--landed to pretend it's on the ground).
"""
from __future__ import annotations

import argparse
import asyncio
import json
import sys

from station.brain import Brain


async def main(a) -> int:
    phrases = [p for p in a.phrases if p != "-"]
    if "-" in a.phrases:
        phrases += [ln.strip() for ln in sys.stdin if ln.strip()]
    if a.file:
        with open(a.file) as f:
            phrases += [ln.strip() for ln in f if ln.strip() and not ln.startswith("#")]
    if not phrases:
        print("no phrases given (see --help)")
        return 2
    ai = None
    if a.llm:
        from station.ai import AI
        from station.config import Settings, load_dotenv
        load_dotenv()
        ai = AI(Settings())
    brain = Brain(ai)
    tel = {"armed": not a.landed, "alt": 0.0 if a.landed else 1.0, "n": 0.0, "e": 0.0,
           "yaw": 0.0, "mode": "GUIDED", "battery_v": 12.4}
    empty = 0
    for p in phrases:
        r = await brain.think(p, tel)
        empty += not r.actions
        lat = f" {r.latency_s:.2f}s" if r.latency_s else ""
        print(f'"{p}"  [{r.source}{lat}]')
        print(f"   says:  {r.reply}")
        if r.actions:
            for act in r.actions:
                print(f"   do:    {json.dumps(act)}")
        else:
            print("   do:    (nothing)")
    if ai:
        await ai.aclose()
    print(f"\n{len(phrases)} phrases, {empty} with no action")
    return 0


if __name__ == "__main__":
    ap = argparse.ArgumentParser(prog="tools.phrase_check", description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("phrases", nargs="*", help="phrases ('-' reads stdin)")
    ap.add_argument("--file")
    ap.add_argument("--llm", action="store_true", help="use the Vultr LLM (key in .env)")
    ap.add_argument("--landed", action="store_true", help="pretend the drone is on the ground")
    sys.exit(asyncio.run(main(ap.parse_args())))
