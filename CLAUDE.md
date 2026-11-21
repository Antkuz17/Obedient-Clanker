# CLAUDE.md

Voice-controlled indoor drone for Hack Western 13 (Nov 20–22, 2026). Tracks: Best Use of QNX,
Best Use of Vultr. Two people.

**Read first:** [docs/PRD.md](docs/PRD.md) (the plan, the split, the contracts), [docs/TODO.md](docs/TODO.md) (the to-do list), then
[docs/STATUS.md](docs/STATUS.md) (what's built, what's verified, what's next). Hardware and
flight details: `docs/hardware.md`, `docs/flight-setup.md`, `docs/qnx.md`.

## Who does what

- **Person A: Flight + Voice** (the repo user working with Claude): `edge/` core, `tools/`,
  QNX real-time, voice, LLM, planner, Vultr deployment and dashboard.
- **Person B: Perception + Memory** (teammate): `edge/flow/`, `edge/vision/`, target
  localization, VLM tasks, visual memory.

By default, build Person A's items. Don't write B's modules beyond the stubs and interfaces in
PRD section 5.

## Commands

```sh
uv venv --python 3.11 .venv && uv pip install --python .venv -e '.[dev]'
.venv/bin/python -m pytest -q                  # all tests (sim-based, about 20 s)
.venv/bin/python -m edge --sim                 # fake drone: TCP :7000, video :8080
.venv/bin/python -m tools.cmd                  # REPL: takeoff alt=1 / move_body fwd=1 / t / land
# ArduPilot SITL (Docker/OrbStack; x86 binary runs under Rosetta)
docker build -t hope-sitl tools/sitl && docker run --rm -d --name hope-sitl -p 5760:5760 hope-sitl
.venv/bin/python -m tools.sitl_flight              # scripted flight, must print ALL PASS
.venv/bin/python -m edge --mav tcp:127.0.0.1:5760 --sitl   # or fly it by hand with tools.cmd
```
Restart the SITL container (`docker restart hope-sitl`) between runs.

## Rules

- `shared/protocol.py` is the frozen A↔B contract. Change it only with both people agreeing,
  and update PRD section 5 at the same time.
- Every flight command goes through `edge/safety.py`. Never add a path to the vehicle that
  skips the gate. `stop` and `land` are always allowed; landing beats stop.
- Code under `edge/` and `shared/` must run on the Pi's **Python 3.14** (QNX 8; the suite passes on
  3.14 and 3.11): stdlib + pymavlink + numpy
  + OpenCV only. No new pip dependencies there without checking they install on QNX.
- Safety limits live in `edge/config.py`. Tune them there, not inline.
- Keep `docs/STATUS.md` current at the end of every work session: done, verified, next.
- Commit in small logical commits on `main` (the team works on main during the hackathon).
