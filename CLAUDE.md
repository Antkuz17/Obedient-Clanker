# CLAUDE.md

Voice-controlled indoor drone for Hack Western 13 (Nov 20–22, 2026). Tracks: Best Use of QNX,
Best Use of Vultr. Two people.

**Flying the real drone (any laptop):** [docs/RUN_FLIGHTS.md](docs/RUN_FLIGHTS.md).

**Read first:** [docs/HANDOVER.md](docs/HANDOVER.md) (current state, how to run everything, gotchas), then [docs/PRD.md](docs/PRD.md) (the plan, the split, the contracts), [docs/TODO.md](docs/TODO.md) (the to-do list), then
[docs/STATUS.md](docs/STATUS.md) (what's built, what's verified, what's next). Hardware and
flight details: `docs/hardware.md`, `docs/flight-setup.md`, `docs/qnx.md`.

## Who does what

Roles, not "Person A/B": both people run Claude in this repo, so letters were read both ways.
Agreed split (Anton, 2026-11-21; partner to confirm):

- **Drone lead (Anton, GitHub `Antkuz17`)**: everything on the drone and on QNX.
  `edge/` (core, safety gate, executor, watchdog, MAVLink/USB relay), **`edge/flow/` optical
  flow**, **`edge/vision/` on-board TFLite person detection**, `edge/camgrab`, rangefinder,
  QNX real-time (priorities, supervisor, jitter, C watchdog loop), FC parameters, transmitter,
  hardware build and every flight test.
- **Cloud lead (teammate)**: everything off the drone that decides *what* to do.
  `station/` (brain/LLM, command schema, planner that sends protocol v1 commands, localization,
  voice in/out) and `cloud/` (Vultr app, models, dashboard, VLM tasks, visual memory).

The boundary is protocol v1 (`shared/protocol.py`, PRD section 5): the cloud lead's planner
sends primitives; the drone lead's edge executes them safely. Changing it needs both.

Build only your own role's modules (ask which role the person you're working with has); don't
write the other role's modules beyond the stubs and interfaces in PRD section 5. Keys (Vultr, ElevenLabs) are in `.env`, never in git; see `.env.example`.

## Commands

```sh
uv venv --python 3.11 .venv && uv pip install --python .venv -e '.[dev]'
.venv/bin/python -m pytest -q                  # all tests (sim-based, about 90 s)
.venv/bin/python -m edge --sim                 # fake drone: TCP :7000, video :8080
.venv/bin/python -m tools.cmd                  # REPL: takeoff alt=1 / move_body fwd=1 / t / land
# ArduPilot SITL (Docker/OrbStack; x86 binary runs under Rosetta)
docker build -t hope-sitl tools/sitl && docker run --rm -d --name hope-sitl -p 5760:5760 hope-sitl
.venv/bin/python -m tools.sitl_flight              # scripted flight, must print ALL PASS
.venv/bin/python -m edge --mav tcp:127.0.0.1:5760 --sitl   # or fly it by hand with tools.cmd
```
Restart the SITL container (`docker restart hope-sitl`) between runs.

```sh
.venv/bin/python -m station.dev        # sim edge + video + cloud + station; dashboard http://127.0.0.1:8100
.venv/bin/python -m station.check_ai   # once keys are in .env: test every model + voice latency
.venv/bin/python -m station --edge 127.0.0.1:7100 --video-port 8180 --local --stub-locate  # no cloud
```

```sh
.venv/bin/python -m tools.supervisor tools/supervisor_laptop.json   # sim edge :7100 + fake camgrab, restarts on kill
.venv/bin/python -m tools.cmd 127.0.0.1:7100
```
C heartbeat watchdog: `cc -O2 -o edge/hbwd/hbwd edge/hbwd/hbwd.c -lpthread`; SITL check
`docker run --rm -d --name hope-sitl-hbwd -p 5770:5760 -p 5772:5762 hope-sitl && .venv/bin/python -m tools.sitl_hbwd`.

macOS: AirPlay Receiver listens on :7000, so `edge --sim` on the default port fails with
"address already in use"; pass `--port 7100` (or turn AirPlay Receiver off).

## Rules

- `shared/protocol.py` is the frozen A↔B contract. Change it only with both people agreeing,
  and update PRD section 5 at the same time.
- Every flight command goes through `edge/safety.py`. Never add a path to the vehicle that
  skips the gate. `stop` and `land` are always allowed, except while the pilot has another
  mode (the pilot always wins); landing beats stop.
- Code under `edge/` and `shared/` must run on the Pi's **Python 3.14** (QNX 8; the suite passes on
  3.14 and 3.11): stdlib + pymavlink + numpy
  + OpenCV only. No new pip dependencies there without checking they install on QNX.
- Safety limits live in `edge/config.py`. Tune them there, not inline.
- Keep `docs/STATUS.md` current at the end of every work session: done, verified, next.
- Commit in small logical commits on `main` (the team works on main during the hackathon).
