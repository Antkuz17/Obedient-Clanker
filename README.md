# Hope Drone Don't Go Kaboom

A voice-controlled indoor drone assistant with a personality, a bit like Jarvis. You tell it
"go over there and do X" and it does it. Built at Hack Western 13 (Nov 20–22, 2026).

> "Edge for reflexes, cloud for intelligence. Vultr thinks and remembers; QNX keeps it alive.
> Kill the cloud, kill the laptop, kill a process: the drone still stops on your hand."

Target tracks: **Best Use of QNX** and **Best Use of Vultr**.

**Plan and split:** [docs/PRD.md](docs/PRD.md). **Progress:** [docs/STATUS.md](docs/STATUS.md).

```sh
uv venv --python 3.11 .venv && uv pip install --python .venv -e '.[dev]'
.venv/bin/python -m edge --sim     # simulated drone: protocol on :7000, video on :8080
.venv/bin/python -m tools.cmd      # type: takeoff alt=1, move_body fwd=1, t, land
.venv/bin/python -m pytest -q
```

## What it does

- **Voice commands, one task at a time:** "turn around", "move back", "tell me what's written
  here", "solve this math problem", "fly to the door and take a photo". No chained tasks.
- **Personality:** it refuses dumb or dangerous orders with attitude ("crash into a wall?
  absolutely not"). Hard safety rules in code back that up.
- **Hand gestures:** ✋ palm = stop and hover, 👉 point = go there, 👍 confirm, 👎 cancel.
- **Visual memory:** "where did I leave my keys?" It searches everything it has seen, says
  where and when, then flies there.
- **Jarvis from anywhere:** a web dashboard on Vultr. Anyone can open it on a phone and command
  the drone.

## Architecture

```
 phone / dashboard ──► VULTR (cloud brain)
                        • speech-to-text (Whisper)
                        • LLM + vision model: command -> mission(where, what, after)
                        • visual memory: captions + embeddings + position, vector search
                        • dashboard backend, flight logs
                              │
 laptop station ◄─────────────┘   mic, speakers (TTS), drone video from the iPhone
        │ missions
        ▼
 QNX Raspberry Pi 5 (reflexes, real-time)
   • ArUco tracker -> VISION_POSITION_ESTIMATE (indoor "GPS")
   • safety gate: altitude, fence, battery, distance to people
   • watchdog: laptop or cloud lost -> hover, then land
   • gesture AI (QNX TFLite hand landmarks)
        │ only safe commands, MAVLink
        ▼
 Matek H743 / ArduPilot 4.7.2 (GUIDED mode) ──► F450
```

### Indoor positioning (no GPS, no optical-flow sensor)

- **Plan A (outside-in):** a webcam looks down at the flying area. An ArUco marker sits on top
  of the drone. OpenCV computes position and heading at about 30 Hz and sends them as
  `VISION_POSITION_ESTIMATE`.
- **Plan B (inside-out, upgrade):** the QNX Pi rides on the drone, powered by a USB power bank,
  with the webcam pointing down at ArUco markers on the floor.

ArduPilot parameters for both:

| Parameter | Value |
|---|---|
| `VISO_TYPE` | 1 (MAVLink) |
| `EK3_SRC1_POSXY` | 6 (ExternalNav) |
| `EK3_SRC1_POSZ` | 6 (ExternalNav) |
| `EK3_SRC1_YAW` | 6 (ExternalNav) |

Also send `SET_GPS_GLOBAL_ORIGIN` once at startup.

### Commands

The LLM turns speech into a single call:

```
mission(where: "the door" | "3 m left" | "where I'm pointing" | "back to me",
        what:  photo | video | describe | read | solve | hover | look_around | follow | land,
        after: come_back | stay | land)
```

If the target isn't in view, the drone does a slow 360° scan first. Each command is then
checked by the safety gate on QNX before anything flies.

## QNX demo points

1. A live latency graph shows the safety loop meeting its deadline while the CPU is maxed out
   by AI work. Optionally, the same test on Linux shows jitter for comparison.
2. Kill the vision process mid-flight. QNX restarts it and the drone keeps hovering.
3. A palm gesture stops the drone even with the laptop unplugged.

## Vultr demo points

1. "Where are my keys?": visual memory search, then the drone flies there.
2. A judge commands the drone from their own phone through the Vultr dashboard.
3. Big models (vision-language model, Whisper, open-vocabulary detector) run on Vultr: on a GPU
   if the credits allow, otherwise on Vultr Serverless Inference with the vision models on the
   GPU laptop.

## Build order

1. ArduPilot SITL + Gazebo with a simulated camera; build everything here first.
2. ArUco tracker -> MAVLink bridge -> indoor position hold on the real drone.
3. Voice -> LLM -> `mission()` -> relative moves.
4. Safety gate + watchdog on the QNX Pi.
5. Detector -> "fly to the named thing"; then read, solve, photo and describe.
6. Vultr: models, visual memory, dashboard.
7. Gestures; then move the Pi onboard (Plan B).

## Open questions

- Laptop ↔ flight controller link: the ESP8266 Wi-Fi bridge (`SERIAL2`). First find its
  firmware baud rate and match `SERIAL2_BAUD`. See [docs/hardware.md](docs/hardware.md).
- Do the Vultr credits include GPUs?
- Which QNX open-source AI modules are available (oss.qnx.com)?

## Safety

The props are 10", and there are no prop guards yet. Fly in a cleared area with people behind
the line, keep a pilot on the transmitter with the Land switch ready at all times, and test in
SITL before flying.
