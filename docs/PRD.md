# PRD: Hope Drone Don't Go Kaboom

Status: v2, 2026-11-21 (Hack Western 13, day 2). Owners: **Person A: Flight + Voice** and
**Person B: Perception + Memory**. Both work on QNX *and* Vultr (see section 6).

## 1. Problem and pitch

Tell an indoor drone what you want in plain speech ("fly to the door and take a photo", "where
did I leave my keys?") and it does it, with a personality, and it **never does anything unsafe
even when the cloud, the laptop or a process dies**.

> Edge for reflexes, cloud for intelligence. Vultr thinks and remembers; QNX keeps it alive.

Target prizes: **Best Use of QNX** and **Best Use of Vultr**. Every feature below maps to a
judging point in one of the two tracks; anything that doesn't is a stretch goal.

## 2. Decisions locked in this PRD

These resolve conflicts between the earlier docs. Change them only by agreement.

| # | Decision | Why |
|---|---|---|
| D1 | **Positioning = onboard software optical flow (down webcam) + HC-SR04 height**, per [flight-setup.md](flight-setup.md). The README's ArUco outside-in setup is the **fallback** behind the same interface. | Newest plan; it puts the QNX Pi *on* the drone, which is the strongest QNX story. |
| D2 | **The QNX Pi 5 rides on the drone** and is the only thing that talks MAVLink to the flight controller (GPIO UART to `SERIAL1`). | Safety AI must run on embedded hardware ([qnx.md](qnx.md)). |
| D3 | **Forward camera = second webcam on the Pi.** The Pi streams it to the laptop. The iPhone XR is a fallback video source only. | One pose-synced video source; no phone mounting problem. |
| D4 | **The Edge exposes a small, safe flight API of primitives.** The Brain turns speech into `mission()` and then into a sequence of primitives. The Edge never trusts the Brain: every primitive passes the safety gate. | A clean contract between the layers. |
| D5 | **Laptop ↔ Pi link = newline-delimited JSON over TCP on Wi-Fi**, using only the Python stdlib on the Pi. The ESP8266 bridge becomes a backup telemetry link for Mission Planner only. | Fewest QNX dependencies; fewest unknowns. |
| D6 | **Python 3.14 on the Pi** (verified image), 3.11+ elsewhere. Pi: stdlib + pymavlink + numpy + OpenCV + tflite-runtime (all installed and verified, see qnx.md). Laptop/cloud: anything. | Matches the QNX image. |
| D7 | One task at a time. A new command cancels the running one. `stop` always wins. | Matches the README scope. |

## 3. Users and core flows

- **Operator** (at the laptop or the Nest Mini): speaks commands and hears replies.
- **Judge / remote user**: opens the Vultr dashboard on their phone, types or speaks a command,
  and watches live video and telemetry.
- **Safety pilot**: holds the FlySky with SwA = Land at all times. Not optional.

Flows that must work in the demo:

1. "Take off" → "move back 1 metre" → "turn around" → "land".
2. "Fly to the door and take a photo, then come back": scan → detect → approach → photo → return.
3. "What's written here?" / "Solve this math problem": hover, capture, VLM answer, spoken reply.
4. "Where did I leave my keys?": memory search → spoken answer (where and when) → fly to the pose.
5. "Fly up 10 m" / "Crash into the wall": the LLM refuses with attitude; if it doesn't, the
   safety gate rejects it and shows the reason.
6. Failsafes: pull the laptop's Wi-Fi → hover then land; palm → stop; a person steps in front →
   stop; `slay` the vision process → it restarts while the drone keeps hovering.

## 4. System architecture

```
 phone / dashboard ─► CLOUD (Vultr)                laptop STATION
                      Whisper, LLM [A]   ◄──────► mic, TTS → Nest Mini [A], planner [A],
                      VLM, memory [B], web [A]      detector + localization [B]
                                                        │  TCP JSON (protocol v1)
                                                        ▼  + MJPEG video back
                                              EDGE: QNX Pi 5 on the drone
                                              flight API, safety gate, watchdog,
                                              real-time [A]; optical flow,
                                              TFLite person + palm [B]
                                                        │  MAVLink, UART
                                                        ▼
                                              Matek H743 / ArduPilot 4.7.2 → F450
```

## 5. The contract: Edge protocol v1 (frozen at kickoff)

Source of truth: [`shared/protocol.py`](../shared/protocol.py). Both sides import it. Changing
it needs both people.

**Transport:** TCP, the Pi listens on port `7000`. One JSON object per line, UTF-8. One station
connects at a time.

### Station → Edge

| Message | Fields | Notes |
|---|---|---|
| `heartbeat` | `t` | At ≥ 5 Hz. Watchdog: none for **1.0 s** → hover; none for **5.0 s** → land. |
| `cmd` | `id`, `op`, `args` | One primitive. Answered with `ack`, then `done`. |

Ops (all distances in metres, angles in degrees, local frame = NED from the takeoff point):

| `op` | `args` | Meaning |
|---|---|---|
| `takeoff` | `alt` | Arm in GUIDED and climb to `alt`. |
| `move_body` | `fwd`, `right`, `up` | Relative move in the body frame. |
| `goto` | `n`, `e`, `alt` | Absolute local position. |
| `yaw` | `deg`, `relative` (bool) | Turn in place. |
| `scan` | `rate_dps` | Slow 360° yaw; finishes after one turn or when `stop` arrives. |
| `hover` | | Hold position. |
| `stop` | | Cancel whatever is running and hover. Always accepted. |
| `return_home` | | Go to (0, 0) at the current altitude. |
| `land` | | Land and disarm. Always accepted. |

### Edge → Station

| Message | Fields |
|---|---|
| `ack` | `id`, `ok`, `reason` (rejection text from the safety gate) |
| `done` | `id`, `status` = `ok` / `aborted` / `failed`, `reason` |
| `telemetry` (10 Hz) | `t`, `state`, `armed`, `mode`, `n`, `e`, `alt`, `yaw`, `vn`, `ve`, `vd`, `battery_v`, `flow_quality`, `range_m`, `active_cmd` |
| `event` | `kind` (`safety_reject`, `watchdog`, `person_close`, `palm_stop`, `restart`, `battery`), `detail` |

`state` ∈ `DISCONNECTED, IDLE, TAKING_OFF, BUSY, HOVER, LANDING, FAILSAFE`.

**Video:** `http://<pi>:8080/forward.mjpg` (stream) and `/forward.jpg` (one frame with an
`X-Pose: n,e,alt,yaw,t` header, for visual memory).

**Simulator:** `python -m edge --sim` runs the full Edge stack against a simulated vehicle and
serves the same protocol plus a synthetic room on video (labelled objects at known positions,
projected through the real camera model). Everything except optical flow and flight tuning can
be built against it. `python -m tools.cmd` is a REPL that speaks the protocol.

**Perception input (B → A, on the Pi):** B's vision process sends UDP JSON to
`127.0.0.1:7001`: `{"person_m": float|null, "palm": bool}` at ≥ 5 Hz. B's flow process sends
`{"fx", "fy", "q", "dt_us", "ground_m"}` (rad/s) to `127.0.0.1:7002`, and A's core forwards it
to the FC as `OPTICAL_FLOW`. They are separate processes, so `slay`-ing them never stops the
core.

**Localization (B → A, on the laptop):** `station/localize.py`:
`locate(name, jpeg, pose, K) -> {"n", "e", "dist", "bearing", "conf"} | None`. A's planner
calls it for "fly to the X"; until B's version lands, A uses a stub that reads the sim's
ground-truth world.

**Memory (B → A, on Vultr):** `POST /memory/search {"q"}` → `[{caption, n, e, alt, yaw, t,
jpeg_url, score}]`. A's planner and dashboard call it; B owns ingest and ranking.

### Safety gate defaults (`edge/config.py`)

| Rule | Default |
|---|---|
| Altitude window | 0.3–1.8 m (the HC-SR04 tops out at about 2 m) |
| Geofence | Box ±3 m N/E around home |
| Max single move | 3 m |
| Max horizontal / vertical speed | 0.5 / 0.3 m/s |
| Battery | Reject new moves below 10.6 V (3S); auto-land below 10.2 V |
| Flow quality | Reject moves when quality < 50 |
| Person too close | Reject forward moves if a person is closer than ~1.0 m (bounding-box height proxy) |
| State | Moves need `armed` + GUIDED; `stop` and `land` are always allowed |

## 6. Work split

The principle: **both people work on QNX and on Vultr.** Each person owns full features from
the Pi to the cloud, which means some blocking between you, and that's accepted. The coupling
points are the interfaces in section 5, and stubs cover them until the real piece lands.

Each person has one QNX-heavy problem, one Vultr-heavy problem, one "hard maths" problem and a
fair share of hardware chores.

- **Person A: Flight + Voice.** You fly the drone and make it talk.
- **Person B: Perception + Memory.** The drone sees and remembers.

### Person A: Flight + Voice

| # | Task | Difficulty | Track |
|---|---|---|---|
| A1 | Edge core on the Pi: MAVLink link (GUIDED, arm, takeoff, position targets, origin, telemetry), safety gate, watchdog, protocol v1 server, simulator | M | QNX |
| A2 | **QNX real-time**: port to the Pi, `SCHED_FIFO` priorities, process split, restart on kill (HAM or a supervisor), jitter/latency logger and the live graph under CPU load. *Hard systems.* | H | QNX |
| A3 | Voice loop: mic → VAD → Whisper on Vultr → reply → TTS → Nest Mini. Latency under 3 s | M | Vultr |
| A4 | **LLM + planner**: speech → `mission(where, what, after)` tool call with personality and refusals → a primitive sequence, including scan-until-found, approach and come back. *Hard logic.* | H | Vultr |
| A5 | **Vultr deployment + phone dashboard**: live video relay, telemetry, command box, flight log, a command queue with one active user, the cloud ↔ station link | H | Vultr |
| A6 | Hardware: FC params, compass + `COMPASS_MOT` calibration, Pi UART wiring, Nest pairing, Vultr account and GPU check | M | both |

### Person B: Perception + Memory

| # | Task | Difficulty | Track |
|---|---|---|---|
| B1 | **Optical flow → `OPTICAL_FLOW`** on the Pi: LK features at 320×240, px → rad/s, quality from inliers, derotation, sign check, `EK3_FLOW_DELAY` tuning. *Hard maths.* | H | QNX |
| B2 | **On-board AI on QNX**: TFLite person detection on the forward camera and hand-landmark palm stop, sent to the core over UDP (section 5) | M-H | QNX |
| B3 | **Target localization**: open-vocabulary detector → bearing from the intrinsics → range from box size or depth → a local NED goal using the frame's pose. Also "where I'm pointing" | H | both |
| B4 | VLM tasks on Vultr: read, solve, describe, photo capture and storage | M | Vultr |
| B5 | **Visual memory on Vultr**: frames → caption + embedding + pose + time → vector DB; the `/memory/search` endpoint and re-ranking | H | Vultr |
| B6 | Hardware: HC-SR04 wiring (divider, free pads, `RNGFND1_*`), camera mounts, ESP8266 baud, power bank, 👍/👎 gestures | M | both |

Shared and in pairs: battery charging, first power-up (smoke stopper), every real flight (one
person on the laptop, the other on the transmitter), and Devpost (A writes up the real-time
and dashboard work, B writes up the on-board AI and memory).

### Where you block each other, and the stub that covers it

| Needs | From | Stub until it lands |
|---|---|---|
| B's flow for real position hold | A's core forwarding flow to the FC | A uses the sim/SITL; B's flow can be tested on the Pi with the drone unpowered |
| A's planner "fly to the door" | B3 `locate()` | Sim ground truth (`station/localize_stub.py`) |
| A's "where are my keys" flow | B5 `/memory/search` | A hard-coded result |
| B's memory ingest needs pose-tagged frames | A1 `/forward.jpg` with `X-Pose` | Already in the sim |
| B's person/palm needs to stop the drone | A1 perception UDP input | Already in the core |

## 7. Milestones

| Phase | Exit criteria | Who |
|---|---|---|
| **P0: Contract** (now, ~1 h) | `shared/protocol.py` frozen; `python -m edge --sim` works; both can fly the sim | A |
| **P1: Parallel** | A: speech → mission → primitives flies the sim; edge core on the QNX Pi talks to the FC; dashboard on Vultr. B: flow running on the Pi; `locate()` works on sim frames; memory ingest + search on Vultr | both |
| **P2: Integration** | B's station drives the real Edge; demo flows 1, 2 and 5 work end to end | together |
| **P3: Wow** | Visual memory (flow 4), failsafe demos (flow 6), latency graph, dashboard on a judge's phone | both |
| **P4: Freeze** | 2 h before submission: no new features, rehearse the demo 3×, record a backup video, Devpost | both |

Cut order if behind: Plan B positioning tuning → gestures → video tasks → visual memory flying
(keep the spoken answer) → person detection (keep palm). **Never cut:** safety gate, watchdog,
Land switch.

## 8. Success metrics (demo-day)

- Position hold drift under 0.5 m over 20 s.
- Voice → drone starts moving in under 3 s.
- Safety loop period jitter under 2 ms at 100% CPU on QNX (show the graph).
- The watchdog lands the drone within 6 s of the station dying.
- 0 safety-gate bypasses: every primitive in the flight log has a gate verdict.

## 9. Risks

| Risk | Mitigation |
|---|---|
| Flow is unstable on the venue floor | Patterned mat, bright light; ArUco fallback behind the same interface |
| QNX blocks something (camera, UART, tflite) | Prototype on Linux/Mac with `--sim`; Pi 4 Linux fallback (weaker for the track) |
| Vultr has no GPU | Vultr Serverless Inference for the LLM/Whisper; vision on the GPU laptop |
| Wi-Fi at the venue | Phone hotspot; the watchdog covers drops |
| 10" props, no guards | Cleared area, people behind a line, pilot on the Land switch, SITL first |

## 10. Repo layout

```
shared/protocol.py   contract (both)
edge/                QNX Pi core: server, safety, watchdog, mav, sim, video (A)
edge/flow/           optical flow process (B)
edge/vision/         TFLite person + palm process (B)
tools/               CLI and test tools (A)
station/             laptop: voice, LLM, planner (A); detector + localize (B)
cloud/               Vultr: API + dashboard (A); memory + VLM (B)
tests/
```
