# Design: Obedient Clanker

A drone you command by voice. Say "rise and do a 360" or "go two metres to the left and take a
photo" and it flies, answers back, and refuses anything unsafe. This page is the design and the
contracts between the parts. What works today is in [STATUS.md](STATUS.md); how the pieces
connect, and what happens when one dies, is in [flow.md](flow.md).

> Edge for reflexes, cloud for intelligence. Vultr thinks; QNX keeps it alive.

## 1. Goal

Plain speech in, safe flight out, for people whose hands are busy. The drone must refuse unsafe
commands, and the layers that keep people safe must not depend on the cloud or the laptop.

## 2. Design decisions

| # | Decision | Why |
|---|---|---|
| D1 | **Outdoors: GPS for sideways position, a downward HC-SR04 rangefinder for height.** Indoors: software optical flow from the down camera + the same rangefinder (it has not held position in the air yet, so indoors is a no-fly demo mode). | Flow quality fell to 0 at hover throttle; GPS doesn't care. The flight controller's own height estimate drifts on the ground. |
| D2 | **The QNX Pi 5 rides on the drone** and is the only thing that talks MAVLink to the flight controller, over USB. | The safety layer has to be on the vehicle. |
| D3 | **Two cameras on the Pi:** forward (person detection, video, photos) and down (optical flow). | One pose-synced video source. |
| D4 | **The edge exposes a small flight API of primitives.** The brain turns speech into actions, the planner turns actions into primitives, and the edge never trusts either: every primitive passes the safety gate. | A clean contract between the layers. |
| D5 | **Laptop ↔ Pi link = newline-delimited JSON over TCP on Wi-Fi**, Python stdlib only on the Pi. | Fewest QNX dependencies. |
| D6 | **Python 3.14 on the Pi**, 3.11+ elsewhere. Pi: stdlib + pymavlink + numpy + OpenCV + tflite-runtime. | Matches the QNX image. |
| D7 | One task at a time. A new command cancels the running one. `stop` always wins; landing beats stop. | Predictable. |
| D8 | **Speech-to-text and the brain are hosted models on Vultr Serverless Inference** (`whisper-large-v3-turbo`, `glm-5.3-flash`); common flying phrases are parsed by rules with no LLM call. See [models.md](models.md). | Latency is mostly speech-to-text. |
| D9 | **Voice out = ElevenLabs**, played by the browser; Vultr `chatterbox` as a fallback. | Best voice. |
| D10 | **Mic = push-to-talk on the dashboard.** The cloud app (FastAPI) currently runs on the laptop; the station dials out to it over a websocket, so it can move to a VM unchanged ([cloud/deploy.md](../cloud/deploy.md)). | NAT-safe. |
| D11 | **Pi ↔ flight controller over USB through a small root relay process**; the core runs unprivileged and accepts only the allowed station IPs. | Root confined to one small user-space USB driver. |
| D13 | **The downward rangefinder is required**, and must read valid on the ground. Takeoff height is decided by the rangefinder on the laptop (`--rng`). | SITL: no rangefinder, no position; field: the FC's height drifts up on the ground. |
| D14 | **"The AI suggests, QNX decides."** Separate processes restarted by our supervisor; a C watchdog at top `SCHED_FIFO` priority; every command through the gate. | [qnx.md](qnx.md). |
| D15 | **A safety pilot holds the transmitter on every flight**: one switch takes over or lands. The computer never fights the pilot. | The last layer. |

## 3. Users

- **Operator:** speaks or types commands on the dashboard and hears replies.
- **Viewers:** anyone with the dashboard open can watch and press STOP or LAND.
- **Safety pilot:** holds the FlySky with SwA = Land at all times. Not optional.

## 4. System architecture

```
 dashboard (browser) ─► CLOUD APP                    laptop STATION
                        Whisper, LLM, vision  ◄────► planner, rangefinder height guard,
                        (Vultr), voice, memory        localization
                                                          │  TCP JSON (protocol v1)
                                                          ▼  + MJPEG video back
                                                EDGE: QNX Pi 5 on the drone
                                                flight API, safety gate, watchdogs,
                                                optical flow, TFLite person detector
                                                          │  MAVLink, USB
                                                          ▼
                                                Matek H743 / ArduPilot 4.7.2 → F450
```

## 5. The contract: Edge protocol v1 (frozen)

Source of truth: [`shared/protocol.py`](../shared/protocol.py). Both sides import it. Changing
it needs both leads.

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
| `stop` | | Cancel whatever is running and hover. Always accepted, except while the pilot has another mode (the pilot wins). |
| `return_home` | | Go to (0, 0) at the current altitude. |
| `land` | | Land and disarm. Always accepted, except while the pilot has another mode (the pilot wins). |

### Edge → Station

| Message | Fields |
|---|---|
| `ack` | `id`, `ok`, `reason` (rejection text from the safety gate) |
| `done` | `id`, `status` = `ok` / `aborted` / `failed`, `reason` |
| `telemetry` (10 Hz) | `t`, `state`, `armed`, `mode`, `n`, `e`, `alt`, `yaw`, `vn`, `ve`, `vd`, `battery_v`, `flow_quality`, `range_m`, `active_cmd` |
| `event` | `kind` (`safety_reject`, `watchdog`, `person_close`, `palm_stop`, `restart`, `battery`, `position`), `detail` |

`state` ∈ `DISCONNECTED, IDLE, TAKING_OFF, BUSY, HOVER, LANDING, FAILSAFE`.

**Video:** `http://<pi>:8080/forward.mjpg` (stream) and `/forward.jpg` (one frame with an
`X-Pose: n,e,alt,yaw,t` header, for visual memory).

**Simulator:** `python -m edge --sim` runs the full Edge stack against a simulated vehicle and
serves the same protocol plus a synthetic room on video (labelled objects at known positions,
projected through the real camera model). Everything except optical flow and flight tuning can
be built against it. `python -m tools.cmd` is a REPL that speaks the protocol.

**Perception input (on the Pi):** the vision process sends UDP JSON to
`127.0.0.1:7001`: `{"person_m": float|null, "palm": bool}` at ≥ 5 Hz, optionally `"t"` (frame
capture time) and `"infer_ms"` (`edge/vision`). The flow process sends
`{"fx", "fy", "q", "dt_us", "ground_m"}` (rad/s) to `127.0.0.1:7002`, to the core; with `--fc` it sends
`OPTICAL_FLOW` straight to the FC and the core only records the quality. The rangefinder bridge (`tools/rangefinder.py`) sends
`{"range_m"}` (down, to the FC) and `{"fwd_m"}` (forward HC-SR04, -1 = no echo; the gate
refuses forward moves ending within `obstacle_close_m`) to the same port. They are separate processes, so `slay`-ing them never stops the
core.

**Camera frames (on the Pi):** `edge/camgrab/camgrab.c` (QNX `libcamapi`) publishes the
latest raw frame to shared memory: `camgrab_fwd` (forward camera) and `camgrab_down` (down
camera, optical flow). Read them with `edge.camera.ShmFrameReader(name)`: `.wait_new(seq)`,
`.gray()` (cheap, for flow) and `.bgr()`. On a laptop, `python -m tools.fake_camgrab --name
camgrab_down --webcam 0` publishes the same layout from a webcam, so flow and vision code run
unchanged before they reach QNX.

**Localization (on the laptop):** `station/localize.py`:
`await locate(ai, name, jpeg, pose, K) -> {"n", "e", "dist", "bearing", "conf"} | None`. The planner
calls it for "fly to the X"; `station/localize_stub.py` reads the sim's ground-truth world
instead.

**Memory (cloud app):** `POST /memory/search {"q"}` → `[{caption, n, e, alt, yaw, t,
jpeg_url, score}]`. The planner and dashboard call it.

### Safety gate limits

Defaults live in `edge/config.py`; the real drone always runs the slow-and-low profile in
`edge/__main__.py` unless started with `--full-limits`.

| Rule | Real drone | `edge/config.py` default |
|---|---|---|
| Altitude window | 0.3–1.5 m; takeoff ≤ 1.2 m | 0.3–5 m |
| Geofence | off | off (`fence_half` restores a box around the first takeoff) |
| Max single move | 2 m | none |
| Speed | 0.5 m/s horizontal and descent, 1.0 m/s climb | 1.5 / 10 / 1.5 m/s |
| Battery (3S, under load) | reject new moves below 10.0 V; auto-land below 9.6 V for 2 s | same |
| Flow quality | reject moves below 50; not checked with `--gps` | same |
| Person ahead | any forward travel must end ≥ 1.0 m from a person (estimate read 30% closer); a running forward move is stopped; no live detector = no forward moves | detector not required |
| Forward obstacle | ≥ 0.6 m from an echo; the sensor is not wired, and an unwired sensor reads "clear" | same |
| State | moves need `armed` + GUIDED + a position estimate; `stop`, `hover` and `land` are always allowed unless the pilot has taken over | same |

On the laptop, in `--rng` mode (`station/planner.py`): land if the rangefinder is lost for 1 s,
reads above 1.2 m, the drone sits within 10 cm of the ground for 3 s, or it turns more than 45°
with no turn commanded. These depend on the laptop and the link; the edge's link watchdog
(hover after 1 s, land after 5 s) covers losing both.

## 6. Not built

- Palm-to-stop and other gestures (`palm` is always false).
- The forward obstacle sensor's hardware.
- Flying to a named object or a remembered place on the real drone: the planner, `locate()`
  and visual memory exist and work in the simulator only.
- A public deployment of the cloud app.

## 7. Repo layout

```
shared/protocol.py   the laptop ↔ drone contract
edge/                QNX Pi: server, safety gate, executor, watchdog, MAVLink, sim, video
edge/flow/           optical flow process
edge/vision/         TFLite person detector process
station/             laptop: brain (rules + LLM), planner, voice, localization
cloud/               cloud app, dashboard, visual memory
tools/               deploy, preflight, flight tests, supervisor, SITL harness
tests/
```
