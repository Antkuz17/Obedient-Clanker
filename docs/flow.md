# How the system works

What runs where, what talks to what, which processes matter most, and what happens when any
piece dies. Use it for the presentation and for debugging. The message formats themselves are
in [PRD.md](PRD.md) section 5 and `shared/protocol.py`. Numbers here are taken from the code
(`edge/config.py`, `tools/supervisor_pi.json`, `edge/server.py`, `station/station.py`); if you
change one there, change it here.

> **Edge for reflexes, cloud for intelligence.** Everything that decides *what* to do (speech,
> LLM, vision, memory, planning) may crash without danger. Everything that keeps the drone
> *safe* runs on the drone, on QNX, and doesn't need the laptop or the cloud.
> **The AI suggests, QNX decides.**

## 1. The devices

| Device | Where | Job | Code |
|---|---|---|---|
| **Phone / browser** | Anyone's hand | Dashboard: push-to-talk mic, live video, telemetry, command box, STOP | `cloud/static/index.html` |
| **Vultr VM** (cloud) | Internet, HTTPS | Speech-to-text, the LLM brain, text-to-speech, vision questions, visual memory, the website, flight log | `cloud/app.py`, `cloud/memory.py` |
| **Vultr Serverless Inference** | Vultr hosted models | `glm-5.3-flash` brain, `whisper-large-v3-turbo` STT, `qwen3.8-27b` vision, `qwen3-embedding-4b` memory, `chatterbox` TTS fallback | called via `station/ai.py` |
| **ElevenLabs** | External API | The Jarvis voice | `station/ai.py` |
| **Laptop: station** | Next to the flying area | Planner (actions → flight primitives), `locate()`, video and telemetry uplink, audio out | `station/` |
| **Nest Mini** | Near the operator | Bluetooth speaker for Jarvis (laptop speaker as fallback) | `station/audio.py` |
| **QNX Raspberry Pi 5: edge** | **On the drone**, power bank | Safety gate, watchdogs, cameras, optical flow, rangefinder, MAVLink to the FC | `edge/`, `tools/` |
| **Flight controller** (Matek H743, ArduPilot 4.7.2) | On the drone | Stabilisation, position estimate (EKF from flow + rangefinder), GUIDED position control, motors, its own failsafes | parameters only |
| **Down webcam** (C270) | On the drone | Optical flow: how fast the floor slides by → horizontal velocity | `edge/camgrab`, `edge/flow` |
| **Forward webcam** (C270) | On the drone | Video for the dashboard, `locate()`, memory; person detection | `edge/camgrab`, `edge/video.py` |
| **HC-SR04 + Arduino** | On the drone, pointing down | Height above the floor. **Required**: without it the EKF never gets a position and GUIDED is refused | `tools/hcsr04/hcsr04.ino`, `tools/rangefinder.py` |
| **FlySky transmitter** | Safety pilot's hands | Radio straight to the FC. SwA up = AltHold, down = **Land**. Works when everything else is dead | — |

## 2. Links

```
 PHONE ──HTTPS / WSS──► VULTR CLOUD ◄──WSS (laptop dials out)── LAPTOP STATION ──Bluetooth──► Nest Mini
  mic, video,           STT → brain → TTS                         planner, locate()
  telemetry, STOP       vision, memory DB                              │   ▲
                                              TCP :7000, JSON lines    │   │  HTTP :8080 video
                                              (protocol v1)            ▼   │  (+ X-Pose header)
                                                            QNX PI 5 on the drone
                                                                       │ UDP 127.0.0.1:14555 / :14556
                                                                       ▼
                                                            usb_relay (root) ──USB──► FLIGHT CONTROLLER ──► motors
                                                                                             ▲
                                                                       FlySky radio (Land) ──┘
```

| Link | Transport | Why this way |
|---|---|---|
| Phone ↔ Vultr | HTTPS + WSS (`/voice`, `/text`, `/ws/ui`) | Browsers only give the mic to HTTPS pages, so the VM needs a domain + TLS (Caddy) |
| Laptop ↔ Vultr | WSS `/ws/station`, **the laptop dials out**, token `STATION_TOKEN`, ping 15 s, reconnect with backoff up to 15 s | The laptop is behind NAT; the cloud can never connect in |
| Laptop ↔ Pi | TCP `:7000`, one JSON object per line (protocol v1). Heartbeat 5 Hz from its own thread | Pi has only the Python stdlib. The Pi accepts only the station's IP (`--station-ip`) |
| Pi → laptop video | HTTP `:8080/forward.mjpg` (stream), `/forward.jpg` with `X-Pose: n,e,alt,yaw,t` and `X-Pose-Age` | Each frame carries where the drone was, so a box in the picture can become a point in the room |
| Core ↔ FC | UDP `127.0.0.1:14555` ↔ `usb_relay` ↔ USB | QNX has no USB-serial driver; pyusb needs root, so root is confined to one small process |
| Sensors / hbwd → FC | UDP `127.0.0.1:14556` (relay's write-only aux inlet) | Flow, height and the C watchdog reach the FC **without** going through the Python core |
| Pi processes → core | UDP `:7001` (perception), `:7002` (flow quality / range), `:7003` (core → video: pose), `:14560` (core → hbwd: pets) | Separate processes, so killing one never stops another |
| Pi → laptop stats | UDP `:7010` (opt-in `--stats-udp`) → station → cloud | Live jitter graph and restart times on the dashboard |
| Cameras → processes | Shared memory `camgrab_fwd`, `camgrab_down` | The forward camera has two readers (video, person detector); a pipe feeds only one |

## 3. A voice command, hop by hop

Example: *"fly to the door and take a photo, then come back"*.

| # | Where | What happens |
|---|---|---|
| 1 | Phone | Hold to talk; the clip (about 3 s) goes to `POST /voice` |
| 2 | Cloud | ffmpeg → 16 kHz mono wav → Whisper → text |
| 3 | Cloud: brain | Text + latest telemetry → `glm-5.3-flash` with the Jarvis prompt and 8 tools (`takeoff / land / stop / home / move / mission / look / recall`) → actions `[mission{target:"door", action:"photo", after:"come_back"}]` + a spoken line. Dangerous request: no action, just a refusal. No key / LLM down: regex fallback (`rule_parse`) |
| 4 | Cloud → laptop | `{"type":"actions", actions, reply, req}` over the WSS. TTS runs in parallel; the mp3 goes to the phone and the laptop (Nest) |
| 5 | Laptop: planner | `scan` until `locate("door")` finds it (frame + pose → vision box → bearing + range → north/east point) → `goto` 1 m short in legs ≤ 3 m, looking again halfway → face it → photo → `return_home` |
| 6 | Laptop → Pi | One primitive at a time: `{"type":"cmd","id":..,"op":"goto","args":{"n":2,"e":1}}` |
| 7 | Pi: safety gate | Checks the command (section 6). Rejected → `ack ok=false reason=…` → Jarvis says why |
| 8 | Pi → FC | MAVLink position target via the relay |
| 9 | FC | Flies there in GUIDED, position from flow + rangefinder |
| 10 | Pi → laptop | `done` (ok / aborted / failed), telemetry 10 Hz, events. The planner only sends the next step after `done` |
| 11 | Laptop → cloud | Telemetry 5 Hz, frames ~4 fps, memory frames when the drone moved, the photo → every dashboard |

Voice-to-moving budget: upload 0.3 + STT 0.7 + LLM 0.8 + WSS 0.2 + first command 0.1 ≈ **2.1 s**
(target under 3 s).

## 4. Processes on the Pi and their priorities

All are started and watched by `tools/supervisor.py` with `tools/supervisor_pi.json`. Priority
is QNX `SCHED_FIFO`: a higher number always runs first when the CPU is busy; normal processes
(and everything on the laptop and cloud) run only when no real-time one wants the CPU.

| Prio | Process | Language | Does | Rate |
|---|---|---|---|---|
| **60** | `hbwd`: heartbeat watchdog | C, pinned to CPU 3 | Gets a "pet" from the core after every complete safety tick. Sends the FC its GCS heartbeat. **No pet for 250 ms → stops the heartbeat and, if the drone is armed in GUIDED/LAND, commands LAND (3×, 100 ms apart)**. Leaves pilot modes alone | 50 Hz |
| **50** | `usb_relay` (root) | Python | The only path to the FC: USB ↔ UDP :14555 (core) and :14556 (aux inlet) | as fast as data comes |
| **45** | `edge` core | Python | Protocol server :7000, safety gate, executor, **50 Hz safety loop** (station watchdog, battery, position-lost, pilot takeover, re-checks the running move), FC heartbeat, flight log | 50 Hz loop, 10 Hz telemetry |
| **44** | `edge.flow` | Python + OpenCV | Down camera → Lucas-Kanade flow → `OPTICAL_FLOW` straight to the FC (aux inlet); quality to the core | ~15–30 Hz |
| **43** | `camgrab_down` | C (`libcamapi`) | Down camera, 320×240 → shared memory | 30 fps |
| **42** | `rangefinder` (root) | Python | Arduino over USB → `DISTANCE_SENSOR` straight to the FC | ~18 Hz |
| **40** | `camgrab_fwd` | C | Forward camera, 640×480 → shared memory | 30 fps |
| (planned) | `edge/vision` | Python + TFLite | Person / palm detection → UDP :7001. **Not written yet** | 5–10 Hz |
| **10** | `edge.video` | Python | MJPEG + JPEG with pose on :8080 | 10–15 fps |
| normal | `supervisor` | Python | Starts everything, restarts the dead, logs restart times. Ignores SIGHUP (survives the SSH session ending) | — |
| — | laptop + cloud | — | Not real-time at all, on purpose | — |

### Why this order

Ranked by "what goes wrong if this one is starved of CPU for half a second":

1. **`hbwd` (60): the last line of defence.** It's the thing that lands the drone when
   everything above it has gone wrong, so nothing may delay it. Written in C (no garbage
   collector, no interpreter pauses), tiny, pinned to its own core. It also proves the core is
   alive: the core only pets it after a *complete* safety tick, so a hung or crash-looping core
   lands the drone within ~250 ms (SITL: 267–273 ms).
2. **`usb_relay` (50): every byte to the FC goes through it.** If it stalls, the core's
   commands, the heartbeat, flow and height all stall together. It has to sit above everything
   that uses it.
3. **`edge` core (45): the brain stem.** The safety gate and the station watchdog. It must stay
   above the sensors so that a sensor process spinning out can't stop it from landing.
4. **Position sensing (44, 43, 42): flow, down camera, rangefinder.** Without them the FC
   doesn't know where it is and drifts. They're above the forward camera because holding
   position matters more than seeing ahead.
5. **`camgrab_fwd` (40): forward camera.** Feeds the person detector (safety) and the video
   (not safety). Below position sensing, above video.
6. **`edge.video` (10): lowest on purpose.** It's for the demo and for the laptop's AI. Under
   load *this* is what stutters, not the safety loop.
7. **Laptop and cloud: not real-time.** They only suggest. Any of them can die; section 5 says
   what happens.

The QNX demo shows this: with a CPU hog running, the safety loop's jitter stays flat at FIFO
priority and spreads out at normal priority (`tools/jitter.py --compare`; macOS baseline without
FIFO: p99 5–12 ms, max up to 34 ms).

## 5. What happens when something dies

Every layer has a backstop below it. Read the table top to bottom: the further down, the more
gets lost, and the closer the backstop is to the motors.

| What dies | What happens | Who handles it | Time |
|---|---|---|---|
| A phone / the dashboard | Nothing. Other viewers, the laptop mic, `tools.cmd` still work | — | — |
| **The cloud (Vultr) or the internet** | No new voice commands; the running plan finishes. If the drone then hovers idle with no cloud for 30 s, the laptop says "I've lost the cloud, so I'm landing" and lands it | Station (`cloud_watchdog`) | 30 s |
| An AI model / key (LLM, STT, TTS) | Brain falls back to the regex parser; TTS falls back ElevenLabs → `chatterbox` → text only. Typed commands still work | Station / cloud | immediate |
| **The laptop, or its Wi-Fi** | Heartbeats stop: the edge **hovers after 1 s, lands after 5 s**. The laptop itself, if alive, treats 3 s without telemetry as a dead link, cancels the plan and reconnects | Edge watchdog (core) | 1 s / 5 s |
| `edge.video` | No video on the dashboard; `locate()` can't find targets. Flight is unaffected. Supervisor restarts it. If the *core* is gone, `X-Pose-Age` grows and the station refuses to place targets with a pose older than 0.5 s | Supervisor | ~0.1–0.2 s |
| `camgrab_fwd` | Video freezes; the person detector (when it exists) goes stale → after 1 s the gate treats "no detector" as "maybe a person": **forward moves refused**. Restarted | Gate + supervisor | ~0.2 s (measured 143–208 ms to a new frame) |
| Person detector (`edge/vision`) | Stale after 1 s → forward moves refused on the real drone (by design; `--no-person-check` overrides for the bench). Today it doesn't exist, so forward moves are always refused on the real drone | Gate | 1 s |
| `edge.flow` or `camgrab_down` | Flow quality → 0. A running move is stopped after 0.5 s of bad flow and the drone holds; new moves refused. The FC loses horizontal velocity; if its position estimate goes bad the core lands after 1 s (and ArduPilot's EKF failsafe, `FS_EKF_ACTION=1`, lands too). Restarted | Executor + gate + FC | 0.5 s / 1 s |
| `rangefinder` | The FC loses height; the EKF loses position → core lands after 1 s, FC's EKF failsafe as backup. Restarted | Core + FC | ~1 s |
| **`edge` core** (crash, hang, `slay`) | hbwd stops getting pets → **LAND within ~250 ms**. Flow and height go straight to the FC, so it lands with a position estimate, not blind. Without hbwd, the FC's own GCS failsafe lands after 2 s. Supervisor restarts the core (measured ~4 ms respawn, ~120 ms serving) | hbwd, then FC | 0.25 s / 2 s |
| One exception inside the safety loop | Logged, the loop keeps running. If every tick keeps failing, hbwd gets no pets → LAND; without hbwd the core lands itself after 1 s of failures | Core + hbwd | 0.25–1 s |
| `hbwd` | Lose the fast backstop. The core still heartbeats the FC, so normal flight continues. Supervisor restarts it | Supervisor | ms |
| **`usb_relay`** | The FC hears nothing from the Pi: no heartbeat, no flow, no height → FC GCS failsafe lands after 2 s, EKF failsafe too. Supervisor restarts it and the core re-sends speed limits and the EKF origin when the link comes back | FC failsafes + supervisor | 2 s |
| `supervisor` | Children keep running but nothing restarts them any more | — | — |
| **The whole Pi** (power, crash) | Same as the relay: FC GCS failsafe → LAND | ArduPilot (`FS_GCS_ENABLE=5`, `FS_GCS_TIMEOUT=2`) | 2 s |
| Battery low | Below 10.6 V: new moves refused. Below 10.2 V for 2 s: auto-land (2 s so throttle sag doesn't end a flight) | Gate / core; ArduPilot battery failsafe as backup | 2 s |
| **Everything** | Safety pilot flips **SwA down = Land** | FlySky → receiver → FC | ~0.2 s |

### The pilot always wins

If the pilot flips SwA (AltHold or Land), the edge sees a mode it didn't set and **stands
down**: it forgets its targets, refuses every command (even `land` and `stop` from the
dashboard), never switches back to GUIDED, and hbwd leaves pilot modes alone. Control comes back
only when the station explicitly takes off again. ArduPilot's own failsafes still apply.

## 6. The safety gate

Every command from the station passes `edge/safety.py`; there is no other path to the vehicle.
The core also re-checks the *running* move every tick and stops it if a rule starts failing.
Limits live in `edge/config.py`:

| Rule | Limit |
|---|---|
| Altitude window | Floor 0.3 m; **no ceiling** (outdoors, 2026-11-21; `alt_max` to restore) |
| Geofence | **Off** (outdoors). `fence_half` restores ±N m around the first takeoff of this edge run |
| Single move | ≤ 3 m |
| Speed | 0.5 m/s horizontal, 0.3 m/s vertical, 45°/s yaw, scan ≤ 30°/s |
| Battery | Moves refused below 10.6 V; auto-land below 10.2 V |
| Optical flow quality | Moves refused below 50 (0–255) |
| Person ahead | Forward moves refused if a person is closer than 1.0 m, or if no detector is running (real drone) |
| State | Moves need armed + GUIDED + a healthy position estimate. `stop`, `hover`, `land` always accepted unless the pilot has another mode. Landing beats stop |

On top of the gate: one task at a time, a new command cancels the running one, `stop` always
wins. The LLM refuses dumb requests *first* ("crash into a wall? absolutely not"); the gate is
what makes that a guarantee rather than a hope.

## 7. Where control can come from

| Source | Can do | Guarded by |
|---|---|---|
| Pilot PIN holder on the dashboard | Everything (voice, typed, LAND) | `CONTROL_PIN`; only one viewer holds the controls at a time |
| Any dashboard viewer | **STOP** only | open to all on purpose |
| Laptop (`--local`, `tools.cmd`) | Everything | on the laptop; the Pi accepts only the station IP |
| FlySky transmitter | Take over or land, always | radio, no software in the way |

## 8. Network setup

The Pi and the laptop must be on **one Wi-Fi where they can reach each other**: a travel router,
or the venue Wi-Fi if it has no client isolation and no login page. An iPhone hotspot is last
resort (client-to-client traffic isn't guaranteed); use fixed IPs, not mDNS. The laptop reaches
Vultr through the same Wi-Fi; phones reach Vultr over any internet.

## 9. Open questions

1. A second camera QNX can drive (the `081b` webcam fails on QNX; needs a second C270 or C920).
2. Real C270 field of view (now assumed 45° horizontal) and `locate()` on real frames.
3. Check on the bench that the FC still receives `OPTICAL_FLOW` and `DISTANCE_SENSOR` with the
   core stopped (the direct sensor path).
