# Hope Drone Don't Go Kaboom

A voice-controlled drone with a personality. You talk to it like Jarvis ("rise and do a 360",
"go two metres to the left and take a photo", "land") and it flies, answers back, and refuses
anything unsafe. Built at Hack Western 13 (Nov 20–22, 2026).

> Edge for reflexes, cloud for intelligence. Vultr thinks; QNX keeps it alive.

Tracks: **Best Use of QNX** and **Best Use of Vultr**.

---

## Contents

1. [Why](#why)
2. [What it does](#what-it-does)
3. [How a voice command becomes flight](#how-a-voice-command-becomes-flight)
4. [Architecture](#architecture)
5. [The drone](#the-drone)
6. [QNX: the drone's real-time brain](#qnx-the-drones-real-time-brain)
7. [Safety, in layers](#safety-in-layers)
8. [Vultr: the cloud brain](#vultr-the-cloud-brain)
9. [The dashboard](#the-dashboard)
10. [Judging criteria](#judging-criteria)
11. [Challenges we ran into](#challenges-we-ran-into)
12. [Accomplishments, lessons, what's next](#accomplishments-lessons-whats-next)
13. [Tech stack](#tech-stack)
14. [Repository](#repository)

---

## Why

The people who most need a drone are the ones who can't fly one. A firefighter at a burning
building can see exactly what's going on, but their hands are on a hose, a ladder or a person.
When they need eyes on the second floor, they shouldn't have to put everything down and pick up
a controller. They should be able to say "rise, look at the second-floor window, take a photo"
and keep working.

That's the idea: **a drone you command by voice, for people whose hands are busy and whose
situation is critical.** The person on the ground has the best line of sight and the best
judgment about what matters; the drone becomes an extra pair of eyes they can direct without
breaking focus.

Where this fits:

- **Firefighting:** check an upper floor, a roof or the back of a building without leaving the
  hose line.
- **Search and rescue:** look over a ridge, into a ravine or across rubble while your hands are
  on ropes or a stretcher.
- **Paramedics and crash scenes:** a view from above of a pile-up while treating a patient.
- **Utility and infrastructure crews:** inspect a line, a tower or a roof after a storm without
  climbing it.
- **Anyone with full hands:** construction, warehouses, farms, inspections.

Because it's meant for those moments, it can't be a toy. It has to refuse unsafe orders, keep
working when the network doesn't, and never fly into a person, no matter what it's told. That
requirement is why half of this project is a real-time safety system running on QNX.

---

## What it does

- **Talk to it.** Hold a button (or Space), say what you want, let go. Jarvis plans it, says
  what it's doing in a dry, sarcastic voice, and the drone flies it.
- **Plain-English flying:** take off, land, stop, move by a distance in any direction, go up or
  down, turn by an angle or spin all the way around, and chain several steps in one sentence
  ("rise, go two metres to the left, take a photo, then come back").
- **Eyes:** it takes photos on request and can read text, describe a scene or solve a problem
  the camera can see.
- **Listens mid-flight:** "land" or "stop" interrupts whatever it's doing, even mid-turn.
- **Refuses, out loud, with a reason:** too high, too close to a person, battery too low,
  something that doesn't make sense. Jarvis says why; the drone doesn't move.
- **Lands itself** when it loses its height sensor, starts sinking toward the ground, turns on
  its own, loses the link to the laptop, or the battery is critical.
- **A live dashboard** shows the camera with people boxed in red, what you said, what Jarvis
  planned, what the safety gate decided, and the drone's height, battery and position.

---

## How a voice command becomes flight

What happens when you say "Hey drone, rise and do a 360":

| Step | Where | What happens | Typical time |
|---|---|---|---|
| 1. Listen | Browser (dashboard) | Hold to talk; audio streams to the cloud while you speak | as long as you talk |
| 2. Transcribe | **Vultr**: `whisper-large-v3-turbo` | Speech becomes text | ~1.0–1.6 s after you let go |
| 3. Understand | Cloud app | Common commands are parsed instantly by rules; anything open-ended goes to the LLM | < 10 ms (rules) |
| 3b. Plan with AI | **Vultr**: `glm-5.3-flash` with tool calls | The model turns a sentence into structured steps (`takeoff`, `move`, `look`, `land`…) and writes Jarvis's reply | ~0.5–1.2 s |
| 4. Plan the flight | Laptop station | The planner turns steps into primitives the drone understands (`takeoff`, `move_body`, `scan`, `land`), splits long moves, and holds height from the rangefinder | ms |
| 5. Gate | **QNX** on the drone | The safety gate checks every primitive against altitude, fence, battery, person distance and sensor health, then accepts or refuses it with a reason | ms |
| 6. Fly | ArduPilot flight controller | The accepted primitive becomes a MAVLink target in GUIDED mode | real time |
| 7. Report back | Dashboard | Telemetry, the gate's verdict, Jarvis's spoken reply (ElevenLabs), the live camera | 10 Hz |

From letting go of the button to the drone reacting is about 1.5–2 seconds, almost all of it
speech-to-text. "Land" and "stop" skip the AI entirely and pre-empt anything in progress.

---

## Architecture

```
  DASHBOARD (browser: mic, live video, controls)
        │  voice / text            ▲ video, telemetry, verdicts, Jarvis's voice
        ▼                          │
  VULTR CLOUD APP (FastAPI)
   • speech-to-text ......... whisper-large-v3-turbo   (Vultr Serverless Inference)
   • brain .................. glm-5.3-flash, tool calls (Vultr Serverless Inference)
   • vision ................. qwen3.8-27b: read / describe / caption (Vultr)
   • voice .................. ElevenLabs text-to-speech
   • relays video + telemetry to every viewer; one person "has the controls", anyone can STOP
        │  plan (steps)            ▲ frames, telemetry, events
        ▼                          │
  LAPTOP STATION
   • planner: steps -> primitives, legs, turns, photos
   • rangefinder height hold + guards (sinking, uncommanded yaw, sensor loss, link loss)
        │  protocol v1 over TCP: cmd / ack / done / telemetry / event + 5 Hz heartbeat
        ▼
  QNX 8 on a RASPBERRY PI 5, riding on the drone ("the edge")
   • safety gate + executor (every primitive checked; refusals explained)
   • on-board person detector: TensorFlow Lite SSD-MobileNet (QNX port)
   • optical flow: OpenCV (QNX port) on the down camera
   • ultrasonic rangefinder (height), GPS outdoors
   • C watchdog (SCHED_FIFO 60), supervisor restarts, QNX system log
        │  MAVLink over USB (libusb; QNX has no driver for the flight controller)
        ▼
  MATEK H743 flight controller, ArduPilot 4.7, GUIDED mode ──► F450 quad
        ▲
        └── pilot's FlySky transmitter: one switch takes over and lands, always
```

**Design rule:** the cloud is a convenience layer. Everything that keeps people safe runs on
the drone. Kill the cloud, kill the laptop or kill a process, and the drone still refuses unsafe
moves, holds, and lands.

**The contract between laptop and drone** is a small line-based JSON protocol ("protocol v1").
The laptop sends commands (`takeoff`, `move_body`, `goto`, `yaw`, `scan`, `hover`, `stop`,
`return_home`, `land`); the drone answers each with an `ack` (accepted or refused, with a
reason), later a `done` (ok / aborted / failed), streams telemetry at 10 Hz and pushes events
(safety refusals, watchdog, person too close, battery). `stop`, `land` and `hover` are always
allowed.

---

## The drone

**Hardware**

| Part | What |
|---|---|
| Frame | F450 quad, 10" props |
| Flight controller | Matek H743, ArduPilot 4.7.2, GUIDED mode for computer control |
| Companion computer | Raspberry Pi 5 (4× Cortex-A76, 8 GB) running **QNX 8**, mounted on the drone |
| Cameras | 2× Logitech C270: one forward (person detection, video, photos), one down (optical flow) |
| Height | HC-SR04 ultrasonic rangefinder pointing down (via a small Arduino) |
| Outdoors | HGLRC M100-PRO GPS with compass |
| Power | 3S LiPo |
| Pilot override | FlySky transmitter: switch up = normal, switch down = land now |

**How it knows where it is**

- **Height:** the downward ultrasonic rangefinder. The flight controller's own height estimate
  drifts while the drone sits on the ground with the motors spinning (and GPS vertical speed
  made it worse), so takeoff, hover and "am I sinking?" are all decided by the rangefinder.
- **Indoors:** optical flow from the down camera, computed on QNX with OpenCV and fed to
  ArduPilot as an optical-flow sensor.
- **Outdoors:** GPS for horizontal position; height still from the rangefinder.

**Flight controller failsafes** (on the flight controller itself, independent of everything
above): no heartbeat from the companion computer for 2 s → land; position estimate fails →
land; battery failsafe judged on sag-compensated voltage.

---

## QNX: the drone's real-time brain

The Pi 5 runs QNX 8 and **is the drone's flight computer**. Everything the drone does in flight
runs on it: both cameras (through QNX's camera API, in C), optical flow, the person detector,
the safety gate, the watchdog and the MAVLink link to the flight controller.

### Why QNX

A drone that flies near people has to react on time, every time. The safety loop and the
watchdog must never be late because the AI is busy. QNX gives us hard real-time scheduling
(SCHED_FIFO priorities that are actually honoured), process isolation (a crashing AI process
can't take the safety core down) and a microkernel where drivers are ordinary processes we can
prioritise around. That's exactly the shape of this problem: heavy AI and critical control on
one small computer.

### QNX open-source AI modules we use

- **TensorFlow Lite** (QNX port, `python3-tflite-runtime 2.21`): runs an SSD-MobileNet person
  detector on the forward camera, on board. About 7.5 detections per second, 84–87 ms each.
  Its output gates flight: a forward move is refused if it would end within 1 m of a person
  (assuming the distance estimate may be 30% too far), and a move already running is stopped.
- **OpenCV** (QNX port, `python3-opencv 4.12`): runs optical flow on the down camera, which
  feeds the flight controller's position hold indoors.

### Every process, and its real-time priority

| Process | What it does | Priority |
|---|---|---|
| `hbwd` | C watchdog: lands the drone if the safety core stops responding | **FIFO 60** |
| `usb_relay` | MAVLink to and from the flight controller over USB (libusb) | **FIFO 50** |
| `edge` | Safety core: protocol server, safety gate, executor, battery and link monitors | **FIFO 45** |
| `rangefinder` | Ultrasonic height to the flight controller | **FIFO 42** |
| *QNX USB / Wi-Fi drivers* | *the OS's own drivers* | *21* |
| `flow` | Optical flow (OpenCV) | 20 |
| `camgrab_down` / `camgrab_fwd` | C camera capture into shared memory (QNX camera API) | 19 / 18 |
| `vision` | Person detector (TensorFlow Lite) | 11 |
| `video` | Video stream to the laptop | 10 |
| `blackbox`, `camrec` | Flight recorder: telemetry and camera frames | 5 |

The plan: safety-critical work runs **above** QNX's USB and Wi-Fi drivers; cameras, flow and AI
run **below** them, so no amount of compute can starve a driver or the safety loop. A
supervisor starts everything, restarts anything that dies, and keeps QNX's camera service
alive.

### Measured on the drone

| What | Result |
|---|---|
| C watchdog loop, 50 Hz, FIFO 60 | worst wake-up 0.27–0.98 ms, zero late cycles |
| Same loop, normal priority vs FIFO | p99 14.0 ms vs 1.2 ms |
| Watchdog under SD-card stress, before and after our fix | 433 ms stall → **0.86 ms**, zero late, zero log lines dropped |
| Person detector killed (SIGKILL) | respawned in **73 ms** |
| Safety core frozen for 3 s | watchdog noticed in **260 ms**; flow and rangefinder kept reaching the flight controller |
| QNX camera service killed | back in **3.6 s**, cameras streaming in 4.6 s |

### Observability

Every start, crash, restart and safety refusal is written to QNX's system log (slog2, buffer
set `hope`), so the drone's own record of what happened survives everything else.

More detail: [docs/qnx.md](docs/qnx.md).

---

## Safety, in layers

No single layer is trusted. A bad command has to get past all of these:

1. **Jarvis (the AI)** refuses dumb or dangerous requests out loud.
2. **The planner (laptop)** caps sideways moves, keeps height within what the rangefinder can
   measure, and refuses climbs above it.
3. **The safety gate (QNX, on the drone)** checks every primitive: minimum and maximum
   altitude, fence, battery under load (refuse new moves below 10.0 V, land below 9.6 V),
   optical-flow quality, and distance to people ahead. Refusals come back with a reason that
   Jarvis reads out and the dashboard shows.
4. **The person detector (QNX)** stops a forward move already in progress if someone gets too
   close, and with the detector down, forward moves can be refused entirely.
5. **Guards in flight (laptop):** land if the rangefinder is lost for 1 s, if the drone sinks to
   within 10 cm of the ground, if it turns more than 45° on its own, or reads above its limit.
6. **The watchdog (QNX):** no heartbeat from the laptop for 1 s → hold position; 5 s → land.
   The C watchdog lands the drone if the safety core itself hangs.
7. **Flight controller failsafes:** companion computer silent for 2 s or position estimate
   lost → land; battery failsafe on sag-compensated voltage.
8. **The pilot:** one switch on the transmitter takes over and lands. The computer never
   fights the pilot.

---

## Vultr: the cloud brain

| Job | Model on Vultr Serverless Inference | Notes |
|---|---|---|
| Speech-to-text | `whisper-large-v3-turbo` | Streamed while you hold the button; text ready ~1 s after you let go |
| Brain | `glm-5.3-flash` with tool calling | Chosen after measuring candidates: fastest, correct tool calls, witty refusals with no tool call |
| Backup brain | `qwen3.8-flash-next` | Also verified |
| Vision | `qwen3.8-27b` | Read text, describe a scene, solve a problem, caption photos |

- **OpenAI-compatible API**, so the cloud app calls Vultr like any chat endpoint, with tools
  (`takeoff`, `move`, `mission`, `look`, `land`, `stop`, `home`…).
- **Speed where it matters:** common flying commands never wait for the LLM; they're parsed by
  rules in milliseconds. The LLM handles everything open-ended and the personality.
- **The dashboard backend** also runs as the cloud app: it relays the drone's video and
  telemetry to every viewer, keeps a flight log, and lets one person hold the controls while
  anyone watching can still hit STOP.
- **Voice:** ElevenLabs text-to-speech for the Jarvis voice, played in the browser.

More detail: [docs/models.md](docs/models.md).

---

## The dashboard

- **One screen.** The live forward camera, with a red box and "Person detected · 2.5 m"
  around anyone the drone's detector sees.
- **The pipeline, left to right:** *You said* → *Jarvis plans* → *QNX decides* → *Motors*, with
  animated links that show where a command is. A refused command stops at the QNX node with an
  amber "Refused" and the reason, and the link to the motors is cut with a ✕.
- **Side panel:** the safety gate's decision, how many commands were checked and refused,
  real-time stats from QNX (loop lateness, process restarts), height against the ceiling, and
  battery.
- **Controls:** **Hold to talk** (button or Space), **Type instead**, **Land**, and a big
  **STOP** (also Esc) that never asks for anything.
- **Problem banners only when something's wrong:** link lost, drone not connected, battery
  low or critical, pilot has the transmitter.
- **Photos** flash full-screen for a moment when the drone takes one.
- Works on a laptop and on a phone.

---

## Judging criteria

### Innovation & creativity

- **Voice as the whole interface** for people whose hands are busy: no joystick, no app full of
  buttons. Say what you want in plain English; the drone works out the flight.
- **An AI that's allowed to say no.** Jarvis has a personality and refuses dumb or dangerous
  orders out loud, with the reason, and a hard safety layer on the drone backs that up in code.
  The cloud AI proposes; the drone disposes.
- **Edge for reflexes, cloud for intelligence.** The expensive thinking (speech, planning,
  vision) lives on Vultr; everything that keeps people safe lives on the drone, on a real-time
  OS, and keeps working with the cloud or the laptop gone.

### Technical complexity

- **A real flight computer on a real-time OS:** QNX 8 on a Pi 5 riding on the drone, talking
  MAVLink to ArduPilot over a hand-written USB link (QNX has no driver for the flight
  controller).
- **On-board AI that gates flight:** TensorFlow Lite person detection whose output decides
  whether the drone may move.
- **Real-time engineering:** a SCHED_FIFO priority plan around QNX's own drivers, a C watchdog
  with a sub-millisecond worst case, a lock-free logging fix that took a 433 ms stall to under
  1 ms, and process restarts in tens of milliseconds.
- **Sensor fusion that survives the real world:** optical flow indoors, GPS outdoors, and an
  ultrasonic rangefinder for height.
- **A full stack in two days:** cloud app, speech-to-text, LLM tool calling, vision, a live
  dashboard, a planner, a wire protocol, a safety gate and the embedded side, all integrated
  and flown.

### Completeness

Working on the real drone, by voice, from the dashboard:

- takeoff to a hover held by the rangefinder, then landing on command
- a full 360° turn, then hovering until told to land, including "land" said mid-flight
- moving sideways by a given distance, taking a photo on the way, and coming back
- refusing to fly forward when a person is in front of the camera, and boxing them on screen
- outdoor flight on GPS; refusals for height, battery and distance limits; automatic landing
  when height, sensors or the link are lost

Known limits:

- The airframe is heavy for its 3S battery (about 80% throttle to hover), so sideways flight
  needs a well-charged pack; on a tired one the drone sinks and lands itself.
- The forward ultrasonic obstacle sensor isn't wired yet, so only people (not objects) are
  detected ahead.
- Hand-gesture control (palm to stop) is designed but not implemented.

### Design

- One screen that reads left to right, from what you said to what the motors did.
- One accent colour; status colours only for state (amber = refused, green = approved,
  rose = STOP). Nothing on screen unless it means something.
- The most important control, STOP, is the biggest button, works from the keyboard (Esc) and
  never asks for confirmation.

---

## Challenges we ran into

- **QNX had no driver for our flight controller's USB port.** We talk to it directly over
  libusb from a process running as root, which gave us two-way MAVLink with no soldering.
- **AI starving the operating system.** With flow, vision and cameras at high real-time
  priority, they starved QNX's own USB and Wi-Fi drivers (USB ring overruns, the camera service
  crashed). The fix was a priority plan with safety above the drivers and compute below them.
- **A 433 ms stall in the watchdog.** It printed its log to the SD card, and a busy card blocked
  the real-time loop. We moved logging to a lock-free ring buffer drained by a low-priority
  thread: 0.86 ms worst case under the same stress.
- **The flight controller's height estimate lies near the ground.** Motors spinning on the
  ground made it think the drone was climbing, so normal takeoffs stopped before lift-off. We
  made the rangefinder the source of truth for takeoff and hover.
- **GPS made height worse.** Outdoors, GPS vertical speed reported the drone climbing at
  0.3–0.4 m/s while it sat on the ground; the flight controller cut the throttle. Height now
  comes from the rangefinder only; GPS does horizontal position.
- **The GPS wouldn't let us arm.** The module rejected one of ArduPilot's configuration
  messages every 30 seconds, which blocked arming. We let the module use its own output.
- **Battery sag.** The drone needs about 80% throttle to hover; a 3S pack drops nearly 2 V under
  that load, which tripped our own battery limits mid-flight. We re-tuned them for voltage
  under load.
- **A phone hotspot that gave the laptop no IPv4 address.** The drone still answered over IPv6,
  so we carried the flight link through an SSH tunnel on its IPv6 link-local address.
- **Natural speech.** People say "I want you to rise, turn around, and land", and the LLM
  sometimes dropped steps. Common flying phrases are now parsed directly, matching how
  Whisper actually transcribes them (punctuation and all).

---

## Accomplishments, lessons, what's next

**Proud of**

- A drone that takes off, turns, moves, takes photos and lands from plain speech, and refuses
  when it should.
- Real-time numbers measured on the drone itself, not on a desk.
- A safety design where no single layer, including the AI, is trusted.

**What we learned**

- Real-time isn't just a high priority: if your compute outranks the OS's own drivers, you
  break the OS. Plan priorities around the drivers.
- Never block a real-time loop on I/O, not even a log line.
- Sensors disagree in the field in ways they never do in a simulator; pick one source of truth
  per quantity and guard it.
- For voice, latency is mostly speech-to-text, and the fastest LLM call is the one you don't
  make.

**What's next**

- A lighter airframe or bigger battery, so it can fly longer and further.
- Wire up the forward obstacle sensor, so walls and objects are refused like people are.
- Palm-to-stop and point-to-go hand gestures on QNX.
- Missions to named things ("fly to the red door and read the sign") and visual memory ("where
  did I leave my keys?") from the existing planner and cloud pieces.
- Hardened networking for the field: the drone and the laptop on a dedicated link.

---

## Tech stack

- **Drone OS:** QNX 8 on Raspberry Pi 5
- **On-board AI:** TensorFlow Lite (SSD-MobileNet person detection), OpenCV optical flow (QNX ports)
- **Flight:** ArduPilot 4.7 on a Matek H743, MAVLink (pymavlink), libusb
- **Real-time:** SCHED_FIFO priorities, C watchdog, process supervisor, QNX slog2
- **Cloud:** Vultr Serverless Inference (Whisper, GLM, Qwen vision), FastAPI, WebSockets
- **Voice:** Whisper speech-to-text, ElevenLabs text-to-speech
- **Laptop station:** Python, asyncio
- **Dashboard:** HTML, CSS, vanilla JavaScript, single file, Geist type
- **Languages:** Python, C, JavaScript

---

## Repository

| Folder | What's in it |
|---|---|
| `edge/` | Runs on QNX on the drone: safety gate, executor, watchdog, flight-controller link, cameras, optical flow, person detector |
| `shared/` | The protocol between the laptop and the drone |
| `station/` | Laptop: brain, planner, voice in and out, vision tasks |
| `cloud/` | Vultr app and the dashboard |
| `docs/` | Design, hardware, QNX and model notes |

## Flying safely

The props are 10" with no guards. Fly in a cleared area with people behind the drone and a
pilot on the transmitter with the land switch ready. The pilot always overrides the computer.
