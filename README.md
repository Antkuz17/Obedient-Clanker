# Hope Drone Don't Go Kaboom

A voice-controlled drone with a personality. You talk to it like Jarvis ("rise and do a 360",
"go two metres to the left and take a photo", "land") and it flies, answers back, and refuses
anything unsafe. Built at Hack Western 13 (Nov 20–22, 2026).

> Edge for reflexes, cloud for intelligence. Vultr thinks; QNX keeps it alive.

Tracks: **Best Use of QNX** and **Best Use of Vultr**.

## Why

The people who most need a drone are the ones who can't fly one. A firefighter at a burning
building can see exactly what's going on, but their hands are on a hose, a ladder or a person.
When they need eyes on the second floor, they shouldn't have to put everything down and pick up
a controller. They should be able to say "rise, look at the second-floor window, take a photo"
and keep working.

That's the idea: **a drone you command by voice, for people whose hands are busy and whose
situation is critical.** The person on the ground has the best line of sight and the best
judgment about what matters; the drone becomes an extra pair of eyes they can direct without
breaking focus. The same applies to search and rescue, paramedics at a crash scene, utility
crews checking a line after a storm, or anyone inspecting something they can't climb to.

Because it's meant for those moments, it can't be a toy: it has to refuse unsafe orders, keep
working when the network doesn't, and never fly into a person, no matter what it's told.

## What it does

- **Talk to it.** Hold a button (or Space), say what you want, let go. Jarvis plans it, says
  what it's doing in a dry, sarcastic voice, and the drone flies it.
- **Simple flying in plain English:** take off, land, stop, move by distance and direction,
  turn or spin all the way around, chain a few steps in one sentence.
- **Eyes:** it takes photos on request and can read, describe or solve what the camera sees.
- **Listens mid-flight:** "land" or "stop" interrupts whatever it's doing.
- **Safety first, always:** every command is checked on the drone before the motors move. It
  refuses to fly toward a person, above its ceiling, outside its limits or on a weak battery,
  and it lands itself if it loses height, loses its sensors or loses the link.
- **A live dashboard** shows the camera with people boxed in red, what you said, what Jarvis
  planned, what the safety gate decided, and the drone's height, battery and position.

## How it works

```
  dashboard (browser, mic)
        │ voice / text
        ▼
  VULTR (cloud)
   • speech-to-text: whisper-large-v3-turbo
   • brain: glm-5.3-flash (tool calls) turns a sentence into flight steps
   • vision: qwen3.8-27b reads, describes and captions photos
   • dashboard backend, live video and telemetry relay
        │ plan
        ▼
  laptop station
   • planner: flight steps -> safe primitives (takeoff, move, turn, land)
   • rangefinder height hold, sink / drift / link guards
        │ protocol v1 (TCP)
        ▼
  QNX on a Raspberry Pi 5, riding on the drone
   • safety gate: every primitive checked (ceiling, fence, battery, person distance)
   • on-board person detector (TensorFlow Lite) gates forward motion
   • optical flow (OpenCV) + ultrasonic rangefinder feed the flight controller
   • C watchdog: no heartbeat -> hold, then land
   • supervisor restarts any crashed process
        │ MAVLink over USB
        ▼
  Matek H743 / ArduPilot (GUIDED) ──► F450 frame
```

The cloud is a convenience layer. Everything that keeps the drone safe runs on the drone: with
the cloud or the laptop gone, it still holds, refuses unsafe moves and lands.

## QNX on the drone

The Pi 5 runs QNX 8 and is the drone's flight computer. Both cameras, optical flow, the person
detector, the safety gate, the watchdog and the link to the flight controller all run on it.

- **QNX open-source AI:** TensorFlow Lite (QNX port) runs SSD-MobileNet person detection on
  board, and OpenCV (QNX port) runs the optical flow.
- **Real-time priorities:** the watchdog, flight-controller link and safety core run with
  SCHED_FIFO priority above QNX's USB and Wi-Fi drivers; cameras, flow and AI run below them,
  so heavy compute can never starve the safety loop.
- **Measured on the drone:** the 50 Hz watchdog loop's worst wake-up is under 1 ms. A killed
  detector respawns in about 73 ms. A frozen safety core is caught in about 260 ms while the
  flight controller keeps getting its sensor data.
- **Observability:** every start, crash, restart and safety refusal goes to QNX's system log.

More detail: [docs/qnx.md](docs/qnx.md).

## Vultr in the loop

Speech-to-text, the language model that plans each command, and the vision model all run on
Vultr Serverless Inference (OpenAI-compatible API). The brain answers in about a second;
simple flying commands skip the model entirely for speed. ElevenLabs provides the Jarvis voice.

More detail: [docs/models.md](docs/models.md).

## Positioning

- **Height:** a downward ultrasonic rangefinder. The flight controller's own height estimate
  drifts near the ground, so takeoff and hover are decided by the rangefinder.
- **Indoors:** optical flow from a down-facing camera.
- **Outdoors:** GPS for horizontal position, rangefinder for height.

## Hardware

F450 frame, Matek H743 flight controller (ArduPilot 4.7), QNX Raspberry Pi 5, two Logitech C270
cameras (forward and down), HC-SR04 ultrasonic rangefinders, HGLRC M100-PRO GPS, 3S LiPo,
FlySky transmitter. Wiring and setup: [docs/hardware.md](docs/hardware.md),
[docs/flight-setup.md](docs/flight-setup.md).

## Innovation & creativity

- **Voice as the whole interface.** No joystick, no app full of buttons: you say what you want
  in plain English and the drone works out the flight. Simple commands ("rise and do a 360")
  are understood instantly on the laptop; anything open-ended goes to a language model on Vultr
  that turns it into flight steps.
- **An AI that's allowed to say no.** Jarvis has a personality and refuses dumb or dangerous
  orders out loud, with the reason, and a hard safety layer on the drone backs that up in code.
  The cloud AI proposes; the drone disposes.
- **Edge for reflexes, cloud for intelligence.** The expensive thinking (speech, planning,
  vision) lives on Vultr; everything that keeps people safe lives on the drone itself, on a
  real-time OS, and keeps working with the cloud or the laptop gone.

## Technical complexity

- **A real flight computer on a real-time OS.** A Raspberry Pi 5 running QNX 8 rides on the
  drone and talks MAVLink to an ArduPilot flight controller over USB, with a hand-written USB
  link because QNX has no serial driver for it.
- **On-board AI that gates flight.** TensorFlow Lite person detection runs on the drone; its
  output decides whether a forward move is allowed, and stops one that's already running.
- **Real-time engineering:** a SCHED_FIFO priority plan so AI work can never starve the safety
  loop or QNX's own drivers; a C watchdog with a sub-millisecond worst case; a supervisor that
  restarts any crashed process in tens of milliseconds.
- **Sensor fusion that survives the real world:** optical flow indoors, GPS outdoors, and an
  ultrasonic rangefinder for height, because the flight controller's own height estimate
  drifts near the ground.
- **A full stack in two days:** cloud app, speech-to-text, LLM tool calling, vision, a live
  dashboard, a planner, a wire protocol, a safety gate and the embedded side, all integrated.

## Completeness

Working on the real drone, by voice, from the dashboard:

- takeoff to a hover held by the rangefinder, then landing on command
- a full 360° turn, then hovering until told to land, including "land" said mid-flight
- moving sideways by a given distance, taking a photo on the way
- refusing to fly forward when a person is in front of the camera, and boxing them on screen
- outdoor flight on GPS; refusals for height, battery and distance limits; automatic landing
  when height, sensors or the link are lost

Known limits:

- The airframe is heavy for its 3S battery (about 80% throttle to hover), so sideways flight
  needs a well-charged pack; on a tired one the drone sinks and lands itself.
- The forward ultrasonic obstacle sensor isn't wired yet, so only people (not objects) are
  detected ahead.
- Hand-gesture control (palm to stop) is designed but not implemented.

## Design

- **One screen:** the live camera with people boxed in red; a pipeline that reads left to right
  (You said → Jarvis plans → QNX decides → Motors) with animated links that show where a command
  is and stop with a ✕ where the safety gate refused it; and a side panel with the decision, the
  real-time stats from QNX, height against the ceiling, and battery.
- **Hold to talk** (button or Space), **Land** and a big **STOP** (also Esc) that never asks for
  anything. A problem banner appears only when something is wrong: link lost, low battery,
  pilot override.
- **Readable at a glance:** one accent colour, status colours only for state (amber for refused,
  green for approved), no clutter. Works on a laptop and a phone.

## Repository

| Folder | What's in it |
|---|---|
| `edge/` | Runs on QNX on the drone: safety gate, executor, watchdog, flight-controller link, cameras, optical flow, person detector |
| `shared/` | The protocol between the laptop and the drone |
| `station/` | Laptop: brain, planner, voice in and out, vision tasks |
| `cloud/` | Vultr app and the dashboard |
| `docs/` | Design, hardware, QNX and model notes |

## Safety

The props are 10" with no guards. Fly in a cleared area with people behind the drone and a
pilot on the transmitter with the land switch ready. The pilot always overrides the computer.
