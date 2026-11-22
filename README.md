# Hope Drone Don't Go Kaboom

A voice-controlled drone with a personality. You talk to it like Jarvis ("rise and do a 360",
"go two metres to the left and take a photo", "land") and it flies, answers back, and refuses
anything unsafe. Built at Hack Western 13 (Nov 20–22, 2026).

> Edge for reflexes, cloud for intelligence. Vultr thinks; QNX keeps it alive.

Tracks: **Best Use of QNX** and **Best Use of Vultr**.

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
