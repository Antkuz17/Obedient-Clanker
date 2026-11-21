# End-to-end flow

Every hop from your voice to the motors and back: which device, which link, what's sent, how
often, and what happens when it breaks. The contracts themselves are in [PRD.md](PRD.md)
section 5. This doc is the wiring around them.

## Devices and networks

| Device | Where | Runs |
|---|---|---|
| **Phone** (any judge's or ours) | In hand | Dashboard in the browser: push-to-talk mic, live video, telemetry, command box |
| **Vultr** (cloud) | Internet, `https://<domain>` | Web app + API: speech-to-text, LLM brain, text-to-speech, VLM, visual memory, video relay |
| **Laptop: station** | Near the flight area | `station/`: link to the cloud, planner, localization, video relay, audio out to the Nest |
| **Google Nest Mini** | Near the operator | Speaker only, paired to the laptop over Bluetooth |
| **QNX Pi 5: edge** | On the drone, on a power bank | Camera grabbers, optical flow, person detection, safety gate, watchdog, MAVLink |
| **Flight controller** | On the drone | ArduPilot 4.7.2: GUIDED mode, EKF (flow + baro), motor control |
| **FlySky transmitter** | Safety pilot's hands | Direct RF to the receiver: SwA = Land, works if everything else is dead |

**Network:** the Pi and the laptop are on **one Wi-Fi we control** (a phone hotspot or a
travel router), not the venue Wi-Fi. Venue networks often block devices from talking to each
other and need a login page. The laptop reaches Vultr through that same Wi-Fi. Phones reach
Vultr over any internet.

```
 PHONE ──HTTPS/WSS──► VULTR ◄──WSS (laptop dials out)── LAPTOP STATION ──TCP :7000──► QNX PI ──USB──► FC ──► motors
   ▲  mic, video, telemetry      STT·LLM·TTS·VLM·memory      │  ▲   ▲  ◄─ HTTP :8080 MJPEG ─┘   ▲
   └───────── reply audio/text ◄─────────────┘                │  │   │                          │
                                                Bluetooth ─────┘  │   └ laptop mic (fallback)   │
                                                  NEST MINI ◄─────┘                  FlySky RF ─┘ (Land)
```

## 1. A voice command, step by step

Example: *"fly to the door and take a photo"*.

| # | From → To | Link | Payload | Notes |
|---|---|---|---|---|
| 1 | You → phone | Mic | Speech while holding the button | The browser's `getUserMedia` needs **HTTPS**, so Vultr needs a domain + TLS (GoDaddy domain + Let's Encrypt) |
| 2 | Phone → Vultr `POST /voice` | HTTPS | Audio clip (webm/opus, about 2–5 s) | **Fallback:** the laptop mic with a push-to-talk key, posted to the same endpoint |
| 3 | Vultr: speech-to-text | In process | Text: "fly to the door and take a photo" | Whisper on a Vultr GPU, or a hosted STT; see the open questions |
| 4 | Vultr: LLM brain | Vultr Serverless Inference (OpenAI-compatible) | System prompt (personality, rules) + text + latest telemetry → **one tool call** `mission(where="the door", what="photo", after="come_back")` + a spoken one-liner ("On it.") | Dumb or dangerous requests: no tool call, just a sassy refusal |
| 5 | Vultr → laptop | The WSS connection the laptop opened at startup | `{mission, reply_text, request_id}` | The laptop is behind NAT, so it **dials out**; the cloud never connects in |
| 6 | Vultr: text-to-speech | In process | `reply_text` → mp3 | Sent to the laptop (and the phone) right away, so it talks while it starts moving |
| 7 | Laptop → Nest Mini | Bluetooth A2DP | Plays the mp3 | The Nest is the laptop's default audio output |
| 8 | Laptop: planner | In process | `mission()` → primitives: `scan` until `locate("door")` finds it → `goto(n, e)` stopping about 1 m short → photo → `return_home` | `locate()` (B) needs a forward frame + pose: step 9 |
| 9 | Pi → laptop | HTTP `:8080/forward.mjpg` and `/forward.jpg` + `X-Pose` | JPEG frames, 640×480, 10–15 fps | Pi side: the camera grabber; there's no OpenCV capture on QNX |
| 10 | Laptop → Pi | TCP `:7000`, JSON lines | `cmd {id, op, args}`; heartbeat at 5 Hz | Protocol v1 (PRD section 5) |
| 11 | Pi: safety gate | In process | Verdict per command: altitude window, fence, step size, battery, flow quality, person ahead | Rejected → `ack ok=false reason=…` → spoken by Jarvis |
| 12 | Pi → FC | USB CDC-ACM (pyusb, root) | MAVLink `SET_POSITION_TARGET_LOCAL_NED`, `COMMAND_LONG` (arm, takeoff, mode) | Verified two-way on the Pi |
| 13 | FC → motors | PWM | ArduPilot GUIDED position control | The EKF uses optical flow + baro for position |
| 14 | Pi → laptop | TCP `:7000` | `ack`, `done`, `telemetry` at 10 Hz, `event` | The planner moves to the next primitive on `done` |
| 15 | Laptop → Vultr | WSS | Telemetry at 2–5 Hz, JPEG at about 5 fps, `done`/`event`, the photo | Vultr fans out to every open dashboard |
| 16 | Vultr: VLM (for "read" / "solve" / "describe") | Serverless Inference | Frame + question → answer text → TTS → back through steps 6–7 | |

**Latency budget, voice to "moving":** upload 0.3 s + STT 0.5 s + LLM 1.0 s + WSS 0.1 s +
first `cmd` 0.1 s ≈ **2 s**. Target is under 3 s (PRD section 8).

## 2. Inside the drone (QNX Pi)

```
 down C270 ─► camgrab (C, libcamapi) ─► shared mem ─► flow proc [B] ──UDP :7002──┐
 fwd  C270 ─► camgrab (C, libcamapi) ─► shared mem ─┬► vision proc: TFLite [B] ─UDP :7001─┤
                                                    └► video server :8080 ─► laptop      │
                                                                                         ▼
                                   laptop ◄─TCP :7000─► EDGE CORE [A]: protocol server,
                                                        safety gate, executor, watchdog
                                                                 │ USB (pyusb, root)
                                                                 ▼
                                                          flight controller
 supervisor: restarts camgrab, flow and vision if they die (no HAM on the image)
```

| Process | Priority (`SCHED_FIFO`) | Rate | If it dies |
|---|---|---|---|
| Edge core: watchdog + safety loop | Highest | 50 Hz | The FC's own GCS failsafe (`FS_GCS_ENABLE`) lands, since the core is what heartbeats the FC |
| Flow → `OPTICAL_FLOW` (core forwards it) | High | 30 Hz | Flow quality → 0, the gate refuses moves, the EKF drifts → land |
| Camera grabbers | High | 30 fps | The supervisor restarts them |
| Vision (TFLite person) | Normal | 5–10 Hz | Restarted; meanwhile the gate treats "person unknown" as rejecting forward moves |
| Video server | Low | 10–15 fps | The demo loses video; flight is unaffected |

## 3. Failure behaviour

| What dies | What happens | Who does it |
|---|---|---|
| The phone or dashboard | Nothing; the laptop mic still works | — |
| Vultr / internet | No new voice commands; the current primitive finishes; manual commands from `tools.cmd` still work | Station |
| The laptop (or its Wi-Fi) | Heartbeats stop → **hover after 1 s → land after 5 s** | Edge watchdog |
| A vision, flow or camera process | Restarted by the supervisor; the gate blocks moves meanwhile | Supervisor + gate |
| The edge core on the Pi | The FC loses GCS heartbeats → lands | ArduPilot `FS_GCS_ENABLE` |
| The Pi entirely (power) | Same as above | ArduPilot |
| Everything | The safety pilot flips **SwA = Land** | FlySky → receiver → FC |
| Battery low | Below 10.6 V: new moves refused; below 10.2 V: auto-land | Gate; also ArduPilot `BATT_FS_LOW_ACT` |

## 4. Open questions to verify

1. Can a Windows laptop use the Nest Mini as a Bluetooth speaker? Latency? Fallback: Cast an mp3
   (pychromecast), or the laptop speaker.
2. Does Vultr Serverless Inference offer speech-to-text and a vision model, or do we need a
   Vultr GPU instance for Whisper and the VLM? Do the MLH credits cover GPUs?
3. Can QNX run **two** USB cameras at once (two `usb_camera` sensor units)? What's the USB
   bandwidth at 640×480?
4. Is the camera grabber → shared memory → Python path workable on QNX (`libcamapi` from C,
   `mmap` from Python)?
5. Can the Pi 5 + two webcams run from the power bank (5 V 3–5 A) without brownouts?
6. Wi-Fi: hotspot range and throughput for MJPEG plus telemetry at a few metres.
