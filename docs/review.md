# Plan review (2026-11-21)

Three independent reviews of [flow.md](flow.md), [PRD.md](PRD.md) and the `edge/` code: a QNX
judge's view, a check of every external connection, and an adversarial safety and architecture
review. Code findings marked ✔ were spot-checked against the source; the others are the
reviewer's reading and should be confirmed when fixing them.

## A. Safety bugs in `edge/` (fix before any real flight)

| # | Problem | Fix |
|---|---|---|
| A1 ✔ | `land` before arming has finished returns "already landed" (`executor.py:166`), but the takeoff thread (`mav.py:136-150`) still arms and sends `NAV_TAKEOFF`. `stop`/`land` don't abort a takeoff | Abort flag checked by the takeoff thread before arming and before `NAV_TAKEOFF`; `land`/`stop` set it |
| A2 | `land()` sends LAND once, with no retry. `executor.py:170` ignores the `_wait` result and reports OK even if the drone is still flying | Resend LAND at 2 Hz until the mode reads LAND; return FAILED on timeout |
| A3 ✔ | The GCS heartbeat runs in its own thread (`mav.py:83`), so a hung event loop keeps the FC's failsafe from firing. Reader or heartbeat thread exceptions freeze the state and leave `connected` True | Send the FC heartbeat from `safety_tick`; catch exceptions in both threads and set `connected=False` |
| A4 | FC failsafe parameters aren't set anywhere (`FS_GCS_ENABLE` defaults off; RTL is impossible without GPS) | `FS_GCS_ENABLE=5` (Land), `FS_GCS_TIMEOUT=2`, `FS_EKF_ACTION=1`. Never connect Mission Planner as sysid 255 at the same time |
| A5 | A USB drop mid-flight means no flow, then a landing that drifts. `_setup` gives up after 30 s and never reconnects | Reopen on USB errors, then resend message intervals and the origin. **Pilot switch: AltHold / Land / GUIDED** (3 positions), not Land only |
| A6 | No EKF health check: when position goes bad, the old `n/e` stays and `hold()` targets a stale position. No `mode == GUIDED` check despite PRD:139 | Stream `EKF_STATUS_REPORT`; reject moves without the position flags; auto-land if they're lost in flight; check the mode |
| A7 | If `EXTENDED_SYS_STATE` is missing, the drone counts as landed (`mav.py:118`), which disables the watchdog and battery land (`server.py:163,175`) | Infer airborne from armed + altitude when that message is stale |
| A8 ✔ | Unknown or stale person distance **allows** forward moves (`safety.py:103`); flow.md says it rejects them | Reject forward moves when person data is missing or stale (once the vision process exists) |
| A9 | Anyone on the Wi-Fi can connect to `:7000` and fly it, as root; the newest connection wins | A small **root USB relay process** → UDP `127.0.0.1:14555`; the core runs unprivileged with `--mav udpin:127.0.0.1:14555`. Accept only the station's IP or a token |
| A10 | The station heartbeat shares the planner's event loop (`client.py:58`): one blocking STT or LLM call over 1 s → hover, over 5 s → land | Heartbeat in its own thread; all cloud I/O async |
| A11 | Three threads send MAVLink without a lock (`mav.py:85,153,166`) | A single writer queue |
| A12 | `return_home` targets the EKF origin (where the EKF started at boot), not the takeoff point; the fence is centred on a drifting estimate | Record home at takeoff; centre the fence on it |
| A13 | `pos_tolerance=0.15` m is too tight for flow | 0.3 m |

## B. Feasibility: what the connection check changed

| Link | Verdict | Change |
|---|---|---|
| Nest Mini as a Windows Bluetooth speaker | **Risky.** Often pairs but won't appear as an output; needs Home app setup first | Test early. Fallback: laptop speaker or any Bluetooth/USB speaker. Cast is worse (3–30 s start, mDNS breaks on a hotspot) |
| Phone mic over HTTPS | Works | iOS Safari < 18.4 records only mp4/AAC: always `ffmpeg → 16 kHz mono wav` before Whisper |
| Vultr LLM + tool calling | Works | `https://api.vultrinference.com/v1/chat/completions`; tool calling is documented for `kimi-k2-instruct`. Check `GET /v1/models` first |
| **Vultr speech-to-text** | **Doesn't exist** (only chat, prompt, TTS); vision models are unnamed | **faster-whisper on the laptop CPU** (`small.en` int8, about 1 s per clip). Vultr TTS is fine |
| Vultr GPU via MLH credits | Unknown (a Vultr-run event this month excluded GPUs) | Ask at the booth; plan CPU + Serverless Inference |
| HTTPS on a Vultr VM | Works | Caddy + domain A record; `sslip.io` if DNS is slow |
| Laptop → cloud WSS | Works | Ping every 15–20 s, reconnect with backoff |
| **iPhone hotspot for the Pi ↔ laptop LAN** | **Risky.** Apple doesn't guarantee client-to-client traffic, and iOS 18 broke multicast | Travel router if available; otherwise fixed IPs, no mDNS, and test it |
| Two C270s, YUYV 640×480 @ 30 | **Won't fit on one USB2 bus** (18.4 MB/s each) | One camera per USB controller; the down camera at 320×240 or 15 fps |
| Pi 5 power for two webcams | **Risky.** Without 5 V/5 A, USB is capped at 600 mA total | 5 V/5 A PD bank, `usb_max_current_enable=1`; measure it |

## C. Flight stack gaps

- **Flow with no rangefinder is untested.** EK3 should fall back to baro height over a flat
  floor, but a ±0.3 m baro error ≈ 30% velocity error. **Verify in SITL first** (`GPS1_TYPE=0`,
  `SIM_FLOW_ENABLE=1`, `FLOW_TYPE=10`, no rangefinder), because every SITL pass so far used GPS.
  Lower `alt_max` to about 1.5 m for baro.
- ArduPilot ignores `OPTICAL_FLOW.ground_distance`. Height must arrive as `DISTANCE_SENSOR`
  (`RNGFND1_TYPE=10`); there's no input path for that yet.
- Fly first with `FLOW_OPTIONS=0` and tune `EK3_FLOW_DELAY`; Pi-side derotation later.
- `FLOW_ALT_MIN` may not exist (check a param fetch). USB MAVLink is `SERIAL0`, not `SERIAL1`.
- The camera grabber can simply write raw frames to a stdout pipe; no shared memory needed.
- The video server runs inside the safety process (`__main__.py:44`). Run it as a separate
  process (`--no-video` on the core).
- CPU is fine (LK at 320×240 takes a few ms; SSD-MobileNet about 40 ms).

## D. QNX judging

- **Weakest point today: nothing QNX-specific runs on QNX yet.** The grabber, flow, vision,
  supervisor, USB transport and jitter graph are all still planned.
- **"The real AI is in the cloud"** is the risk. Make the on-board AI visible: boxes on the
  Pi's video, inference ms on the dashboard, palm stop (QNX's own workshop model), and one demo
  with the internet unplugged.
- **Don't claim hard real time from Python asyncio.** Write the small watchdog/heartbeat loop in
  **C** at `SCHED_FIFO`, pin it to one core, and show jitter histograms under a CPU hog, with and
  without priority.
- Present the root libusb link as **a user-space USB driver** (idiomatic QNX).
- **Drop the "Pi 4 Linux flies" fallback.** The fallback is a QNX bench or tethered demo.
- Extras by value per hour: jitter graph (1–2 h) → person → motor-stop latency in ms (1 h) →
  kill-and-restart time in ms (1–2 h) → grabber on the Sensor Framework, said out loud → `slog2`
  logging shown live (1 h) → `/dev/drone` resource manager (3–5 h, stretch).

**60-second QNX demo:** `pidin` with priorities → "fly up 10 m" rejected on the Pi, reason
spoken → a person steps in, it stops, latency on screen → `slay vision`, restart in X ms while
the hover holds, `slog2info` → jitter histogram under a CPU hog → pull the laptop's Wi-Fi:
hover, then land.

## E. Critical path

1. FC params (A4, flow, battery) and a **manual tethered AltHold hover** with the pilot.
2. Fix A1–A3, the USB relay process, and the edge on the Pi, **props off**: arm, land, Wi-Fi
   pull, kill the core (the FC should land).
3. In parallel: the flow-only SITL test; the down-camera grabber + flow; the hand-roll sign
   check (`OF.flowX` vs `OF.bodyX`).
4. Tethered GUIDED hover with `tools.cmd`: takeoff 0.8 m, hold 30 s, land.
5. `move_body fwd=0.5`, then the same command from the voice planner.
6. The forward camera, TFLite and MJPEG; then the QNX demo extras.

**Fallback if flow won't hold:** demo the QNX safety and watchdog story tethered or on the
bench, and run voice against the sim and SITL.
