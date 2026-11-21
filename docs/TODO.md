# To-do

Last updated: 2026-11-21. A = Person A (Flight + Voice + all off-drone software),
B = Person B (on-drone perception + hardware). Split changed 2026-11-21: A took over B's
software that isn't tied to drone hardware (localization, VLM tasks, visual memory).
Details and commands: [STATUS.md](STATUS.md). Plan and split: [PRD.md](PRD.md). Tick items
here as they're done.

## 🔴 CRITICAL: keys and accounts (the voice/vision demo does not run without these)

The software is written and tested as if these exist. Without them: rule-based commands
only, no speech-to-text, no vision, no memory, no voice. Put them in `.env` on the laptop **and**
on the Vultr VM (template: `.env.example`; `.env` is git-ignored, never commit it).
Check with `curl https://<cloud>/health` → `"keys": {"vultr": true, "elevenlabs": true}`.

- [ ] 🔴 **A: Vultr Serverless Inference key** → `VULTR_INFERENCE_KEY`. Powers the brain
      (`glm-5.3-flash`), speech-to-text (`whisper-large-v3-turbo`), vision (`qwen3.8-27b`:
      locate / read / solve / describe) and memory embeddings (`qwen3-embedding-4b`)
- [ ] 🔴 **A: ElevenLabs API key + voice ID** → `ELEVENLABS_API_KEY`, `ELEVENLABS_VOICE_ID`
      (the Jarvis voice). Without it: Vultr `chatterbox` TTS, then text only
- [ ] 🔴 **A: Vultr account/credits for a VM** (cloud app + dashboard) and a **domain** pointing
      at it (phones need HTTPS for the mic). Steps: [cloud/deploy.md](../cloud/deploy.md)
- [ ] 🔴 **A: `STATION_TOKEN`**: a long random string, the same on the laptop and the VM
- [ ] **A: check the models on the real key**: vision model returns usable boxes on real C270
      frames; STT latency; ElevenLabs vs chatterbox (models.md "Still to test")

## Now: before anything flies

- [ ] **A + B: Pi bench test, props OFF.** Commands in STATUS.md "QNX port".
  - [x] Copy `edge/`, `shared/` and `tools/` to the Pi (`~/hope`)
  - [x] Run `tools/usb_relay.py` as root, then `python3 -m edge --mav udpin:127.0.0.1:14555 --station-ip <laptop> --no-video`
  - [x] From the laptop: `python -m tools.cmd <pi ip>`, then `t` shows live telemetry (2026-11-21: battery, mode, attitude; gate rejected takeoff for flow quality 0 and a move while landed)
  - [ ] Arm/disarm works; the gate's rejection reasons show up
  - [ ] Pull the laptop's Wi-Fi → edge hovers, then lands (watch the mode change)
  - [ ] `slay` the edge core → the FC lands on its own (needs A4 params)
- [x] **A: FC failsafe params (review A4)**, set and read back 2026-11-21; stopping the edge core raised "GCS Failsafe" on the FC (disarmed, so no action): `FS_GCS_ENABLE=5`, `FS_GCS_TIMEOUT=2`, `FS_EKF_ACTION=1`. Never connect Mission Planner as sysid 255 at the same time
- [x] **A: transmitter** (2026-11-21, verified): the FS-i6X only offers SwA/VrA/VrB on channels 5/6 (SwB-D unavailable in the stock menu), so **SwA on channel 5: up = AltHold (`FLTMODE1-5=2`), down = Land (`FLTMODE6=9`)**. **The computer enters GUIDED itself**; the pilot takes over by flipping SwA (down = land; down-then-up = AltHold). Mode changes in about 0.2 s.
  - [x] **Edge rule (B / whoever fixes review2 #1, #5):** once the pilot changes mode away from GUIDED, the edge must **never** switch back to GUIDED or re-send LAND on its own. It reports "pilot has control" and rejects moves until the station explicitly re-takes off
- [x] **A: build `camgrab` on the Pi**: needs `apk add qnx-sf-base-dev qnx-screen-dev`. Verified: unit 5 → `camgrab_fwd`, 640×480 NV12, **30 fps, about 3 ms frame age** in Python (after the `edge/camera.py` mmap fix). Note: it warns "640x480 not accepted, keeping the camera default" (the default is 640×480 anyway); `-l` lists units 1, 2 and 5: `apk add qnx-sf-base-dev`, `cc -O2 -o camgrab edge/camgrab/camgrab.c -lcamapi`, `./camgrab -l`. If it fails on `stride` / `uv_stride` / `camera_get_supported_cameras`, check `/usr/include/camera/camera_api.h`
- [ ] **B: two cameras**: does QNX give the second C270 its own unit? Each camera on its own USB controller; the down camera at 320×240

## Next: get it hovering

- [x] **A: flow-only SITL test** (`tools/sitl_flowonly.py`, 2026-11-21): **no rangefinder = no position, ever** (GUIDED refused on the ground and in the air). With a rangefinder that reads valid on the ground: GUIDED takeoff works, hover drift 0.03 m/20 s
- [ ] **A: HC-SR04 → Arduino → Pi → `DISTANCE_SENSOR`** (now required, D13): Arduino sketch prints the distance over USB serial; a Pi process forwards it at 10–20 Hz (`RNGFND1_TYPE=10`, `RNGFND1_ORIENT=25`, low `RNGFND1_MIN`). Check that QNX sees the Arduino (pyusb, like the FC). Mount it away from the props, with a valid reading on the ground
  - [x] Code: `tools/hcsr04/hcsr04.ino` (prints `d <mm>` at 20 Hz; Trig D9, Echo D10), `tools/rangefinder.py` (`--usb` on the Pi as root, `--serial COMx` on a laptop) → `{"range_m"}` on UDP :7002 → edge → `DISTANCE_SENSOR` (min 2 cm, max 200 cm)
  - [x] Sketch uploaded (arduino-cli, Elegoo Uno = `2341:0043` on COM8); 17.8 readings/s on the laptop. Arduino moved to the Pi (USB controller 0, next to the FC); `tools/rangefinder.py --usb` as root reads it (needs SET_LINE_CODING, now in the code)
  - [x] FC params set + FC rebooted: `RNGFND1_TYPE=10`, `RNGFND1_ORIENT=25`, `RNGFND1_MIN=0.02`, `RNGFND1_MAX=2`. **End to end verified 2026-11-21:** the FC reports `range_m` 0.37–0.38 m at about 18 Hz for a surface held at about 37 cm (no echo is reported as 2.01 m = out of range)
  - [ ] Mount it on the drone pointing down, away from the props, on foam; check the on-ground reading is valid (above 2 cm)
  - [ ] Add `tools.rangefinder --usb` (root) to `tools/supervisor_pi.json` next to the relay
- Note for B: on Windows, `test_camera::test_reader_follows_a_restarted_grabber`, `test_jitter::test_measure_runs_at_rate` and `test_supervisor::test_killed_child_is_restarted_and_timed` fail (shared memory, timing and process-kill differences); they pass on macOS. Not Pi bugs
- [x] **A/B: edge re-sends the EKF origin until the FC confirms it** (done: 1 Hz until `GPS_GLOBAL_ORIGIN`, "origin set" or a position) (SITL: the one-shot origin at link-up was ignored before the EKF initialised; watch for "EKF3 IMU0 origin set" or `GPS_GLOBAL_ORIGIN`)
- [x] **A/B: short moves finish early** (done: tolerance min(0.3, max(0.1, 0.3 × distance)), takeoff too; flow-only SITL ALL PASS: fwd 0.5 → +0.34 m, takeoff 0.8 → 0.56 m): `move_body fwd=0.5` reported done at +0.24 m with `pos_tolerance=0.3`; use a tolerance relative to the move (e.g. min(0.3, 0.3 × distance)) or 0.15 m with flow
- [ ] **B: optical flow process** (`edge/flow/`): `ShmFrameReader("camgrab_down").gray()` → LK → rad/s → UDP :7002. Develop now with `python -m tools.fake_camgrab --name camgrab_down --webcam 0`
- [ ] **B: hand-roll sign check** on the real drone: `OF.flowX` tracks `OF.bodyX`
- [ ] **A + B: manual tethered AltHold hover** with the pilot
- [ ] **A + B: tethered GUIDED hover** via `tools.cmd`: takeoff 0.8 m, hold 30 s, land
- [ ] **A + B: `move_body fwd=0.5`** on the real drone

## Then: make it smart (Vultr)

Code done 2026-11-21 (`station/`, `cloud/`, 27 tests in `tests/test_station.py`, smoke-tested end
to end on the sim with no keys). Still to do with real keys / hardware:

- [x] **A: models + LLM command schema**: tools `takeoff / land / stop / home / move / mission /
      look / recall` (`station/brain.py`), rule-based fallback without a key
- [x] **A: planner** (`station/planner.py`): scan until found, approach 1 m short (fence-clamped,
      3 m legs), face it, photo / read / solve / describe, stay / come back / land; recall
- [x] **A: `locate()`** (`station/localize.py`): VLM box (0–1000) → bearing → range (size prior or
      floor contact) → NED; sim ground-truth stub `station/localize_stub.py`
- [x] **A: VLM tasks** (`station/vision.py`) and **visual memory** (`cloud/memory.py`: caption +
      embedding + pose in SQLite, numpy search, `/memory/search`)
- [x] **A: closed-loop approach** (fly half way, look again, correct; sim with +40 % range error:
      0.89 m → 0.34 m off the stop point), **second look for shaky sightings** (conf < 0.7),
      **"fly to where I'm pointing"** (VLM names the object), **memory re-ranking**
      (`bge-reranker-v2-m3` via `/rerank`, falls back if the key/endpoint doesn't have it)
- [x] **A: cloud app + phone dashboard** (`cloud/app.py`, `cloud/static/index.html`): push-to-talk,
      live video, telemetry, STOP/LAND for anyone, one viewer has the controls, flight log,
      photos, memory search
- [x] **A: laptop ↔ cloud WSS** (`station/station.py`: dials out, ping 15 s, backoff to 15 s)
- [ ] 🔴 **A: deploy to the Vultr VM** with Caddy + HTTPS ([cloud/deploy.md](../cloud/deploy.md))
- [ ] **A: measure the size prior and HFOV on the real C270** (`edge/video.py:HFOV_DEG`,
      `station/localize.py:SIZE_PRIOR`) and test `locate()` on real frames
- [ ] **A: audio out**: test the Nest Mini as a Windows Bluetooth speaker early; laptop speaker as
      the fallback (`station/audio.py` plays on the default output; install ffmpeg)
- [ ] **A: voice latency** under 3 s end to end (`/health`, `AI.last_latency`)

## Then: the QNX demo (review D)

- [x] **A: watchdog/heartbeat loop in C** at top `SCHED_FIFO`, pinned to a core (`edge/hbwd/hbwd.c`; SITL: hung core → LAND in 273 ms). On the Pi: `cc -O2 -o edge/hbwd/hbwd edge/hbwd/hbwd.c -lpthread`, then the supervisor runs it at FIFO 60 on CPU 3
- [x] **A: jitter histogram** (`tools/jitter.py`; run on the Pi as root) under a CPU hog, with and without priority
- [x] **A: supervisor** (no HAM; `tools/supervisor.py`): restarts camgrab / flow / vision / relay, logs restart time in ms
- [ ] **A: video server as its own process** (pose from the core over localhost UDP)
- [ ] **B: TFLite person detection** → UDP :7001 (`person_m`); show inference ms
- [ ] **B: palm stop** (QNX workshop hand model)
- [ ] **A + B: person → motor-stop latency** shown on screen
- [ ] **A + B: `slog2` logging** shown live

## Last 2 hours

- [ ] Feature freeze
- [ ] Rehearse the 60-second QNX demo and the Vultr demo 3×
- [ ] Record a backup video
- [ ] Devpost: A writes up real-time, voice, dashboard and memory; B writes up on-board AI
