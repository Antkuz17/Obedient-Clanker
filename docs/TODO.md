# To-do

## 🔴 KEYS FIRST: every key and setting (nothing smart runs without these)

**Names only here, never the values.** This file is in git. The real values go in `.env`
(git-ignored) on the laptop **and** on the Vultr VM; template: `.env.example`.
Without the Vultr key the station silently uses the rule brain: no LLM, no speech-to-text,
no vision, no memory, no voice.

Status on the cloud lead's laptop, checked 2026-11-21 (keys rechecked 14:10: Vultr key answers `check_ai --only models`; realtime STT 401 still open, see STATUS):

| ✓ | `.env` variable | Where | What it powers / how to get it | Status |
|---|---|---|---|---|
| [x] | `VULTR_INFERENCE_KEY` | laptop + VM | Brain (`glm-5.3-flash`), STT (`whisper-large-v3-turbo`), vision (`qwen3.8-27b`), embeddings, rerank, `chatterbox` TTS. Vultr dashboard → Serverless Inference. A working key was used for `docs/models.md` timings; get it from whoever ran those | set |
| [x] | `ELEVENLABS_API_KEY` | laptop + VM | Jarvis voice. elevenlabs.io → Profile → API keys. Without it: `chatterbox`, then text only | set |
| [x] | `ELEVENLABS_VOICE_ID` | laptop + VM | The cloned/chosen Jarvis voice. ElevenLabs → Voices → copy ID | set |
| [x] | `STATION_TOKEN` | laptop + VM (same value) | Laptop ↔ cloud link auth. Generated 2026-11-21; `cloud/deploy.sh` copies it | set |
| [ ] | `CLOUD_URL` | laptop only | `wss://<your-domain>` of the Vultr VM. Needs the VM + domain below | **blank** |
| [ ] | `EDGE_HOST` | laptop only | The Pi's IP on the drone network | **blank** |
| [ ] | 🔴 Vultr VM + credits + **domain** | n/a | Hosts cloud app + dashboard; phones need HTTPS for the mic. Steps: [cloud/deploy.md](../cloud/deploy.md) | not done |

Optional overrides (defaults are fine): `VULTR_INFERENCE_BASE`, `BRAIN_MODEL`,
`BRAIN_BACKUP_MODEL`, `VISION_MODEL`, `STT_MODEL`, `EMBED_MODEL`, `RERANK_MODEL`,
`VULTR_TTS_MODEL`, `ELEVENLABS_MODEL`, `EDGE_PORT`, `VIDEO_PORT`, `DATA_DIR`.

**Verify once filled:**
- [ ] `python -m station.check_ai` on the laptop: brain, STT, vision boxes, read/solve,
      embeddings, rerank, TTS, voice latency (also answers models.md "Still to test")
- [ ] `curl https://<cloud>/health` → `"keys": {"vultr": true, "elevenlabs": true}`

---

Last updated: 2026-11-21. A = Person A (Flight + Voice + all off-drone software),
B = Person B (on-drone perception + hardware). Split changed 2026-11-21: A took over B's
software that isn't tied to drone hardware (localization, VLM tasks, visual memory).
Details and commands: [STATUS.md](STATUS.md). Plan and split: [PRD.md](PRD.md). Tick items
here as they're done.

## 🔴 Outdoors (decided 2026-11-21): what has to change before it can fly outside

Fence, ceiling and step limits are already off (STATUS "Limits removed"). These are not rules,
they are how the drone navigates, and they cap it at about 2 m up until changed:

- [ ] **Turn the GPS on** (HGLRC M100-PRO is already on the drone, disabled): on the Pi,
      `python3 -m tools.fc_profile --outdoor` (check), then `--outdoor --apply --reboot`.
      Re-enable the GPS arming check if it was skipped (`ARMING_SKIPCHK`, flight-setup.md).
      Wait for a 3D fix before arming; the edge refuses takeoff until the EKF has a position
- [x] **Edge in GPS mode** (code done): `edge --gps` (in `supervisor_pi.json`): no fake EKF
      origin, no optical-flow requirement
- [ ] **Link range:** the laptop ↔ Pi link is Wi-Fi. No station heartbeat for 1 s → hover, 5 s → land
      where it is. Range test it outdoors; consider a better antenna / router on the field
- [x] **Speed** (code done): `WP_SPD=20`, `WP_SPD_UP=10`, `WP_SPD_DN=1.5` written on every
      connect. Top speed now depends on the airframe and `ANGLE_MAX`
- [ ] **Battery:** 3S 3000 mAh on an F450 with 10" props is roughly 8–12 min; measure it
- [ ] **Forward safety at speed:** the forward HC-SR04 sees 2 m and the stop distance is 0.7 m.
      At 20 m/s the drone covers that in a tenth of a second and needs ~10+ m to stop, so the
      person / obstacle stops only protect at low speed. The person detector's range estimate
      is weak beyond ~5 m
- [x] **Dashboard**: fence map replaced by a zooming track
- [ ] **Rules:** Transport Canada for a drone over 250 g (an F450 build is well over): registration, a pilot certificate,
      122 m (400 ft) max, distance from bystanders, and controlled airspace near
      London Airport (CYXU). Check before flying on campus

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
- [ ] **B: two cameras**: does QNX give the second C270 its own unit? Each camera on its own USB controller; the down camera at 320×240. `supervisor_pi.json` now expects it as **unit 6 → `camgrab_down`** (flow reads it)

## Next: get it hovering

- [x] **Current sensor offset (2026-11-21)**: idle read 32.9 A (uncalibrated 0.82 V offset × `BATT_AMP_PERVLT` 40), so the FC "used" 3000 mAh in ~6 min and latched "Battery 1 low capacity failsafe". Set `BATT_AMP_OFFSET=0.82` (idle now 0.13 A), all failsafes kept. Still to do: calibrate `BATT_AMP_PERVLT` from the charger's mAh after a flight
- [x] **Motor test, props off (2026-11-21)**: `tools/motor_test.py`: A-D order and directions correct
- [x] **Preflight ALL GO again after rewiring (2026-11-21)**: needed an FC reboot (mAh counter), a stack restart (down camera unplugged at start) and separating the Arduino from the ESP8266 (contact killed its USB until moved apart: **keep them insulated**)
- [x] **Preflight ALL GO on the real drone (2026-11-21)**: `tools/preflight.py` via the relay tap: FC link, profile, no pre-arm errors, **EKF `pos_h_rel` from flow + rangefinder**, flow 20 Hz q≈250 at the FC, rangefinder 0.14 m on the ground, battery 12.3 V, throttle low, SwA up, camera fresh. Fixes needed on the way: FC reboot to clear the battery mAh counter (FC stayed powered over USB during a pack swap), compass recalibration with everything mounted (`tools/compass_cal.py`, "mag field xy diff" pre-arm), relay auto-restart after an FC reboot

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
- [x] **Drone lead: optical flow process** (`edge/flow/`, `python3 -m edge.flow --shm <name> --print`): LK at 320×240, forward-backward check, median shift → rad/s (f = 277 px for a 60° HFOV), quality from inliers + agreement → UDP :7002. Unit tests on synthetic shifts. **On the Pi with the C270: q≈207, ~135 tracks, 10 ms/frame, 15 Hz** (camera at 15 fps in dim light)
  - [x] FC flow params applied with `tools/fc_profile.py --apply --reboot` (backup on the Pi): `FLOW_TYPE=5`, `EK3_SRC1_POSXY=0`, `EK3_SRC1_VELXY=5`, `EK3_SRC1_POSZ=1`, `EK3_SRC1_VELZ=0`, `GPS1_TYPE=0`, `EK3_FLOW_DELAY` ≈ 60 ms (tune), `FLOW_OPTIONS=0`; reboot
  - [x] **Hand-roll sign check** (2026-11-21, real drone, `tools/flow_signcheck.py`): raw flow was backwards on both axes (corr flowX/roll −0.51, flowY/pitch −0.61, cross ≈ 0) → **`--flip-x --flip-y`** in `supervisor_pi.json`; rechecked: +0.50 / +0.34, cross ≈ 0. Strength is limited by light/texture (quality drops while moving; down camera at 8–15 fps in dim light)
  - [ ] Measure the real C270 HFOV (or tune `FLOW_FXSCALER`) and set `edge/flow` `--hfov` (default now 45°, from the 55° diagonal spec; also `edge/video.py:HFOV_DEG`)
  - [ ] **Drone lead: check the direct sensor path (review3 #8)** on the bench: supervisor up, `slay` the edge core, the FC must still show `OPTICAL_FLOW` and `DISTANCE_SENSOR` arriving (Mission Planner status / MAVLink inspector on another link)
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
- [ ] 🔴 **A: deploy to the Vultr VM**: one command once the VM + domain exist: `scp .env root@<vm>:/root/hope.env && ssh root@<vm> 'bash -s' < cloud/deploy.sh <domain>` ([cloud/deploy.md](../cloud/deploy.md))
- [ ] **A: measure the size prior and HFOV on the real C270** (`edge/video.py:HFOV_DEG`,
      `station/localize.py:SIZE_PRIOR`) and test `locate()` on real frames
- [ ] **A: audio out**: test the Nest Mini as a Windows Bluetooth speaker early; laptop speaker as
      the fallback (`station/audio.py` plays on the default output; install ffmpeg)
- [ ] **A: voice latency** under 3 s end to end: `python -m station.check_ai --only brain,voice` prints the estimate

## Then: the QNX demo (review D)

- [x] **A: watchdog/heartbeat loop in C** at top `SCHED_FIFO`, pinned to a core (`edge/hbwd/hbwd.c`; SITL: hung core → LAND in 273 ms). On the Pi: `cc -O2 -o edge/hbwd/hbwd edge/hbwd/hbwd.c -lsocket` (QNX keeps sockets in libsocket), then the supervisor runs it at FIFO 60 on CPU 3
- [x] **A: jitter histogram** (`tools/jitter.py`; run on the Pi as root) under a CPU hog, with and without priority
- [x] **A: supervisor** (no HAM; `tools/supervisor.py`): restarts camgrab / flow / vision / relay, logs restart time in ms
- [x] **A: video server as its own process** (pose from the core over localhost UDP; `python -m edge.video`, core `--video-proc 127.0.0.1:7003`; in the Pi supervisor config at prio 10)
- [ ] **TFLite person detection** → UDP :7001 (`person_m`, `t`, `infer_ms`): **code done** (`edge/vision/`, in `supervisor_pi.json` at prio 41; tests with a fake interpreter). Still to do on the Pi:
  - [ ] `python -m tools.fetch_person_model` on the laptop, then `python -m tools.deploy_pi`
  - [ ] On the Pi: `python3 -m edge.vision --shm camgrab_fwd --print`. Check it starts (tflite-runtime 2.21), the inference time, and that a person in view reads a sensible distance (stand at 1, 2 and 3 m). Class 0 must be "person" (the script prints the first labels)
  - [ ] Then forward moves work on the real drone without `--no-person-check`
- [ ] **Forward obstacle HC-SR04** (code done: sketch pings down D9/D10 and forward D11/D12 alternately; `rangefinder.py` sends `fwd_m`; gate keeps 0.7 m from echoes): wire it, re-upload the sketch, check `python -m tools.rangefinder ... --print` shows forward readings (an unwired sensor reads "clear"), then add `--require-obstacle-sensor` to the edge in `supervisor_pi.json`
- [ ] **B: palm stop** (QNX workshop hand model)
- [x] **A: dashboard QNX card** (code done, sim-verified): live 50 Hz loop jitter graph
      (`edge --stats-udp <laptop>:7010`), jitter benchmark table (`tools.jitter --compare --udp
      <laptop>:7010`), supervisor process list with respawn / serving ms
      (`tools.supervisor ... --events-udp <laptop>:7010`), stop-latency tile. Pi → UDP :7010 →
      station → cloud → every viewer. `supervisor_pi.json` already passes `--stats-udp LAPTOP_IP:7010`
- [ ] **A + B: person → motor-stop latency** shown on screen: the dashboard shows it once the
      detector's UDP message carries `"t": time.time()` of the frame (and optionally `"infer_ms"`);
      without `t` it only shows the edge's own reaction time
- [ ] **A + B: `slog2` logging** shown live

## Last 2 hours

- [ ] Feature freeze
- [ ] Rehearse the 60-second QNX demo and the Vultr demo 3× (Vultr: script in [demo.md](demo.md); `python -m station.dev` runs it on the sim)
- [ ] Record a backup video
- [ ] Devpost: cloud half drafted in [devpost-cloud.md](devpost-cloud.md) (fill in real latencies after `check_ai`); drone lead writes the QNX / on-board half
