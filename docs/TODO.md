# To-do

Last updated: 2026-11-21. A = Person A (Flight + Voice), B = Person B (Perception + Memory).
Details and commands: [STATUS.md](STATUS.md). Plan and split: [PRD.md](PRD.md). Tick items
here as they're done.

## Now: before anything flies

- [ ] **A + B: Pi bench test, props OFF.** Commands in STATUS.md "QNX port".
  - [ ] Copy `edge/`, `shared/` and `tools/` to the Pi
  - [ ] Run `tools/usb_relay.py` as root, then `python3 -m edge --mav udpin:127.0.0.1:14555 --station-ip <laptop> --no-video`
  - [ ] From the laptop: `python -m tools.cmd <pi ip>`, then `t` shows live telemetry
  - [ ] Arm/disarm works; the gate's rejection reasons show up
  - [ ] Pull the laptop's Wi-Fi → edge hovers, then lands (watch the mode change)
  - [ ] `slay` the edge core → the FC lands on its own (needs A4 params)
- [ ] **A: FC failsafe params (review A4)**: `FS_GCS_ENABLE=5`, `FS_GCS_TIMEOUT=2`, `FS_EKF_ACTION=1`. Never connect Mission Planner as sysid 255 at the same time
- [ ] **A: transmitter**: 3-position switch AltHold / Land / GUIDED (not Land only)
- [ ] **A: build `camgrab` on the Pi**: `apk add qnx-sf-base-dev`, `cc -O2 -o camgrab edge/camgrab/camgrab.c -lcamapi`, `./camgrab -l`. If it fails on `stride` / `uv_stride` / `camera_get_supported_cameras`, check `/usr/include/camera/camera_api.h`
- [ ] **B: two cameras**: does QNX give the second C270 its own unit? Each camera on its own USB controller; the down camera at 320×240

## Next: get it hovering

- [ ] **A: flow-only SITL test** (`GPS1_TYPE=0`, `SIM_FLOW_ENABLE=1`, `FLOW_TYPE=10`, no rangefinder). Every SITL pass so far used GPS
- [ ] **B: optical flow process** (`edge/flow/`): `ShmFrameReader("camgrab_down").gray()` → LK → rad/s → UDP :7002. Develop now with `python -m tools.fake_camgrab --name camgrab_down --webcam 0`
- [ ] **B: hand-roll sign check** on the real drone: `OF.flowX` tracks `OF.bodyX`
- [ ] **A + B: manual tethered AltHold hover** with the pilot
- [ ] **A + B: tethered GUIDED hover** via `tools.cmd`: takeoff 0.8 m, hold 30 s, land
- [ ] **A + B: `move_body fwd=0.5`** on the real drone

## Then: make it smart (Vultr)

- [ ] **A + B: decide the models** (PRD "Still to decide"): check `GET https://api.vultrinference.com/v1/models`, GPU credits
- [ ] **A: LLM command schema**: tool definitions for `mission(where, what, after)` → protocol primitives
- [ ] **A: planner** (`station/planner.py`): mission → primitives, scan-until-found, approach 1 m short, come back. Uses a stub `locate()` until B's lands
- [ ] **A: Vultr VM**: faster-whisper STT (D8), LLM brain, ElevenLabs TTS (D9), Caddy + HTTPS
- [ ] **A: laptop ↔ Vultr WSS link** (ping every 15–20 s, reconnect with backoff)
- [ ] **A: phone dashboard**: push-to-talk, live video, telemetry, command box, flight log
- [ ] **A: audio out**: test the Nest Mini as a Windows Bluetooth speaker early; laptop speaker as the fallback
- [ ] **B: `locate(name, jpeg, pose, K)`**: open-vocabulary detector → bearing → range → NED goal. Test on the sim room (`python -m edge --sim`, `/forward.jpg`)
- [ ] **B: VLM tasks**: read / solve / describe / photo
- [ ] **B: visual memory**: ingest pose-tagged frames, `POST /memory/search`

## Then: the QNX demo (review D)

- [ ] **A: watchdog/heartbeat loop in C** at top `SCHED_FIFO`, pinned to a core
- [ ] **A: jitter histogram** under a CPU hog, with and without priority
- [ ] **A: supervisor** (no HAM): restarts camgrab / flow / vision / relay, logs restart time in ms
- [ ] **A: video server as its own process** (pose from the core over localhost UDP)
- [ ] **B: TFLite person detection** → UDP :7001 (`person_m`); show inference ms
- [ ] **B: palm stop** (QNX workshop hand model)
- [ ] **A + B: person → motor-stop latency** shown on screen
- [ ] **A + B: `slog2` logging** shown live

## Last 2 hours

- [ ] Feature freeze
- [ ] Rehearse the 60-second QNX demo and the Vultr demo 3×
- [ ] Record a backup video
- [ ] Devpost: A writes up real-time + dashboard; B writes up on-board AI + memory
