# Status

Last updated: 2026-11-21. Update this at the end of every session.

## Done (Person A)

| Piece | Files | Verified how |
|---|---|---|
| Protocol v1 contract | `shared/protocol.py` | unit tests |
| Async client (heartbeats, `run()` / `send()` / `wait_done()`) | `shared/client.py` | used by all e2e tests |
| Safety gate (alt window, fence, step, battery, flow quality, person ahead) | `edge/safety.py`, `edge/config.py` | unit tests |
| Executor: one task at a time, supersede, `stop` wins, landing beats `stop`, timeouts | `edge/executor.py` | e2e tests vs the sim |
| Server: TCP protocol, 10 Hz telemetry, 50 Hz safety loop, watchdog (hover 1 s, land 5 s), battery auto-land, perception UDP :7001, flow UDP :7002 → FC | `edge/server.py` | e2e tests vs the sim |
| Kinematic sim vehicle | `edge/sim.py` | e2e tests |
| Video server + sim camera (fake room: door, whiteboard, keys, chair, plant) | `edge/video.py` | manual: `/forward.jpg`, `X-Pose`, `/camera.json` |
| MAVLink vehicle (GUIDED, arm, takeoff, position targets, origin, telemetry, `OPTICAL_FLOW` forwarding, GCS heartbeat, FC pre-arm reasons in `done`) | `edge/mav.py` | **ArduPilot SITL** (Copter stable, `tools/sitl_flight.py`): takeoff, fwd/yaw/right moves, gate rejection, return home, watchdog hover → land → disarm all pass with altitude held within 0.15 m. `OPTICAL_FLOW` forwarding is **not** tested (SITL has no flow). Not run on the real FC yet |
| SITL harness | `tools/sitl/Dockerfile`, `tools/sitl_flight.py` | see above |
| **USB MAVLink transport** (`--mav usb`): pymavlink `StreamMav` over Anton's pyusb CDC-ACM code | `edge/mavlink_usb.py` | Full SITL flight passes through `StreamMav` (`sitl_flight rawtcp:127.0.0.1:5760`) + regression test. The `UsbCdc` libusb calls are copied from the verified probe but **not yet run inside the edge on the Pi** |
| **Camera grabber** (C, `libcamapi` → shared memory, seqlock, NV12/YUYV, strips stride padding, `-l` lists units) | `edge/camgrab/camgrab.c` | Compiled with `-Wall -Wextra` and run on the Mac against a stub camera library (`tests/camstub/`) that pads rows: Python sees exact pixels in order. **Not compiled on QNX yet**, and the stub copies the *real* header only as far as camgrab uses it (see below) |
| Shared-memory frame reader/writer + laptop stand-in | `edge/camera.py`, `tools/fake_camgrab.py` | unit tests; manual: sim video → fake_camgrab → shm → `edge --camera shm:NAME` → `/forward.jpg` |
| Fail-safe: forward moves refused when the person detector is silent (real drone only; `--no-person-check` to override) | `edge/safety.py`, `edge/server.py` | unit test |
| Entry point + `SCHED_FIFO` flag | `edge/__main__.py` | manual smoke test with `--sim` |
| Protocol REPL | `tools/cmd.py` | not run interactively yet |

**Review fixes (docs/review.md section A):** A1, A2, A3, A5, A6, A7, A8, A9 (relay + `--station-ip`),
A10, A11, A12, A13 are done; every one has a unit test or a SITL check. **Not done: A4**
(set the FC params on the real board: `FS_GCS_ENABLE=5`, `FS_GCS_TIMEOUT=2`,
`FS_EKF_ACTION=1`; SITL proves FS_GCS works).

SITL checks (`tools/sitl_flight.py`, all PASS, also through `tools/usb_relay.py`): takeoff,
moves, yaw, gate rejection, return to the takeoff point, watchdog hover → land, and **a hung
edge event loop makes ArduPilot's own GCS failsafe land the drone**. FC reboot under the relay:
the edge re-sends stream rates + origin and position comes back.

The test suite has 36 passing tests (`.venv/bin/python -m pytest -q`), plus `tools/sitl_flight.py`, which
must print ALL PASS.

Fixed after SITL: the stream transport handed pymavlink whole chunks, so messages queued up
and telemetry lagged further and further behind (the drone armed but never took off). It now
hands over only the bytes pymavlink asks for. Relative moves now build on the last *commanded* target, not the measured
position (altitude used to sag about 0.1 m per command).

## Still open from the review (Person A)

- **A4** FC params on the board (above). Review C: flow-only SITL (`GPS1_TYPE=0`,
  `SIM_FLOW_ENABLE=1`, `FLOW_TYPE=10`); a `DISTANCE_SENSOR` input path for height; the video
  server as its own process (needs pose from the core, e.g. UDP on localhost).
- Review D: the watchdog/heartbeat loop in **C** at `SCHED_FIFO` with a jitter histogram; the
  supervisor.
- Kept shared memory rather than the suggested stdout pipe: the forward camera has two
  readers (video + TFLite), and a pipe serves one.

## Next for Person A, in order

1. ~~SITL run of `edge/mav.py`~~ done.
2. **Planner** (`station/planner.py`): `mission(where, what, after)` → primitive sequence using
   `shared/client.py`. Include scan-until-found with a stub `locate()`
   (`station/localize_stub.py`, reading `edge.video.SIM_WORLD`).
3. **LLM** (`station/brain.py`): tool-call schema for `mission()`, personality, refusals.
   Provider: Vultr Serverless Inference (OpenAI-compatible); keep it swappable.
4. **Voice loop**: mic → VAD → Whisper → brain → TTS → Nest Mini.
5. **Vultr cloud + dashboard** (`cloud/`).
6. **QNX port** (Pi facts in qnx.md, "Verified on our Pi"): copy `edge/` and `shared/` to the Pi
   and run with `--rt-prio`.
   - **FC link: USB via the relay.** On the Pi, props **off**:
     `sudo env PYTHONPATH=$HOME/.local/lib/python3.14/site-packages python3 -m tools.usb_relay`
     then (as qnxuser) `python3 -m edge --mav udpin:127.0.0.1:14555 --station-ip <laptop>
     --no-video`; connect `python -m tools.cmd <pi ip>` from the laptop, then check telemetry,
     arm/disarm, the Wi-Fi pull (hover → land) and `slay` of the core (FC lands).
   - **Camera grabber: written, needs building on the Pi.** `apk add qnx-sf-base-dev`, then
     `cc -O2 -o camgrab edge/camgrab/camgrab.c -lcamapi`, `./camgrab -l` to list units, and
     `./camgrab -u 5 -n camgrab_fwd`. Check that `python3 -c "from edge.camera import
     ShmFrameReader as R; print(R('camgrab_fwd').wait_new().gray().shape)"` prints
     `(480, 640)`. Then the edge video uses it by default on `--mav`. Two cameras: run two
     camgrabs (`-u 5 -n camgrab_fwd`, `-u <other unit> -n camgrab_down`); whether QNX gives the
     second C270 its own unit is still open (flow.md Q3).
   - **Supervisor: done** (`tools/supervisor.py`, no HAM). `python3 -m tools.supervisor
     tools/supervisor_pi.json` starts usb_relay (root), edge and camgrab (as qnxuser, SCHED_FIFO
     prio from the config), restarts any that die, and logs JSON events with `spawn_ms`
     (death -> respawn) and `ready_ms` (death -> serving again: new shm frame / TCP accepting).
     Crash loops back off to 5 s. `--events-udp host:port` streams events (dashboard: not wired
     yet). Laptop demo (`tools/supervisor_laptop.json`): `kill -9` camgrab -> respawn 3.5 ms,
     new frame 208 ms; kill edge -> 3.9 ms, serving 119 ms. Add flow/vision entries when they
     exist. On the Pi: replace `LAPTOP_IP`, run as root, then `slay camgrab` for the demo.
   - **Jitter benchmark: done** (`tools/jitter.py`). On the Pi, as root:
     `python3 -m tools.jitter --compare --out jitter.json` runs idle/hog x normal/FIFO 50 and
     prints p50/p99/max lateness + a histogram; the JSON (every sample) feeds the dashboard graph.
     macOS baseline (no FIFO): p50 ~2.5-3.4 ms, p99 5-12 ms, max up to 34 ms under the hog.
     Not yet: the edge's own `EdgeServer.loop_stats` streamed live, and the loop in C.

## Review follow-ups (from [review.md](review.md), 2026-11-21)

Before any real flight:
- [x] A1: `land`/`stop` abort an in-progress takeoff (an abort flag in the takeoff thread)
- [x] A2: LAND resent until the mode reads LAND; FAILED on timeout
- [x] A3 (SITL: hung loop → FC lands): FC heartbeat sent from `safety_tick`; thread exceptions → `connected=False`
- [ ] A4 (**board params + transmitter, by hand**): FC params `FS_GCS_ENABLE=5`, `FS_GCS_TIMEOUT=2`, `FS_EKF_ACTION=1`; pilot switch AltHold / Land / GUIDED
- [x] A5–A7 (A5 SITL reboot test): USB reconnect; EKF health + GUIDED check in the gate; airborne from armed + alt when `EXTENDED_SYS_STATE` is stale
- [x] A8: forward moves refused when the person detector is dead (`40f6f54`)
- [x] A9 (`tools/usb_relay.py`, `--station-ip`; SITL flight through it): root USB relay → UDP `127.0.0.1:14555`, core unprivileged, station-only connections
- [x] A10–A13: station heartbeat thread; one MAVLink writer; home = takeoff point; `pos_tolerance` 0.3
- [ ] Flow-only SITL test (no GPS, no rangefinder) before relying on baro height
- [ ] Decide models and the LLM command schema (PRD "Still to decide")

## Second review (2026-11-21)

[review2.md](review2.md): a fresh-eyes review of the design and the code. 13 findings, 8 reproduced
with scripts in `tools/review/`. **Fixed (A, 2026-11-21): #1-#8, #10-#13**; #14 fixed by B.
Verified: 45 tests; Anton's repros rerun unchanged (`sim_exp`, `sitl_exp`, `tools/review/hung_planner.py`
with the edge in its own process); `tools.sitl_flight` ALL PASS 8 of 9 runs.
- Speed limits: set on every connect as `WP_SPD*` (ArduPilot 4.7, m/s) or `WPNAV_SPEED*` (older,
  cm/s), read back; takeoff is refused until confirmed. A re-setup after a telemetry gap keeps them.
- **Open: intermittent SITL failure** (1 of 9): re-takeoff after the watchdog landing aborted with
  "position estimate lost". The failsafe now logs the EKF report age and flags; next failure tells
  us stale telemetry (our side) vs bad EKF flags (may be the same root as #9).
- **Open: #9** flow-only takeoff on the predicted-position flag; check with `tools.sitl_flowonly`.

## Open questions / known gaps

- **camgrab vs the real `camera_api.h`:** `rpi_cam.cpp` (QNX's workshop) confirms
  `camera_open`, frame types, `camera_set/get_vf_property`, `camera_start_viewfinder` and
  `framedesc.nv12.{width,height,uv_offset}` / `framedesc.ycbycr.{width,height}`. From memory
  and **unconfirmed**: `nv12.stride`, `nv12.uv_stride`, `ycbycr.stride` and
  `camera_get_supported_cameras()` (only used by `-l`). If the compile fails on those, check
  `/usr/include/camera/camera_api.h` on the Pi.

- Is `flow_rate_x/y` (vs `flow_x/y` pixels) what ArduPilot 4.7's MAV flow backend reads? The
  docs say rate. Confirm in SITL or in `AP_OpticalFlow_MAV.cpp`.
- SITL needs about 20 s after boot for the EKF ("Need Position Estimate"); the flight script
  retries takeoff. On the real drone the same error means no origin or no flow; see
  flight-setup.md.
- `MavVehicle` reports `flow_quality` from what the Pi last forwarded. With no flow process
  running, it's 0, so the gate refuses takeoff. That's intended on the real drone; for SITL
  without flow, lower `flow_quality_min` or feed fake flow.
- The FC speed limit comes from `MAV_CMD_DO_CHANGE_SPEED` at takeoff. Also set
  `WPNAV_SPEED=50`, `WPNAV_SPEED_UP=30`, `WPNAV_SPEED_DN=30` on the FC.
- Set `FS_GCS_ENABLE` on the FC so the FC itself lands if the edge process dies (the edge
  heartbeats as sysid 255).
- Sim camera HFOV is 60°; measure the real C270 and update `edge/video.py:HFOV_DEG`.
- Only one station can connect at a time; the newest connection wins. The cloud dashboard
  must go through the station (or be the station), not connect separately.
