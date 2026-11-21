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
| MAVLink vehicle (GUIDED, arm, takeoff, position targets, origin, telemetry, `OPTICAL_FLOW` forwarding, GCS heartbeat) | `edge/mav.py` | **NOT tested yet**: no SITL run so far |
| Entry point + `SCHED_FIFO` flag | `edge/__main__.py` | manual smoke test with `--sim` |
| Protocol REPL | `tools/cmd.py` | not run interactively yet |

The test suite has 20 passing tests (`.venv/bin/python -m pytest -q`).

## Next for Person A, in order

1. **SITL run of `edge/mav.py`.** ArduPilot Copter SITL in Docker, then `python -m edge --mav
   tcp:127.0.0.1:5760`; fly takeoff/move/yaw/land from `tools.cmd`. Expect issues with arming
   checks (`ARMING_SKIPCHK`), the EKF origin and `landed` detection.
2. **Planner** (`station/planner.py`): `mission(where, what, after)` → primitive sequence using
   `shared/client.py`. Include scan-until-found with a stub `locate()`
   (`station/localize_stub.py`, reading `edge.video.SIM_WORLD`).
3. **LLM** (`station/brain.py`): tool-call schema for `mission()`, personality, refusals.
   Provider: Vultr Serverless Inference (OpenAI-compatible); keep it swappable.
4. **Voice loop**: mic → VAD → Whisper → brain → TTS → Nest Mini.
5. **Vultr cloud + dashboard** (`cloud/`).
6. **QNX port**: copy `edge/` and `shared/` to the Pi, install pymavlink (`--no-deps`; check
   pyserial on QNX), run with `--rt-prio`, restart on kill, jitter graph from
   `EdgeServer.loop_stats`.

## Open questions / known gaps

- Is `flow_rate_x/y` (vs `flow_x/y` pixels) what ArduPilot 4.7's MAV flow backend reads? The
  docs say rate. Confirm in SITL or in `AP_OpticalFlow_MAV.cpp`.
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
