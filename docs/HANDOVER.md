# Handover: drone side

**To fly the real drone from any laptop (field setup, GPS, voice flights): [RUN_FLIGHTS.md](RUN_FLIGHTS.md).**

For anyone picking this up cold, human or Claude. Read this, then [STATUS.md](STATUS.md) (what
works) and [TODO.md](TODO.md) (what's open). The design and the laptop ↔ drone contract are in
[PRD.md](PRD.md); the end-to-end wiring is in [flow.md](flow.md); the QNX facts verified on the
board are in [qnx.md](qnx.md).

## Where things stand

- **Flown outdoors** on GPS (sideways) + rangefinder (height), by voice from the dashboard:
  takeoff, hover, a full turn, "land" mid-flight. Sideways legs work on a full pack and sink
  on a tired one (about 80% throttle to hover).
- **Indoors on optical flow it has not held position in the air:** flow quality fell to 0 at
  hover throttle (vibration). Indoors, use demo mode (below): nothing flies.
- Computer takeoff uses the rangefinder to decide the height (`--rng`): the flight
  controller's own estimate drifts up on the ground with the motors spinning.
- The black box records every flight (`tools/flight_report.py --pi`).

## Who does what

- **Drone lead (Anton):** everything under `edge/`, the QNX side, FC parameters, hardware and
  flight tests.
- **Cloud lead (teammate):** `station/` (brain, planner, voice) and `cloud/` (Vultr, dashboard,
  memory).
- The boundary between them is protocol v1 (`shared/protocol.py`). See CLAUDE.md.

## Hardware as built

- **Airframe:** F450, 2212 920KV motors, 30A ESCs, 1045 props, 3S 3000 mAh packs.
- **FC:** Matek H743-SLIM running ArduPilot 4.7.2.
- **Companion:** a QNX 8 Raspberry Pi 5 on board, on a USB power bank. The Pi's USB ports:
  - **Controller 0:** the forward C270 (camera unit 5) and the Arduino (Elegoo Uno, `2341:0043`).
  - **Controller 1:** the FC (`1209:5740`) and the down C270 (camera unit 6).
- **Height sensor:** an HC-SR04 pointing down, wired to the Arduino (Trig D9, Echo D10). Sketch:
  `tools/hcsr04/hcsr04.ino`, uploaded with arduino-cli.
- **Pilot:** FlySky FS-i6X. **SwA on channel 5: up = AltHold, down = Land.** The computer enters
  GUIDED itself; flipping SwA always takes over.
- **Keep the Arduino insulated from the ESP8266 Wi-Fi module.** When they touched, the
  Arduino's USB died until the two were separated. The ESP8266 is unused.

## Getting into the Pi

- **SSH:** the QNX quick-start image's default login (the tools read `QNX_USER` / `QNX_PASS`;
  `sudo` works; **change the defaults before using a shared network**) at **192.168.68.131** on the
  `bootie` Wi-Fi. `qnxpi.local` does not resolve on Windows.
- **Code location:** `/data/home/qnxuser/hope`.
- **If it vanishes from the network:** put a script at `/boot/onboot` on the SD card. It runs as
  root at boot. See qnx.md.
- **`python -m tools.deploy_pi`** (needs paramiko on the laptop):
  - With no options, it syncs `edge/`, `shared/` and `tools/` (changed files only).
  - `--build` rebuilds the C programs camgrab and hbwd.
  - `--stations IP1,IP2` writes `supervisor_local.json` with the laptops that may command the
    drone. Currently that's `.130` (Anton) and `.100` (teammate).
  - `--run "<cmd>"` runs a command in `~/hope` with the right `PYTHONPATH`.
  - `--autostart on|off` installs or removes the boot auto-start.

## Running the drone stack (on the Pi)

`start_sup.sh` (a copy of `tools/start_pi.sh`) does two things as root:

1. It restarts QNX's camera service, which clears ghost camera streams.
2. It runs `tools/supervisor.py supervisor_local.json`. The supervisor starts these children and
   restarts any that die, logging each restart:

| Child | What it does |
|---|---|
| `sensor_keeper` (root) | Restarts QNX's camera service (`sensor`) if it dies (its `/dev/sensor/camera5` vanishes) |
| `usb_relay` (root) | Owns the FC's USB. UDP `:14555` goes to the core; `--aux :14556` is a write-only inlet for hbwd, flow and rangefinder; `--tap :14557` is a two-way port for tools |
| `rangefinder` (root) | Arduino over USB → `DISTANCE_SENSOR` to the FC (direct) and to the edge. Holds prop-noise spikes (`SpikeFilter`; `--no-filter` turns it off) |
| `hbwd` (root, C) | 50 Hz FIFO watchdog: lands the drone if the core hangs |
| `edge` | Protocol v1 on TCP `:7000`, safety gate, executor, watchdog |
| `camgrab_fwd`, `camgrab_down` (C, libcamapi) | Write frames to shared memory |
| `flow` | Optical flow from the down camera → `OPTICAL_FLOW` to the FC (direct). Runs with `--flip-x --flip-y`, verified by the hand-roll check |
| `vision` | TFLite person detector on the forward camera → UDP `:7001` |
| `video` | MJPEG on `:8080` (`/forward.jpg` carries an `X-Pose` header) |
| `blackbox` | Records flights to `logs/blackbox/` through the tap |

**Priorities (SCHED_FIFO):** hbwd 60, relay 50, edge 45, rangefinder 42 (light I/O) stay above
the USB and Wi-Fi drivers (21-24); flow 20, camgrab 19/18, vision 11, video 10, blackbox 5 sit
below them. With flow/vision/camgrab at 40-44 they starved the drivers: USB `RING_OVERRUN` on a
camera, Wi-Fi SDIO timeouts and the camera service dying (2026-11-21). Keep heavy compute below 21.

**Restart everything:**

```
sudo sh -c 'slay -f camgrab; slay -f hbwd; slay -f python3; nohup ~/hope/start_sup.sh >/dev/null 2>&1 </dev/null &'
```

- Each child's log is in `~/hope/logs/<name>.log`; supervisor events are in `logs/supervisor.out`.
- **The stack starts at boot** (`/boot/onboot`, installed by `deploy_pi --autostart on`): edge up
  about 55 s after power-on. Log: `/boot/onboot.log`. Off: `touch ~/hope/NO_AUTOSTART`.
  The script relaunches itself with `on -d`: a plain `&` job is killed when mount_fs.sh exits.

## Demo mode (no flying indoors)

Indoors: talk to the laptop, Jarvis answers through the JBL (Bluetooth to the laptop),
the drone never moves. Turn it on with
`python -m tools.deploy_pi --stations 192.168.68.130,192.168.68.100 --demo`, then restart the
stack; turn it off by running the same without `--demo`. In demo mode (`edge/demo.py`):

- Every flight command still goes through the real safety gate with real sensor data. If the
  gate refuses, Jarvis says the real reason ("move of 3.0 m exceeds the 2.0 m limit"). If it
  clears, Jarvis gives a sarcastic "I would, but we're indoors" line. Nothing reaches the FC.
- Say "take off" first. A cleared takeoff starts a pretend hover (telemetry: armed, 1 m, mode
  `GUIDED-DEMO`) so moves are judged as if flying; "land" ends it.
- Verified on the real drone 2026-11-21: take off / turn around / move back / forward /
  "move right 3 m" (real gate refusal) / land; the FC never left STABILIZE.
- `auto_hover` and every takeoff are refused while it's on: turn it off for flight tests.

## QNX demo (on the laptop, against the real drone)

1. `python -m tools.demo_laptop` on Anton's laptop (.130), then open http://127.0.0.1:8100.
   Dashboard: drone video, telemetry, the QNX card, typed or push-to-talk commands; replies
   play in the browser (the JBL is the laptop's speaker).
2. **Live safety-loop timing:** the edge streams its 50 Hz loop stats (p99 about 2 ms).
3. **Kill a process:** on the Pi, `kill -9` the vision process (or `slay`): the card shows it
   die and the supervisor respawn it in about 30-55 ms (verified).
4. **slog2:** on the Pi, `slog2info -w -b hope` shows every start, crash, restart and safety
   refusal live (edge/slog2d; verified).
5. **Real-time vs normal priority** (`tools/jitter.py --compare --udp <laptop>:7010`, as root).
   Measured on the Pi with no load: normal priority p99 14.0 ms / max 19.0 ms; SCHED_FIFO 50
   p99 1.2 ms / max 2.2 ms. **Warning:** the CPU-hog cases (all 4 cores pegged) took the Pi off
   the network on 2026-11-21 with the whole stack running on the USB power bank (likely power;
   not confirmed). Don't run them on the flying stack: wall power, stack stopped, or fewer hogs.

## Tools (laptop unless noted)

| Tool | Purpose |
|---|---|
| `tools/system_check.py` | PASS/FAIL for every link: Pi, the 10 processes, cameras, detector, preflight, video, command path, Vultr AI, cloud |
| `tools/preflight.py` (Pi) | GO / NO-GO before every flight |
| `tools/fc_profile.py` (Pi) | Check / apply / back up the FC parameter profile (indoor flow + rangefinder; `--outdoor` exists). `--apply --reboot` |
| `tools/auto_hover.py` | Computer-flown takeoff → hold → land with a report. Lands itself if the FC height disagrees with the rangefinder |
| `tools/flight_report.py` | Analyses a black-box flight (`--pi` fetches the newest one) |
| `tools/flow_signcheck.py` (Pi) | Hand-roll check of the optical-flow axes |
| `tools/compass_cal.py` (Pi) | Onboard compass calibration (rotate the drone) |
| `tools/motor_test.py` (Pi) | Props-off motor order and direction |
| `tools/cmd.py` | Protocol REPL (`takeoff alt=0.5`, `move_body right=0.5`, `land`) |
| `tools/sitl_flight.py`, `tools/sitl_flowonly.py` | ArduPilot SITL in Docker (`hope-sitl` image) |

## Safety configuration (the real drone)

- **FC failsafes** (all in `tools/fc_profile.py`, checked by the preflight): `FS_GCS_ENABLE=5`
  (Land), `FS_GCS_TIMEOUT=2`, `FS_EKF_ACTION=1`, `FS_THR_ENABLE=3` (transmitter lost → Land;
  bench-check the receiver's failsafe), `BATT_LOW_VOLT=10.5` with `BATT_FS_LOW_ACT=1` (Land)
  on sag-compensated voltage. `BATT_AMP_OFFSET=0.82`: the uncalibrated current sensor read
  33 A at idle. `BATT_AMP_PERVLT` still needs calibrating against the charger's mAh.
- **Edge slow-and-low profile** (`edge/__main__.py`, whenever it runs with `--mav`):
  - ceiling 1.5 m, takeoff ≤ 1.2 m, ≤ 2 m per move;
  - 0.5 m/s horizontal and descent, 1.0 m/s climb; accel 2, jerk 5;
  - flow quality ≥ 50 (not checked with `--gps`); battery under load: refuse moves below
    10.0 V, land below 9.6 V;
  - forward moves refused unless the person detector is alive.
  - `--full-limits` opts out; only do that once the drone has proven itself.
- **The edge never fights the pilot.** It only acts in GUIDED or a LAND it started itself. The
  station watchdog hovers after 1 s and lands after 5 s with no laptop heartbeat.

## Known problems / gotchas (all found on the real hardware)

- **Verified 2026-11-21 (no flying):** boot auto-start; full `system_check` PASS on the drone
  side; with the edge core frozen, the FC still gets flow (21 Hz, q 251) and the rangefinder
  (10 Hz) directly, and hbwd sees the gap without landing a disarmed drone.

- **Camera stalls / camera service dying:** root cause found 2026-11-21: our real-time
  children starved the USB driver (see Priorities above). Fixed by the new priorities; the
  first start after a reboot now usually has no stall, and `sensor_keeper` restarts the
  service if it ever dies. If stalls come back, suspect power next (the Pi's 600 mA USB limit:
  a 5 V/5 A PD bank and `usb_max_current_enable=1` in `/boot/config.txt`).
- **The battery "low capacity failsafe" latches** whenever the FC stays powered over USB during a
  pack swap. Fix: reboot the FC.
- **The relay goes stale after an FC reboot.** It now exits after 5 s of silence and the
  supervisor restarts it, in under 1 s.
- **Compass:** the "mag field xy diff" pre-arm comes back if power wires move near the mast.
  Run `tools/compass_cal.py`.
- **The down camera runs at 8–15 fps in dim light**, and flow quality drops while moving. Light
  the floor.
- **Windows-only test failures:** test_camera restarted-grabber, test_jitter rate and
  test_supervisor killed-child. They pass on macOS and are not Pi bugs.

## Open items

See [TODO.md](TODO.md).

## Rules that bit us

- Commit in small commits on `main`; pull with `--rebase` before pushing (both people push).
- No Claude attribution in commits.
- Don't change FC failsafes or disable safety checks without the pilot's explicit OK.
- Never mention the drone's earlier history in this repo; describe it only as hardware.
