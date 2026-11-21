# Handover: drone side (as of 2026-11-21 evening)

For anyone picking this up cold, human or Claude. Read this, then [TODO.md](TODO.md) and
[STATUS.md](STATUS.md). The plan and the A↔B contract are in [PRD.md](PRD.md); the end-to-end
wiring is in [flow.md](flow.md); the QNX facts verified on the board are in [qnx.md](qnx.md).

## Where things stand

**Verified on the real drone:**
- Preflight is ALL GO (`tools/preflight.py`).
- Motors: order and direction are correct (`tools/motor_test.py`, props off).
- The computer arms and lands it from the laptop.
- Manual tethered AltHold hover: it lifts.
- The black box records every flight.

**Two autonomous hover tries, neither lifted** (details in TODO.md under "tethered GUIDED hover"):
1. The baro read prop wash as a ~1 m climb while the rangefinder said 0.16 m, so the FC cut
   throttle on the ground. There was also a violent tilt from a 3.7°/4.2° level offset and a
   back-left-heavy frame.
2. The tilt was fixed (level calibration and balance: 0° tilt, all motors equal), but the baro
   still faked a climb. `auto_hover`'s height cross-check caught it and landed.

**Fix applied for try 3 (not yet flown):**
- `EK3_SRC1_POSZ=2`: the rangefinder is the main height source.
- `EK3_RNG_USE_HGT=70`.
- Foam over the FC baro (done by the pilot).
- The real-drone ceiling is 1.5 m.

**Next action:** charge the batteries, then:

```
python -m tools.system_check
python -m tools.deploy_pi --run "python3 -m tools.preflight"
python -m tools.auto_hover
python -m tools.flight_report --pi
```

Only run `auto_hover` if the preflight says ALL GO; `flight_report` analyses the flight from the
black box.

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

- **SSH:** `qnxuser` / `qnxuser` (root password `root`, `sudo` works) at **192.168.68.131** on the
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

The indoor demo: talk to the laptop, Jarvis answers through the JBL (Bluetooth to the laptop),
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

- **FC failsafes:** `FS_GCS_ENABLE=5` (Land), `FS_GCS_TIMEOUT=2`, `FS_EKF_ACTION=1`.
  Battery: `BATT_LOW_VOLT 10.5`. `BATT_AMP_OFFSET=0.82`: the uncalibrated current sensor read
  33 A at idle. `BATT_AMP_PERVLT` still needs calibrating against the charger's mAh.
- **Edge slow-and-low profile** (`edge/__main__.py`, whenever it runs with `--mav`):
  - ceiling 1.5 m, takeoff ≤ 1.2 m, ≤ 2 m per move;
  - 0.5 m/s horizontal, climb and descent; accel 2, jerk 5;
  - flow quality ≥ 50; battery: refuse moves below 10.6 V, land below 10.2 V;
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

## Open items: who does what next (2026-11-21 evening)

The indoor demo: talk to the laptop, Jarvis answers on the JBL, the drone (demo mode) judges
every command with its real safety gate but never flies. Run it with
`python -m station --edge 192.168.68.131 --local --mic` (Enter to talk, Enter to send; needs
`pip install sounddevice`). Verified with typed commands against the real drone.

**Drone lead (Anton): the software-heavy and drone-heavy work**
- [ ] QNX demo on screen against the real drone (with Claude): dashboard on the laptop with the
      live safety-loop jitter, supervisor restarts (`slay` a process, watch it come back),
      jitter under a CPU hog with and without real-time priority, slog2 live.
- [ ] Rehearse the voice demo with your own voice (only typed commands are tested).
- [ ] Person detector distance: stand at 1, 2 and 3 m in front of the forward camera.
      Inference on the Pi measured: 50-80 ms, 7.5-8 Hz.
- [ ] Measure the forward C270's horizontal field of view (45 deg assumed): tape + one photo.
- [ ] Transmitter on, preflight fully GO.
- [ ] Wire the forward HC-SR04 (Arduino D11/D12; code ready), then `--require-obstacle-sensor`.
- [ ] When flying is allowed: hover try 3 (spike filter in), a real move, Wi-Fi pull, braking,
      `BATT_AMP_PERVLT`. Demo mode off first (`deploy_pi --stations ...` without `--demo`).
- [ ] Devpost: the QNX / on-board half.

**Cloud lead (teammate): station/ and cloud/**
- [ ] Demo wording: a refused command is spoken as the brain's "On it." and then "Safety says
      no: I'd love to turn around...". Make it one line: skip the brain's reply when the edge
      refuses, and drop the "Safety says no:" prefix for demo-mode lines (the edge reason
      already says it; demo refusals carry `(demo)` in the `safety_reject` event).
- [ ] `locate()` / "fly to the door": `check_ai` vision fails (no usable box). Test on a real
      frame from `http://192.168.68.131:8080/forward.jpg` before changing the prompt.
- [ ] `rule_parse` reads "50 cm" as 50 m (the gate catches it, still a bug).
- [ ] Dashboard has no auth since the PIN was removed: fine on the laptop, not on a public VM.
- [ ] Cloud VM deploy (only needed if judges use their own phones).
- [ ] Vultr demo rehearsal (docs/demo.md) and the Devpost cloud half (docs/devpost-cloud.md).
- Keys: replaced 2026-11-21 evening, `check_ai` 13/15 PASS, voice to response est. 1.8 s.

## Rules that bit us

- Commit in small commits on `main`; pull with `--rebase` before pushing (both people push).
- No Claude attribution in commits.
- Don't change FC failsafes or disable safety checks without the pilot's explicit OK.
- Never mention the drone's earlier history in this repo; describe it only as hardware.
