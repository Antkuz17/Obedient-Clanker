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
| `usb_relay` (root) | Owns the FC's USB. UDP `:14555` goes to the core; `--aux :14556` is a write-only inlet for hbwd, flow and rangefinder; `--tap :14557` is a two-way port for tools |
| `rangefinder` (root) | Arduino over USB → `DISTANCE_SENSOR` to the FC (direct) and to the edge |
| `hbwd` (root, C) | 50 Hz FIFO watchdog: lands the drone if the core hangs |
| `edge` | Protocol v1 on TCP `:7000`, safety gate, executor, watchdog |
| `camgrab_fwd`, `camgrab_down` (C, libcamapi) | Write frames to shared memory |
| `flow` | Optical flow from the down camera → `OPTICAL_FLOW` to the FC (direct). Runs with `--flip-x --flip-y`, verified by the hand-roll check |
| `vision` | TFLite person detector on the forward camera → UDP `:7001` |
| `video` | MJPEG on `:8080` (`/forward.jpg` carries an `X-Pose` header) |
| `blackbox` | Records flights to `logs/blackbox/` through the tap |

**Restart everything:**

```
sudo sh -c 'slay -f camgrab; slay -f hbwd; slay -f python3; nohup ~/hope/start_sup.sh >/dev/null 2>&1 </dev/null &'
```

- Each child's log is in `~/hope/logs/<name>.log`; supervisor events are in `logs/supervisor.out`.
- The stack does **not** start at boot yet. `deploy_pi --autostart on` is written but untested.

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

- **Camera stalls ("Timeout waiting for USB buffer", both at once):** most likely the Pi's
  600 mA total USB limit without a 5 V/5 A supply.
  - Fix: a 5V/5A PD power bank and `usb_max_current_enable=1` in `/boot/config.txt`.
  - Meanwhile, restart the stack; camgrab exits on a stall and the supervisor retries.
- **QNX's camera service died once** for an unknown reason. Restarting the stack brings it back.
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

**Cloud lead:**
- The Vultr key returns 401 on every AI call, so voice is down. Regenerate it and update `.env`.
- The dashboard PIN was removed (`/text` and `/voice` have no auth).
- `rule_parse` reads cm as m.
- The cloud VM has not been deployed.
- Audio out and voice latency are untested.

**Drone lead:**
- Autonomous hover try 3.
- Verify the person detector's inference time and distance on the Pi.
- slog2 logging (needs testing on the Pi; don't write it blind).
- Test the boot auto-start.
- Measure the C270 HFOV (45° assumed).
- Check the direct sensor path with the edge killed (review3 #8).
- Measure braking.
- A second (forward) HC-SR04: the code is ready (D11/D12), but the sensor is missing.
- QNX demo items: latency on screen, slog2 live.
- The slideshow, after a successful hover.

## Rules that bit us

- Commit in small commits on `main`; pull with `--rebase` before pushing (both people push).
- No Claude attribution in commits.
- Don't change FC failsafes or disable safety checks without the pilot's explicit OK.
- Never mention the drone's earlier history in this repo; describe it only as hardware.
