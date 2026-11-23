# To-do

Last updated: 2026-11-22. What works is in [STATUS.md](STATUS.md).

## Before the next flight

- [ ] **Drone lead: review the 2026-11-22 fixes in `edge/` and `tools/`** (STATUS table, rows 6,
      8, 10) and run `tools.sitl_flight`, `tools.sitl_flowonly`, `tools.sitl_hbwd` (ALL PASS).
- [ ] **Apply the new failsafe parameters** with the pilot's OK: on the Pi,
      `python3 -m tools.fc_profile --gps` (check), then `--gps --apply`. The preflight is NO-GO
      until then.
- [ ] **Bench-check the transmitter failsafe:** transmitter off must raise "Radio Failsafe" on
      the flight controller. If it doesn't, set the FS-iA6B's failsafe so throttle drops below
      `FS_THR_VALUE`.
- [ ] Change the Pi's default passwords.

## Never verified on the real drone

- [ ] Pull the laptop's Wi-Fi mid-hover: hover after 1 s, land after 5 s.
- [ ] Stop the edge core in flight: the C watchdog commands LAND; the FC's own failsafe lands
      without it.
- [ ] Braking distance (`tools/sitl_brake.py` works against the FC too); compare with
      `stop_react_s` / `brake_decel` in `edge/config.py`.
- [ ] Person detector: distance at 1, 2 and 3 m; latency (`person_latency_s`, assumed 0.3 s).
- [ ] The forward C270's horizontal field of view (45° assumed).
- [ ] `BATT_AMP_PERVLT` calibration (the battery failsafe uses sag-compensated voltage, which
      depends on it).
- [ ] Find the root cause of the uncommanded sink + yaw in GUIDED (black box flights of
      2026-11-22).

## Hardware

- [ ] Lighter airframe or a bigger battery (about 80% throttle to hover today); Pi on a 5 V BEC
      instead of the power bank.
- [ ] Wire the HC-SR04 height sensor straight to the flight controller (`RNGFND1_TYPE=30`,
      [flight-setup.md](flight-setup.md)), so a dead Pi doesn't take the height source with it.
- [ ] Wire the forward HC-SR04 (Arduino D11/D12; code ready), then run the edge with
      `--require-obstacle-sensor`.
- [ ] A motor kill switch on the transmitter.
- [ ] Vibration damping for the down camera, then retry indoor flow.

## Software

- [ ] Decide what optical flow is for outdoors: fuse it as a fallback to GPS (a second EKF
      source set) or stop running it in `--gps` mode.
- [ ] Move the sink / yaw-drift / lost-rangefinder guards from the laptop onto the edge.
- [ ] Check short sideways and backward steps for people (a second sensor, or always turn to
      face the path).
- [ ] `locate()` and "fly to the X" on real frames; measure the size prior.
- [ ] Palm stop.
- [ ] Deploy the cloud app to a Vultr VM ([cloud/deploy.md](../cloud/deploy.md)); it needs
      authentication on the dashboard first.
- [ ] Rules for flying a drone over 250 g outdoors in Canada (registration, pilot certificate,
      distance from bystanders, controlled airspace).
