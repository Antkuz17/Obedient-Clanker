# Indoor flight setup: software optical flow + ultrasonic height

No GPS indoors and no optical-flow sensor. The Pi computes optical flow from the down webcam and
sends it to ArduPilot as if it were a real flow sensor. An HC-SR04 gives height. ArduPilot's own
BlueOS OpticalFlow extension does the same thing; we reuse its pixel-to-radians math.

## Optical flow (Pi → ArduPilot)

- MAVLink **`OPTICAL_FLOW` (#100)**; ArduPilot ignores `OPTICAL_FLOW_RAD`.
- Fill **`flow_rate_x/y` in rad/s**: `dpix / (px_per_rad * dt)`.
- `quality` 0–255: derive it from the tracked-feature (LK inlier) count, never a constant 255.
- Send raw angular flow. ArduPilot subtracts its own gyro unless `FLOW_OPTIONS` bit 0 is set.
  If rotation leaks in as false motion, derotate on the Pi with timestamped `ATTITUDE` /
  `RAW_IMU` and set `FLOW_OPTIONS = 1`.
- Gaps over 0.5 s zero the flow. Send at camera rate (30 Hz; try 320×240 for speed).
- Check signs with a hand-roll test over textured ground: `OF.flowX` should track `OF.bodyX`.

## Height (HC-SR04 → flight controller, direct)

ArduPilot supports the HC-SR04 natively. No Arduino or Pi needed.

| Parameter | Value |
|---|---|
| `RNGFND1_TYPE` | 30 (HC-SR04) |
| `RNGFND1_PIN` | Echo GPIO |
| `RNGFND1_STOP_PIN` | Trigger GPIO |
| `RNGFND1_MIN` | 0.2 |
| `RNGFND1_MAX` | 2 |
| `RNGFND1_ORIENT` | 25 (down) |

Set `SERVOx_FUNCTION = -1` on the two pins used. Which pads on the H743-SLIM are free for this
is still **TBD**.

- **Echo is 5 V and the board is 3.3 V: use a voltage divider.**
- Mount it away from the props on foam, since prop noise causes spikes. It has about 2 m of
  range; carpet and angled surfaces drop echoes.
- Fallback: the Pi reads it and sends `DISTANCE_SENSOR` with `RNGFND1_TYPE = 10`, distances in
  cm, `orientation = 25`, at 10–20 Hz or more.

## ArduPilot parameters

| Parameter | Value | Note |
|---|---|---|
| `FLOW_TYPE` | 5 | MAVLink |
| `FLOW_OPTIONS` | 0 | 1 if the Pi derotates |
| `EK3_FLOW_DELAY` | 100–150 | ms; tune from logs |
| `FLOW_ALT_MIN` | 0.3 | |
| `EK3_SRC1_POSXY` | 0 | |
| `EK3_SRC1_VELXY` | 5 | optical flow |
| `EK3_SRC1_POSZ` | 1 | baro |
| `EK3_SRC1_VELZ` | 0 | |
| `EK3_SRC1_YAW` | 1 | compass |
| `GPS1_TYPE` | 0 | |
| `SERIAL1_PROTOCOL` | 2 | Pi link |
| `ARMING_SKIPCHK` | skip GPS | 4.7 replaced `ARMING_CHECK` |
| `FLOW_FXSCALER` / `FLOW_FYSCALER` | tune | Calibrate the C270's FOV |

- **EKF origin is required.** The Pi sends `SET_GPS_GLOBAL_ORIGIN` at startup; without it you
  get "Need Position Estimate".
- **Compass:** heading still comes from the compass. Recalibrate at the venue, including the
  motor-interference calibration (`COMPASS_MOT`).
- **GUIDED:** with the origin set and flow working, `SET_POSITION_TARGET_LOCAL_NED` velocity and
  position commands work.

## Risks

1. **Latency:** frames are about 100 ms old, so tilt can read as motion and cause wobble.
   Derotate on the Pi and tune `EK3_FLOW_DELAY`.
2. **Floor:** it needs texture and bright light. Bring a patterned mat if the floor is plain.
3. **C270:** auto-exposure blur and rolling shutter. Use good lighting and a damped camera mount.
4. **Drift:** this is velocity only, so position slowly drifts. Fine for short commands.
5. **QNX porting:** prototype on the Pi 4 (Linux) first, then move to QNX.

## Sources

- [AP_OpticalFlow_MAV.cpp](https://raw.githubusercontent.com/ArduPilot/ardupilot/master/libraries/AP_OpticalFlow/AP_OpticalFlow_MAV.cpp)
- [BlueOS OpticalFlow](https://ardupilot.org/copter/docs/common-blueos-opticalflow.html) and its [code](https://github.com/BlueOS-community/blueos-opticalflow)
- [Optical flow setup](https://ardupilot.org/copter/docs/common-optical-flow-sensor-setup.html)
- [HC-SR04 in ArduPilot](https://ardupilot.org/copter/docs/common-rangefinder-hcsr04.html)
- [Non-GPS position / EKF origin](https://ardupilot.org/dev/docs/mavlink-nongps-position-estimation.html)
- [4.7 parameter renames](https://ardupilot.org/copter/docs/common-param-name-changes.html)
