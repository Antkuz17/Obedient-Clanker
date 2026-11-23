# Hardware

As built. Flight controller parameters are in `tools/fc_profile.py`; wiring and QNX details are
in [HANDOVER.md](HANDOVER.md) and [qnx.md](qnx.md).

## Drone

| Part | Details |
|---|---|
| Frame | F450 quad, separate power distribution board |
| Motors / ESCs | 4× 2212 920KV, 4× 30 A |
| Props | 1045 (10"), **no guards** |
| Flight controller | Matek H743-SLIM, ArduPilot Copter 4.7.2 |
| GPS + compass | HGLRC M100-PRO (QMC5883L compass): sideways position outdoors |
| Power | Analog voltage/current module; 2× Zeee 3S LiPo 3000 mAh, XT60 |
| Weight | Heavy for 3S: about 80% throttle to hover |

## On-board computer and sensors

| Part | Details |
|---|---|
| Companion computer | Raspberry Pi 5 (8 GB) running QNX 8, on a USB power bank |
| Link to the flight controller | USB (libusb from a root relay process; QNX has no driver for it) |
| Forward camera | Logitech C270 (camera unit 5): person detection, video, photos. About 45° horizontal field of view (assumed from the 55° diagonal spec; not measured) |
| Down camera | Logitech C270 (camera unit 6), 320×240: optical flow |
| Height | HC-SR04 ultrasonic, pointing down, on an Arduino Uno (Trig D9, Echo D10) → USB → Pi → flight controller. About 2 m range; poor over grass |
| Forward obstacle sensor | Second HC-SR04 on the same Arduino (D11/D12): code ready, **not wired** |

## Pilot and ground

| Part | Details |
|---|---|
| Transmitter + receiver | FlySky FS-i6X + FS-iA6B (iBUS). **SwA (channel 5): up = AltHold, down = Land.** The computer enters GUIDED itself; flipping SwA takes over |
| Ground station | A laptop on the same Wi-Fi as the Pi (a phone hotspot in the field) |
| Speaker | Bluetooth speaker on the laptop for Jarvis's voice |
| Charger / checker | B3 PRO balance charger, BX100 cell checker |

## Not fitted

| Part | Why it matters | Today |
|---|---|---|
| Prop guards | 10" props near people | Cleared area, people behind the drone, tether, pilot on the Land switch |
| Motor kill switch | Every failsafe is "land", which keeps the props spinning until touchdown | Pilot's Land switch only |
| ESP8266 Wi-Fi bridge | — | On the frame, unused. **Keep the Arduino insulated from it** (contact killed the Arduino's USB) |
