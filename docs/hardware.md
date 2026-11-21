# Hardware

Everything the team has to work with. We'll go through it together and tick **Confirmed**
for each item once it's checked in person.

## Drone

| Part | Details | Confirmed |
|---|---|---|
| Frame | F450 quad frame, separate power distribution board (PDB) | ☐ |
| Motors | 4× 2212 920KV | ☐ |
| ESCs | 4× 30A | ☐ |
| Props | 1045 (10"), plus spares | ☐ |
| Flight controller | Matek H743-SLIM, ArduPilot 4.7.2 (Copter) | ☐ |
| GPS + compass | HGLRC M100-PRO (QMC5883L compass). Useless indoors; the compass may need to be off indoors | ☐ |
| Power module | Analog voltage/current sensor, XT60 in and out | ☐ |
| Batteries | 2× Zeee 3S LiPo, 3000 mAh, XT60 | ☐ |
| Logging | microSD card (Netac P500 32 GB) | ☐ |

## Links

| Part | Details | Confirmed |
|---|---|---|
| RC transmitter + receiver | FlySky FS-i6X + FS-iA6B (iBUS). SwA up = Stabilize, SwA down = Land | ☐ |
| Wi-Fi MAVLink bridge | ESP8266 module (V2.0, external antenna) wired to the flight controller's UART1 pads (`SERIAL2`). Broadcasts a network called `Drone`. **Not working yet:** its firmware baud rate is unknown, so `SERIAL2_BAUD` has to be matched first. This is the laptop ↔ drone link | ☐ |

## Cameras and compute

| Part | Details | Confirmed |
|---|---|---|
| iPhone XR | Drone's forward camera, streaming video to the laptop | ☐ |
| Webcam | ArUco tracking camera (model: ?) | ☐ |
| GPU laptop | RTX PRO 2000 Blackwell (8 GB VRAM), Core Ultra 9 285H, 30 GB RAM | ☐ |
| Second laptop | Ground station and dev | ☐ |
| QNX Raspberry Pi 5 | **Borrowed from the QNX booth.** Get it on day 1 | ☐ |
| Arduino kit | Contents: ? (list what's in it) | ☐ |

## Power and tools

| Part | Details | Confirmed |
|---|---|---|
| Charger | B3 PRO balance charger | ☐ |
| Cell checker | BX100 | ☐ |
| Smoke stopper | For first power-ups | ☐ |
| USB power bank | Would power the Pi onboard (Plan B). Have one? | ☐ |
| Soldering iron, wire, heat shrink | ? | ☐ |

## Missing

| Part | Why it matters | Workaround |
|---|---|---|
| Optical flow / lidar sensor | Indoor position hold | ArUco + webcam (see README) |
| Prop guards | 10" props near people | Cleared flying area, people behind a line, Land switch ready |
| SiK telemetry radio | Long-range MAVLink | ESP8266 Wi-Fi bridge |
