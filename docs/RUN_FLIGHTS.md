# Running the real drone from any laptop (flights + voice)

For the cloud lead's laptop or anyone else's. Written 2026-11-22 after the first field tests.
Windows: use `python`. macOS: use `.venv/bin/python` (or activate the venv). Everything below
runs **from the laptop**; the drone side is updated by the tools themselves.

## What works today (field-tested 2026-11-22)

- **Computer takeoff works** with the rangefinder deciding the height (`auto_hover --rng`): it
  left the ground and climbed to 0.6 m. Why it's needed: the flight controller's own height
  estimate drifts up while the drone sits on the ground with the motors spinning, so a normal
  takeoff stopped short of lift-off every time.
- **Sideways hold on the down camera failed in the air:** optical-flow quality fell to 0 at about
  80% throttle (vibration), the FC's EKF failsafe switched to LAND and it drifted. So outdoors
  we fly on **GPS for sideways position, rangefinder for height** (`--gps` below). GPS mode is
  built and tested on the simulator but **has not flown yet**: the first GPS flight is the plain
  hover.
- The drone needs about 80% throttle to hover (heavy for 3S). The FC is set for that
  (`MOT_THST_HOVER=0.65`, stays on the FC). Lighter (power bank off, Pi on a 5 V BEC) would help a lot.
- Every flight is recorded on the Pi: telemetry in `logs/blackbox/flight_*.jsonl`, camera frames
  in `logs/blackbox/cam_*/`.

## One-time setup on the laptop

```sh
git pull
uv venv --python 3.11 .venv && uv pip install --python .venv -e '.[dev]'   # includes paramiko
```
`.env` needs the Vultr and ElevenLabs keys (same as for the dashboard). `EDGE_HOST` doesn't
matter; the commands below take the drone's address.

**Always `git pull` before going out:** `field_setup` copies *this laptop's* `edge/`, `shared/`
and `tools/` onto the drone. An old checkout would put old code on it.

**Only one laptop program may talk to the drone at a time.** Stop `station.dev`, the station,
the dashboard and any other laptop's voice page first (the drone accepts one connection; the
newest one wins and the others keep getting kicked).

## 1. Get on the same network as the drone

- **Indoors:** the `bootie` Wi-Fi. The drone is usually at `192.168.68.131`.
- **Outdoors:** a phone hotspot named exactly **`QNX`** (the QNX image's built-in network; Anton
  has the password). The Pi joins it on its own when `bootie` isn't around. On an iPhone, turn on
  *Maximize Compatibility*. The laptop joins the same hotspot.

Power the drone (Pi on). It takes about a minute to boot and join; the drone stack starts by
itself at power-on.

## 2. Find the drone and allow this laptop

```sh
python -m tools.field_setup            # indoors / optical flow
python -m tools.field_setup --gps      # outdoors on GPS (after step 3)
python -m tools.field_setup --demo     # no-fly demo mode (indoor demos: refuses every flight)
```
It finds the Pi on this network, copies this laptop's code to it, lets this laptop command it,
restarts the drone stack and prints the drone's address (call it `<pi>` below). `--find-only`
just prints the address.

## 3. Outdoors only: switch the flight controller to GPS (once per session)

```sh
python -m tools.deploy_pi --host <pi> --run "python3 -m tools.fc_profile --gps --apply --reboot"
```
GPS sideways, rangefinder height, failsafes unchanged. The FC reboots (about 30 s). Then run
`field_setup --gps` (step 2). Leave the drone **under open sky for 1-3 minutes** for a GPS fix;
the preflight waits for 3D fix, 8+ satellites and HDOP <= 1.5.

**Back indoors:** `... --run "python3 -m tools.fc_profile --apply --reboot"` (the indoor profile
turns GPS off again), then `field_setup` without `--gps`.

## 4. Fly

### With voice (for filming)
```sh
python -m tools.voice_flights --edge <pi> --gps      # drop --gps indoors
```
Open **http://127.0.0.1:8200** (on this laptop, so the browser mic works without HTTPS; allow the
mic). Press **Preflight**, wait for the green "preflight GO", then hold **Space** (or the pink
button), talk, release. Replies play in the browser (pair the JBL with this laptop first).

| Say something like | It flies |
|---|---|
| "Hey drone, rise" / "take off and hover" | 0.5 m, hold 10 s, land |
| "Rise, turn around, land" / "do a 360" | rise, one full turn, land |
| "Rise, go left, then right, then come down" | rise, 5 m left, 5 m right, land |
| "Stop" / "land" while flying | lands now (so does the LAND button) |

A new preflight is needed before every flight. The page shows the drone's camera, what it heard,
which flight it picked, Jarvis's reply, the flight log and live charts (real height vs the FC's
estimate, the track from above, battery).

### From the dashboard (one page for everything; not flown yet, see STATUS.md)
```sh
python -m tools.demo_laptop --edge <pi> --rng
```
Open **http://127.0.0.1:8100**: hold Space (or the pink button) and talk, or "Type instead".
`--rng` takes off by the rangefinder like `auto_hover --rng` (0.5 m), holds from it, lands if it's
lost or above 1.2 m. Run the preflight above first. STOP / Esc always works.

### The four filming tests from the dashboard (station `--rng`)
Phone hotspot with no IPv4 on the laptop: see `tools/v6_tunnel.py` (SSH tunnel over IPv6).
1. "Hey drone, rise" ... "land".
2. "Hey drone, rise and do a 360" (no land): one full turn, then it hovers. Say "land" while it
   hovers (or mid-turn) to show it listens mid-flight.
3. "Hey drone, rise, go 2 meters to the left, take a photo, then 2 meters to the right" ... "Land."
   Sideways moves are capped at 2 m per command in `--rng` (5 m in legs wandered). It turns to
   face each move first. Needs a FULL pack: on a tired one it sinks while tilting (~80% hover
   throttle) and the guard lands it ("I'm sinking").
   Photos any time, flying or not: "Yo, take a photo." (full-screen flash on the dashboard).
4. Person ahead: "Hey drone, rise"; a person stands ~2.5 m in front of the camera; "fly forward
   2 metres" must be refused by the gate ("Safety says no: person 2.5 m ahead"), dashboard
   shows Refused, the drone doesn't move. Then "land".
   **Before test 4, prove the detector on the ground** (drone disarmed): the person walks in front
   of the camera and `logs/vision.log` on the Pi must show their distance. The detector has never
   run with the real model on the Pi before; if it doesn't see the person, a forward move WOULD
   fly toward them. Never stand inside the 2 m path during the flight; stand at ~2.5 m.

### Without voice
```sh
python -m tools.auto_hover --host <pi> --rng --test hover       # then spin, then leftright
```
Run the preflight first: `python -m tools.deploy_pi --host <pi> --run "python3 -m tools.preflight --min-batt 11.2 --gps"`
(drop `--gps` indoors). Only fly on `>>> GO <<<`.

**Test order:** hover, then spin, then leftright, and only move on if the previous one was
clean. Every test lands by itself; it also lands early if the rangefinder loses the ground for
1 s, reads above 1.2 m, or the FC leaves GUIDED.

## Safety (every flight)

- A **pilot on the FlySky with a thumb on SwA**: up = AltHold (normal), **down = land now**. The
  computer never fights the pilot.
- Tethered, people **behind** the drone (sideways moves turn to face their path first; a person
  within about 1 m of the end of a move gets it refused), about 6 m clear to its left.
- Light wind only. Pavement / court / packed dirt beats grass (the ultrasonic sensor hates grass).
- Charge the battery: preflight wants >= 11.2 V at rest; the edge refuses new moves below
  10.0 V and lands below 9.6 V (under load; was 10.6 / 10.2 until 2026-11-22: packs sag ~1.7 V at hover).

## After a flight

```sh
QNX_HOST=<pi> python -m tools.flight_report --pi          # macOS / Linux
$env:QNX_HOST='<pi>'; python -m tools.flight_report --pi   # Windows PowerShell
```
Height, flow quality, vibration, battery sag, tilt, the FC's messages. Camera frames:
`/data/home/qnxuser/hope/logs/blackbox/cam_<time>/` on the Pi.

## When something's wrong

| Symptom | Fix |
|---|---|
| `field_setup`: drone not found | Pi powered? Wait a minute. Laptop on the same network as the Pi? |
| Voice page: "drone not connected" | Something else holds the connection (station, dashboard, another laptop). Stop it |
| Preflight: `RC not found`, throttle / SwA NO-GO | Transmitter on, throttle down, SwA up |
| Preflight: `Battery 1 low voltage failsafe` | It latches after the pack was unplugged: `python -m tools.deploy_pi --host <pi> --reboot-fc`, wait 30 s, preflight again |
| Preflight: no camera frames / flow 0 on the ground | `ls /dev/sensor` on the Pi lacks `camera5`/`camera6`: `python -m tools.deploy_pi --host <pi> --restart` |
| Preflight: GPS fix NO-GO | Open sky, wait; the fix improves over the first few minutes |
| It flies but drifts badly | Indoors: light the floor, textured ground. Outdoors: use `--gps` |

More detail on the whole system: [HANDOVER.md](HANDOVER.md).
