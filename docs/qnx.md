# QNX track

## Prize wording

> Build an embedded system that uses AI. Your project must run on QNX OS and use at least one
> open-source AI module from QNX. Judges will look for a use case where real-time performance
> or reliability matters, a creative use of AI, and a demo running on embedded hardware.

## How we fit

| Requirement | Our answer |
|---|---|
| Embedded system that uses AI | The QNX Pi 5 rides **on the drone** as its flight computer |
| Must run on QNX | Everything the drone does in flight runs on QNX: both cameras, optical flow, the safety gate, the watchdog and MAVLink to the flight controller |
| Open-source AI module from QNX | **TensorFlow Lite** from qnx-ports runs person detection on board, with QNX's **OpenCV** port for optical flow. The Devpost write-up names and links both |
| Real-time / reliability matters | 30 Hz optical flow feeds position hold; person detection stops the drone; the watchdog lands it if the link dies; every command passes the safety gate |
| Demo on embedded hardware | A drone flying with QNX inside it |

**Rule:** the AI that matters for safety runs on the Pi. The cloud LLM is a convenience layer.
With the laptop or cloud dead, the drone still holds position and lands safely.

## What runs on the QNX Pi

1. **Down webcam:** OpenCV optical flow, sent to ArduPilot as an `OPTICAL_FLOW` sensor (see
   [flight-setup.md](flight-setup.md)).
2. **Forward webcam:** TFLite person detection (SSD-MobileNet / EfficientDet-Lite at low
   resolution). A person too close blocks the move or backs the drone off. Frames also stream
   to the laptop for the LLM tasks.
3. **Safety gate:** checks every laptop command against altitude, a fence box, battery and
   person distance, and logs the reason for any rejection.
4. **Watchdog:** if the laptop heartbeat stops, the drone hovers, then lands.
5. Optional: palm = stop, using the hand-landmark model from QNX's AI workshop (about 1–2 h).

Use real-time priorities (`SCHED_FIFO`): watchdog and safety first, flow next, AI last.

## Demo moments

1. **Pull the laptop's Wi-Fi mid-hover.** The watchdog lands the drone; show the timestamp log.
2. **A person steps in front.** On-board AI stops the drone; show the inference latency.
3. **Kill the vision process live** (`slay`). It restarts while the safety loop keeps running.
4. **Max out the CPU.** The control loop's jitter graph stays flat.
5. **Tell the AI "fly up 10 m".** The safety gate rejects it and shows the reason.

## Verified on our Pi (2026-11-21)

- QNX 8.0.0 on a Raspberry Pi 5: 4× Cortex-A76, 8 GB RAM, 17 GB free on `/data`. Internet works.
- **SSH:** `ssh qnxuser@192.168.68.131` on the `bootie` Wi-Fi (`qnxpi.local` doesn't resolve
  on Windows). `sudo` works, and `vncserv` is running for a remote desktop.
- **Installed:** `python3-numpy` (2.4.1), `python3-opencv` (4.12, with `calcOpticalFlowPyrLK`)
  and `python3-tflite-runtime` (2.21) via `apk`; `pymavlink` and `pyusb` via
  `pip --user --no-deps`. The image has **Python 3.14**, not 3.11.
- **`devc-serusb` is not on the image and not in the repo.** Workaround: `libusb` (apk) + `pyusb`
  talk to the flight controller's USB CDC-ACM endpoints directly. This **must run as root**;
  as `qnxuser`, libusb segfaults. Load it with
  `get_backend(find_library=lambda _: "/usr/lib/libusb-1.0.so.0")`.
- **Flight controller over USB: verified.** The Pi enumerates it as `0x1209:0x5740` (ArduPilot
  MatekH743). `tools/fc_usb_probe.py` gets two-way MAVLink: heartbeats, a `COMMAND_ACK` to
  `SET_MESSAGE_INTERVAL`, and ATTITUDE at about 30 Hz. No soldering needed.
- **C270 on QNX: verified.** It enumerates as `0x046d:0x0825` and is Sensor Framework camera
  unit 5. `camera_example1_callback -u 5` streams 640×480 @ 30 fps. Frame types: YCbYCr,
  NV12, NV21, CbYCrY, YCbCr420P, YCrCb420P, RGB8888.
  - **OpenCV can't open it:** `cv2.VideoCapture(0)` fails; the only backends are GStreamer and
    uEye, and there's no QNX camera source. Frames have to come from QNX's camera API (C,
    `libcamapi`), so we need a small grabber that hands frames to Python.
  - For a one-off still, `camera_example4_record -u 5 -f mov -c none -t CbYCrY` (it needs a
    terminal; press a key to start, then to stop) writes an uncompressed MOV to
    `/data/share/sensor/`, and OpenCV on a laptop can read it. The MP4/AVC1 path segfaults.
- **Second camera (2026-11-21):** the sensor config now pins **unit 5 = C270 (`046d:0825`)** and
  **unit 6 = `046d:0x081b`** by USB ID (`/etc/config/sensor/sensor_rpi5.conf`; original saved as
  `.orig`; the unused Intel D435 example units 6/7 are commented out). After editing, restart
  the service as root: `slay -f sensor; sensor -U 521:521 -b external -r /data/share/sensor -c
  /etc/config/sensor/sensor_rpi5.conf &`. **The 081b camera (probably a C310) does not work on
  QNX:** `slog2info` shows `Missing format/frame descriptors ... numFormats 3`, then `unit 6:
  Failed to connect device interface: err = 22`. It's a UVC parser incompatibility, not
  bandwidth (it still fails on its own USB controller). **Fix: a second C270 or C920** (QNX's
  verified models); it then works as unit 6 by changing that unit's PID to `0x0825` / `0x082d`
  (two identical C270s would need bus/device numbers instead of VID/PID). Put the cameras on
  different USB controllers (`usb -v` → "Upstream Host Controller").
- **Running the whole stack (2026-11-21, verified):** `sudo nohup ./tools/start_pi.sh &` with
  `supervisor_local.json` (= `tools/supervisor_pi.json` with the laptop IP). Everything runs with
  0 crashes: relay (`--aux`, `--tap`), rangefinder, hbwd (loop max 0.7 ms), edge, camgrab ×2,
  flow (straight to the FC: 20 Hz, quality 252 at the FC), video. Gotchas found on the board:
  - **Two identical C270s** are told apart by USB bus in the sensor config (`address =
    /dev/usb/io-usb-otg, <bus>, -1, 0x046d, 0x0825`): unit 5 = bus 0 = forward, unit 6 = bus 1
    = down (verified by grabbing a frame from each).
  - **Ghost camera streams:** a camgrab that died without `camera_close` leaves the sensor
    service with a stale primary stream; new clients attach as secondary and get 0 fps
    forever. `start_pi.sh` restarts the sensor service first; camgrab now exits after 5 s with
    no frames so the supervisor retries.
  - The supervisor must keep the user's groups when dropping root (camera needs `sensor`).
  - hbwd links with `-lsocket` on QNX.
  - The C270 drops to 8–15 fps in dim light: light the floor under the drone.
- **HAM and adaptive partitioning are not available.** For demo 3 we write our own supervisor
  that restarts a crashed process; for demo 4 we use `SCHED_FIFO` priorities.
- Wi-Fi config the Pi actually reads: `/data/var/etc/settings/wpa_supplicant.conf` (seeded from
  `/boot`). The serial console is the Pi 5's 3-pin debug connector (`/dev/ser10`), not GPIO
  pins 8/10.
- If it ever disappears from the network: put a `/boot/onboot` script on the SD card. The
  image runs it as root at boot (`mount_fs.sh`), so it can write diagnostics back to `/boot`.

## Track evidence (measured on the drone, 2026-11-22)

Live on the voice page's "Running on QNX" panel (`tools/qnx_status.py`, polled over SSH) and
reproducible with `tools/qnx_faults.py`.

| Track asks for | What we show | Measured |
|---|---|---|
| Runs on QNX | The QNX Pi 5 **rides on the drone** and runs everything it does in flight: both cameras (QNX camera API, C), optical flow, the person detector, the safety gate, the C watchdog, MAVLink to the flight controller | `QNX 8.0.0 RaspberryPi5 aarch64le`, 4 cores, 13 supervised processes |
| Open-source AI module from QNX | **TensorFlow Lite** (QNX port, `apk` package `python3-tflite-runtime 2.21.0-r1`) runs the on-board person detector (SSD-MobileNet); **OpenCV** (QNX port, `python3-opencv 4.12.0-r1`, `libopencv_optflow`) runs the optical flow that feeds the flight controller | detector 7.5 Hz, 84-87 ms/frame on the Pi |
| AI where it matters | The detector's output gates flight: forward moves are refused or stopped when a person is too close, and refused entirely if the detector is down | test 4: a person walking in stopped a forward move at 1.8 m |
| Real-time performance | SCHED_FIFO priority plan: watchdog 60, USB link 50, safety core 45, rangefinder 42 **above** QNX's USB/Wi-Fi drivers (21); flow/cameras 20-18 and the AI 11 **below** them, so heavy compute can never starve a driver or the safety loop | C watchdog loop (50 Hz, FIFO 60, pinned): worst wake-up 0.27-0.98 ms, 0 late; same loop at normal priority p99 14.0 ms vs FIFO p99 1.2 ms |
| Real-time engineering | Found a 433 ms stall of the FIFO-60 loop: it printed its log to the SD card and a busy card blocked it. Moved logging to a lock-free ring + normal-priority logger thread | same SD stress: 433 ms -> **0.86 ms**, 0 late, 0 lines dropped |
| Reliability | Our supervisor (QNX has no HAM on this image) restarts any process; a C watchdog lands the drone if the core hangs; sensor_keeper restarts QNX's camera service | detector SIGKILL -> respawn **73 ms**; safety core frozen 3 s -> watchdog noticed in **260 ms**, flow 10 Hz + rangefinder 10.3 Hz kept reaching the FC; camera service killed -> back in **3.6 s**, cameras streaming in 4.6 s |
| Observability | Every start, crash, restart and safety refusal goes to QNX's system log (slog2 buffer set `hope`) | `slog2info -w -b hope` |
| Embedded demo | The drone itself; the page shows the QNX proof live | |

Earlier finding, same theme: with flow/vision/camgrab at FIFO 40-45 they starved QNX's USB and
Wi-Fi drivers (USB `RING_OVERRUN`, the camera service died); the priority plan above fixed it.

## Pi 5 setup (QNX quick-start image)

- **Login:** `qnxuser` / `qnxuser`. Root password `root` (`su`).
- **SSH:** `ssh qnxuser@qnxpi.local`. On Windows add `-m hmac-sha2-256`.
- **Wi-Fi:** `wpa_supplicant.conf` on the SD card's `/boot` partition; the interface is `bcm0`.
- **Packages:** `apk`. Fix the clock first (`ntpdate`), then run:
  `apk update && apk add opencv-dev tflite-runtime tflite-runtime-dev python3-numpy`.
  The preinstalled list is in `/etc/apk/world`.
- **Python:** 3.14, and pip is preinstalled. Install pymavlink with
  `pip install --user --no-deps --break-system-packages pymavlink` (it falls back to a
  pure-Python CRC).
- **Camera:** QNX's AI workshop has verified the C270 and C920. A USB camera is
  Sensor Framework unit 5. Start from github.com/qnx/QNX-AI-workshop (C++).

## Link to the flight controller

- **Primary: USB cable** (Pi USB-A → flight controller USB-C). **Verified 2026-11-21**, no
  soldering. There's no `devc-serusb`, so `libusb` + `pyusb` drive the CDC-ACM endpoints directly
  (data interface 1, bulk IN `0x82`, OUT `0x02`; set DTR with `SET_CONTROL_LINE_STATE`). It
  runs as root. See `tools/fc_usb_probe.py`. MAVLink over USB uses ArduPilot's `SERIAL0`.
- **Later, if we can solder: GPIO UART → flight controller UART7 pads (`Tx7`/`Rx7`,
  `SERIAL1`).** Both sides are 3.3 V. Wire Pi TX → `Rx7`, Pi RX → `Tx7`, GND → `G`. The console
  is on the debug connector, so GPIO pins 8/10 are free.

  ```
  msix-rp1 -m set 25l
  gpio-rp1 set 14,15 a4
  devc-serpl011-rpi5 -b115200 -v -c50000000 -e -F -u1 0x1f00030000,185
  ```

  That gives `/dev/ser1`. Flight controller: `SERIAL1_PROTOCOL = 2`, `SERIAL1_BAUD = 115`,
  flow control off.
- **Pi 4 (Linux) fallback** if QNX blocks the onboard role: the Pi 4 flies, and the QNX Pi runs
  the safety gate on the ground. Weaker for the track.

## Ask at the QNX booth

Answered on the board (see "Verified on our Pi"). Still worth asking:
1. Is there a GStreamer source or Python binding for the QNX camera API, so OpenCV can read it?
2. Is there an official way to get HAM or adaptive partitioning on this image?

## Sources

- [QNX AI workshop](https://github.com/qnx/QNX-AI-workshop)
- [qnx-ports build-files](https://github.com/qnx-ports/build-files)
- [QNX physical AI building blocks](https://qnx.software/en/blog/2026/how-qnx-gives-developers-the-building-blocks-for-physical-ai)
- [Pi GPIO UART on QNX](https://www.qnx.com/developers/docs/qnxeverywhere/com.qnx.doc.interfacing/topic/rpi/rpi_GPIO_UART.html)
- [devc-serusb](https://www.qnx.com/developers/docs/8.0/com.qnx.doc.neutrino.utilities/topic/d/devc-serusb.html)
- [pip on QNX 8](https://devblog.qnx.com/use-pip-with-python-on-qnx-8-0/)
- Past QNX winners: [Beats By Air](https://devpost.com/software/beats-by-air),
  [Hack the 6ix recap](https://qnx.software/en/blog/2026/hack-the-6ix-2025),
  [cuHacking 6 recap](https://qnx.software/en/blog/2026/qnx-at-cuhacking-6)
