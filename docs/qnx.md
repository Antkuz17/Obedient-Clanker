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

## Pi 5 setup (QNX quick-start image)

- **Login:** `qnxuser` / `qnxuser`. Root password `root` (`su`).
- **SSH:** `ssh qnxuser@qnxpi.local`. On Windows add `-m hmac-sha2-256`.
- **Wi-Fi:** `wpa_supplicant.conf` on the SD card's `/boot` partition; the interface is `bcm0`.
- **Packages:** `apk`. Fix the clock first (`ntpdate`), then run:
  `apk update && apk add opencv-dev tflite-runtime tflite-runtime-dev python3-numpy`.
  The preinstalled list is in `/etc/apk/world`.
- **Python:** 3.11. Add pip with `python -m ensurepip --upgrade`. Install pymavlink with
  `pip install --no-deps pymavlink` (it falls back to a pure-Python CRC).
- **Camera:** QNX's AI workshop has verified the C270 and C920. A USB camera is
  Sensor Framework unit 5. Start from github.com/qnx/QNX-AI-workshop (C++).

## Link to the flight controller

- **Primary: Pi 5 GPIO UART → flight controller UART7 pads (`Tx7`/`Rx7`, `SERIAL1`).**
  Both sides are 3.3 V. Wire Pi TX → `Rx7`, Pi RX → `Tx7`, GND → `G`.

  ```
  msix-rp1 -m set 25l
  gpio-rp1 set 14,15 a4
  devc-serpl011-rpi5 -b115200 -v -c50000000 -e -F -u1 0x1f00030000,185
  ```

  That gives `/dev/ser1` on physical pins 8/10. If those pins hold the console
  (`pidin ar | grep devc-ser`), use UART2 on GPIO4/5 instead.
  Flight controller: `SERIAL1_PROTOCOL = 2`, `SERIAL1_BAUD = 115`, flow control off.
- **Fallback: USB cable** to the flight controller's USB-C port, via `devc-serusb` →
  `/dev/serusb1`. Check `which devc-serusb` first.
- **Pi 4 (Linux) fallback** if QNX blocks the onboard role: the Pi 4 flies, and the QNX Pi runs
  the safety gate on the ground. Weaker for the track.

## Ask at the QNX booth

1. Is `devc-serusb` on the image? Is the console on pins 8/10 on the Pi 5?
2. Are tflite-runtime, opencv and python3-numpy preinstalled?
3. Are the High Availability Manager and adaptive partitioning on the image? (Demo moments 3
   and 4 depend on them.)

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
