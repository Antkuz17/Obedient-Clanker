"""Keep QNX's camera service (`sensor`) running. Run as root, under the supervisor.

  python3 -m tools.sensor_keeper

`sensor` daemonizes, so the supervisor can't watch it directly. When it dies (seen on the Pi
2026-11-21: USB RING_OVERRUN on a camera endpoint, then the service was gone), its
/dev/sensor/cameraN entries vanish and every camgrab fails `camera_open` until someone restarts
it by hand. This loop notices the entries are gone and starts the service again; the camgrabs
retry on their own and come back.
"""
from __future__ import annotations

import argparse
import logging
import os
import subprocess
import time

log = logging.getLogger("sensor_keeper")
CMD = ["sensor", "-U", "521:521", "-b", "external", "-r", "/data/share/sensor",
       "-c", "/etc/config/sensor/sensor_rpi5.conf"]


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--watch", default="/dev/sensor/camera5",
                    help="path the camera service publishes while it is alive")
    ap.add_argument("--grace", type=float, default=2.0, help="seconds missing before a restart")
    a = ap.parse_args()
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(name)s %(message)s")
    log.info("watching %s", a.watch)
    missing_since = None
    while True:
        if os.path.exists(a.watch):
            missing_since = None
        elif missing_since is None:
            missing_since = time.monotonic()
        elif time.monotonic() - missing_since >= a.grace:
            log.warning("%s gone for %.0f s: camera service died, restarting it", a.watch,
                        a.grace)
            try:
                r = subprocess.run(CMD, stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL,
                                   stderr=subprocess.DEVNULL, timeout=20)  # it daemonizes
                log.info("camera service started (exit %d)", r.returncode)
            except (OSError, subprocess.TimeoutExpired) as e:
                log.error("could not start the camera service: %s", e)
            missing_since = None
            time.sleep(5.0)  # let it publish its cameras before checking again
        time.sleep(1.0)


if __name__ == "__main__":
    main()
