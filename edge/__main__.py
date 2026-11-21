"""Run the edge core.

  python -m edge --sim                      simulated drone + fake room video (dev, no hardware)
  sudo python3 -m edge --mav usb             real FC on the QNX Pi (USB, needs root)
  python -m edge --mav tcp:127.0.0.1:5760   ArduPilot SITL
"""
from __future__ import annotations

import argparse
import asyncio
import logging
import os
from pathlib import Path

from shared import protocol as P

from .config import SafetyConfig
from .server import EdgeServer


def set_realtime(prio: int) -> None:
    """SCHED_FIFO for the safety core. Works on QNX and Linux (needs root); ignored elsewhere."""
    try:
        os.sched_setscheduler(0, os.SCHED_FIFO, os.sched_param(prio))
        logging.info("SCHED_FIFO priority %d", prio)
    except (AttributeError, PermissionError, OSError) as e:
        logging.warning("could not set SCHED_FIFO (%s); running at normal priority", e)


async def main(a) -> None:
    cfg = SafetyConfig()
    if a.sitl:
        cfg.flow_quality_min = 0  # SITL has no optical flow; this run only
    if a.mav and not a.sitl and not a.no_person_check:
        cfg.require_person_detector = True  # real drone: no detector -> no forward moves
    if a.sim:
        from .sim import SimVehicle
        vehicle = SimVehicle(cfg, speedup=a.speedup)
    else:
        from .mav import MavVehicle
        vehicle = MavVehicle(a.mav, baud=a.baud, speed_xy=cfg.speed_xy)

    srv = EdgeServer(vehicle, cfg, port=a.port, log_path=Path(a.log) if a.log else None)
    await srv.start()

    if not a.no_video:
        from .video import SimCamera, UsbCamera, serve_video
        cam = SimCamera(vehicle.state) if a.sim and a.camera is None else \
            UsbCamera(vehicle.state, a.camera or 0)
        serve_video(cam, port=a.video_port)
        logging.info("video on http://0.0.0.0:%d/forward.mjpg", a.video_port)
    await asyncio.Event().wait()


if __name__ == "__main__":
    ap = argparse.ArgumentParser(prog="edge")
    src = ap.add_mutually_exclusive_group(required=True)
    src.add_argument("--sim", action="store_true", help="simulated vehicle")
    src.add_argument("--mav", help="MAVLink device: usb (QNX Pi), /dev/ser1, tcp:127.0.0.1:5760 (SITL)")
    ap.add_argument("--sitl", action="store_true",
                    help="with --mav: ArduPilot SITL, skip the optical-flow quality check")
    ap.add_argument("--no-person-check", action="store_true",
                    help="real drone only: allow forward moves with no person detector running")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--port", type=int, default=P.EDGE_PORT)
    ap.add_argument("--video-port", type=int, default=P.VIDEO_PORT)
    ap.add_argument("--camera", type=int, default=None, help="USB camera index (real video)")
    ap.add_argument("--no-video", action="store_true")
    ap.add_argument("--speedup", type=float, default=1.0, help="sim only")
    ap.add_argument("--log", default="flight_log.jsonl", help="JSONL flight log ('' = off)")
    ap.add_argument("--rt-prio", type=int, default=0, help="SCHED_FIFO priority (0 = off)")
    a = ap.parse_args()
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(name)s %(message)s")
    if a.rt_prio:
        set_realtime(a.rt_prio)
    try:
        asyncio.run(main(a))
    except KeyboardInterrupt:
        pass
