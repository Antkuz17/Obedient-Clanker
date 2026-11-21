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
        vehicle = MavVehicle(a.mav, baud=a.baud, speed_xy=cfg.speed_xy, speed_z=cfg.speed_z)

    srv = EdgeServer(vehicle, cfg, port=a.port, perception_port=a.perception_port,
                     flow_port=a.flow_port, log_path=Path(a.log) if a.log else None,
                     allowed_peers=set(a.station_ip) if a.station_ip else None,
                     hbwd_addr=(a.hbwd.rsplit(":", 1)[0], int(a.hbwd.rsplit(":", 1)[1]))
                     if a.hbwd else None,
                     video_addr=(a.video_proc.rsplit(":", 1)[0], int(a.video_proc.rsplit(":", 1)[1]))
                     if a.video_proc else None)
    await srv.start()

    if not a.no_video and not a.video_proc:
        from .video import ShmCamera, SimCamera, UsbCamera, serve_video
        if a.camera is None:
            cam = SimCamera(vehicle.state) if a.sim else ShmCamera(vehicle.state)
        elif a.camera.startswith("shm:"):
            cam = ShmCamera(vehicle.state, a.camera[4:])
        else:
            cam = UsbCamera(vehicle.state, int(a.camera))
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
    ap.add_argument("--station-ip", action="append",
                    help="only accept station connections from this IP (repeatable). "
                         "Always set this on the real drone (review A9)")
    ap.add_argument("--allow-any-station", action="store_true",
                    help="bench only: accept a station from any IP")
    ap.add_argument("--perception-port", type=int, default=7001, help="UDP from vision proc")
    ap.add_argument("--flow-port", type=int, default=7002, help="UDP from flow proc")
    ap.add_argument("--camera", default=None,
                    help="forward video: shm:NAME (camgrab; default on --mav) or a cv2 index "
                         "(laptops). Default with --sim: the fake room")
    ap.add_argument("--no-video", action="store_true")
    ap.add_argument("--speedup", type=float, default=1.0, help="sim only")
    ap.add_argument("--log", default="flight_log.jsonl", help="JSONL flight log ('' = off)")
    ap.add_argument("--rt-prio", type=int, default=0, help="SCHED_FIFO priority (0 = off)")
    ap.add_argument("--video-proc", help="video runs as its own process (python -m edge.video); "
                    "send it the pose at this UDP address, e.g. 127.0.0.1:7003")
    ap.add_argument("--hbwd", help="pet the C heartbeat watchdog at this UDP address, "
                    "e.g. 127.0.0.1:14560 (edge/hbwd)")
    a = ap.parse_args()
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(name)s %(message)s")
    if a.mav and not a.sitl and not a.station_ip and not a.allow_any_station:
        ap.error("on the real drone, pass --station-ip <laptop ip> so nobody else on the "
                 "Wi-Fi can fly it (review2 #13), or --allow-any-station for a bench test")
    if a.rt_prio:
        set_realtime(a.rt_prio)
    try:
        asyncio.run(main(a))
    except KeyboardInterrupt:
        pass
