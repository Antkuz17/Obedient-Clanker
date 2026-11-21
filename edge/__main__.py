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

from .config import SafetyConfig, max_checked_speed
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
    if a.sitl or a.gps:
        # SITL has no optical flow; with --gps the FC navigates on GPS, so flow isn't needed
        cfg.flow_quality_min = 0
    if a.mav and not a.sitl and not a.no_person_check:
        cfg.require_person_detector = True  # real drone: no detector -> no forward moves
    if a.mav and not a.sitl and not a.full_limits:
        # Slow-and-low profile for the real drone (drone lead, 2026-11-21), indoors and out,
        # until it has proven itself: 2 m ceiling, gentle speeds, and a 2 m cap per move (which
        # also catches a misread unit: "50 cm" parsed as 50 m flew 16.7 m in sim).
        cfg.alt_max, cfg.max_takeoff_alt, cfg.max_step = 1.5, 1.2, 2.0  # rangefinder height: stay < 2 m
        cfg.speed_xy, cfg.speed_z, cfg.speed_dn = 0.5, 0.5, 0.5
        cfg.accel_xy, cfg.jerk_xy = 2.0, 5.0
        logging.info("slow-and-low profile: ceiling 1.5 m, takeoff <= 1.2 m, move <= 2 m, 0.5 m/s")
    if cfg.speed_xy > (fastest := max_checked_speed(cfg)):
        # the person / obstacle checks couldn't stop it in time any faster (edge/config.py)
        logging.warning("speed_xy %.2f m/s is faster than the checks can stop; using %.2f",
                        cfg.speed_xy, fastest)
        cfg.speed_xy = fastest
    if a.require_obstacle_sensor:
        cfg.require_obstacle_sensor = True  # forward HC-SR04 mounted: silent sensor = no fwd
    if a.sim:
        from .sim import SimVehicle
        vehicle = SimVehicle(cfg, speedup=a.speedup)
    else:
        from .mav import MavVehicle
        vehicle = MavVehicle(a.mav, baud=a.baud, speed_xy=cfg.speed_xy, speed_z=cfg.speed_z,
                             speed_dn=cfg.speed_dn, gps=a.gps, accel_xy=cfg.accel_xy,
                             jerk_xy=cfg.jerk_xy)

    srv = EdgeServer(vehicle, cfg, port=a.port, perception_port=a.perception_port,
                     flow_port=a.flow_port, log_path=Path(a.log) if a.log else None,
                     allowed_peers=set(a.station_ip) if a.station_ip else None,
                     hbwd_addr=(a.hbwd.rsplit(":", 1)[0], int(a.hbwd.rsplit(":", 1)[1]))
                     if a.hbwd else None,
                     video_addr=(a.video_proc.rsplit(":", 1)[0], int(a.video_proc.rsplit(":", 1)[1]))
                     if a.video_proc else None,
                     stats_addr=(a.stats_udp.rsplit(":", 1)[0], int(a.stats_udp.rsplit(":", 1)[1]))
                     if a.stats_udp else None)
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
    ap.add_argument("--gps", action="store_true",
                    help="outdoors: the FC navigates on GPS (tools/fc_profile.py --outdoor). No "
                         "fake EKF origin, no optical-flow requirement, no 2 m height cap")
    ap.add_argument("--full-limits", action="store_true",
                    help="real drone: use edge/config.py's limits instead of the slow-and-low "
                         "profile (2 m ceiling, 0.5 m/s). Only once the drone has proven itself")
    ap.add_argument("--require-obstacle-sensor", action="store_true",
                    help="refuse forward moves while the forward ultrasonic sensor is silent "
                         "(turn on once it is mounted)")
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
    ap.add_argument("--stats-udp", help="send QNX demo stats (loop jitter, stop latency) as JSON "
                    "to host:port, e.g. <laptop>:7010 for the dashboard (off by default)")
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
