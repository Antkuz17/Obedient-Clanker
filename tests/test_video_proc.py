"""Video as its own process: the core streams its pose over UDP; frames carry that pose."""
import asyncio
import socket
import subprocess
import sys
import time
import urllib.request

import pytest

from edge.config import SafetyConfig, WatchdogConfig
from edge.server import EdgeServer
from edge.sim import SimVehicle
from edge.video import PoseReceiver, SimCamera, serve_video
from shared import protocol as P
from shared.client import EdgeClient


def free_port(kind=socket.SOCK_STREAM) -> int:
    with socket.socket(socket.AF_INET, kind) as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


def get_jpg(port):
    with urllib.request.urlopen(f"http://127.0.0.1:{port}/forward.jpg", timeout=5) as r:
        return r.headers, r.read()


async def test_pose_stream_drives_the_video_pose():
    pose = PoseReceiver(0)
    vport = free_port()
    http = serve_video(SimCamera(pose.state), "127.0.0.1", vport, pose_age=pose.age)
    cfg = SafetyConfig()
    sim = SimVehicle(cfg, speedup=8.0)
    srv = EdgeServer(sim, cfg, WatchdogConfig(), host="127.0.0.1", port=0, perception_port=None,
                     flow_port=None, video_addr=("127.0.0.1", pose.port))
    await srv.start()
    edge = await EdgeClient("127.0.0.1", srv.port).connect()
    try:
        await edge.run("takeoff", alt=1.0)
        await edge.run("move_body", fwd=1.0)
        await asyncio.sleep(0.3)
        hdr, jpg = await asyncio.to_thread(get_jpg, vport)
        n, e, alt, yaw, t = map(float, hdr["X-Pose"].split(","))
        assert (n, e, alt) == pytest.approx((sim.s.n, sim.s.e, sim.s.alt), abs=0.1)
        assert float(hdr["X-Pose-Age"]) < 0.2 and jpg[:2] == b"\xff\xd8" and pose.received > 10
        await edge.close()
        await srv.close()               # the core goes away: video keeps serving, pose ages
        await asyncio.sleep(0.6)
        hdr, jpg = await asyncio.to_thread(get_jpg, vport)
        assert float(hdr["X-Pose-Age"]) > 0.5 and jpg[:2] == b"\xff\xd8"
    finally:
        http.shutdown()
        pose.close()


def test_video_runs_as_its_own_process():
    vport, pport = free_port(), free_port(socket.SOCK_DGRAM)
    p = subprocess.Popen([sys.executable, "-m", "edge.video", "--camera", "sim", "--host",
                          "127.0.0.1", "--port", str(vport), "--pose-port", str(pport)])
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        deadline = time.monotonic() + 10
        while True:
            s.sendto(b'{"n": 1.5, "e": -0.5, "alt": 1.0, "yaw": 90.0, "armed": true}',
                     ("127.0.0.1", pport))
            try:
                hdr, jpg = get_jpg(vport)
                if hdr["X-Pose"].startswith("1.500,-0.500,1.000,90.0"):
                    break
            except OSError:
                pass
            assert time.monotonic() < deadline, "video process never served the sent pose"
            time.sleep(0.1)
        assert jpg[:2] == b"\xff\xd8" and float(hdr["X-Pose-Age"]) < 1.0
    finally:
        p.terminate()
        p.wait(5)
