"""On-board person detector (edge/vision) and the forward ultrasonic path, without a real model:
a fake TFLite interpreter returns SSD-shaped outputs."""
import asyncio
import json

import numpy as np
import pytest

from edge.config import SafetyConfig, WatchdogConfig
from edge.safety import Perception, SafetyGate
from edge.server import EdgeServer
from edge.sim import SimVehicle
from edge.vehicle import VehicleState
from edge.vision import Detection, PersonDetector, focal_px, nearest_person, person_distance
from edge.vision.__main__ import run
from shared import protocol as P
from shared.client import EdgeClient
from tools.rangefinder import parse_line

W, H = 640, 480


# --- distance --------------------------------------------------------------------------------
def test_whole_body_distance_from_height():
    f = focal_px(W, 45.0)
    d = Detection(0.4, 0.2, 0.6, 0.7, 0.9)            # 240 px tall, clear of the edges
    assert person_distance(d, W, H) == pytest.approx(1.7 * f / 240)


def test_cut_off_body_uses_width_and_errs_closer():
    """At 1 m the C270 frame is ~0.62 m tall, so a near person is always cut off: height
    would read them as ~2.7 m away. Width (narrow shoulders) must say about 1 m or less."""
    f = focal_px(W, 45.0)
    shoulders_at_1m = 0.45 * f / W                      # real shoulders, 1 m away
    d = Detection(0.5 - shoulders_at_1m / 2, 0.0, 0.5 + shoulders_at_1m / 2, 1.0, 0.9)
    assert person_distance(d, W, H) <= 1.0
    assert person_distance(d, W, H) > 0.7


def test_nearest_person():
    assert nearest_person([], W, H) is None
    far = Detection(0.45, 0.3, 0.55, 0.5, 0.9)
    near = Detection(0.2, 0.0, 0.8, 1.0, 0.8)
    assert nearest_person([far, near], W, H) == person_distance(near, W, H)


# --- detector with a fake interpreter ---------------------------------------------------------
class FakeInterpreter:
    """SSD post-process outputs, listed in a shuffled order like newer TFLite builds do."""

    def __init__(self, boxes, classes, scores, named=True, dtype=np.uint8):
        n = len(scores)
        self.tensors = {
            10: np.array([boxes + [[0, 0, 0, 0]] * (10 - n)], np.float32),
            11: np.array([classes + [0] * (10 - n)], np.float32),
            12: np.array([scores + [0] * (10 - n)], np.float32),
            13: np.array([n], np.float32),
        }
        names = {10: "TFLite_Detection_PostProcess", 11: "TFLite_Detection_PostProcess:1",
                 12: "TFLite_Detection_PostProcess:2", 13: "TFLite_Detection_PostProcess:3"}
        order = [12, 10, 13, 11]
        self.out = [{"index": i, "name": names[i] if named else f"out{i}",
                     "shape": np.array(self.tensors[i].shape)} for i in order]
        if not named:  # unnamed: same shapes for classes and scores, keep the standard order
            self.out = [{"index": i, "name": f"out{i}", "shape": np.array(self.tensors[i].shape)}
                        for i in (10, 11, 12, 13)]
        self.dtype = dtype
        self.input = None

    def get_input_details(self):
        return [{"index": 0, "shape": np.array([1, 300, 300, 3]), "dtype": self.dtype}]

    def get_output_details(self):
        return self.out

    def set_tensor(self, i, v):
        self.input = v

    def invoke(self):
        pass

    def get_tensor(self, i):
        return self.tensors[i]


@pytest.mark.parametrize("named", [True, False])
def test_detector_keeps_confident_people_only(named):
    it = FakeInterpreter(
        boxes=[[0.2, 0.4, 0.7, 0.6], [0.1, 0.1, 0.3, 0.2], [0.0, 0.0, 1.0, 1.0]],
        classes=[0, 62, 0],          # person, chair, person
        scores=[0.8, 0.9, 0.3],      # the second person is below min_score
        named=named)
    people = PersonDetector(it, min_score=0.5).detect(np.zeros((H, W, 3), np.uint8))
    assert len(people) == 1
    p = people[0]
    assert (p.x0, p.y0, p.x1, p.y1, p.score) == pytest.approx((0.4, 0.2, 0.6, 0.7, 0.8), abs=1e-6)
    assert it.input.shape == (1, 300, 300, 3) and it.input.dtype == np.uint8


def test_float_models_get_normalised_input():
    it = FakeInterpreter([], [], [], dtype=np.float32)
    PersonDetector(it).detect(np.full((H, W, 3), 255, np.uint8))
    assert it.input.dtype == np.float32 and it.input.max() == pytest.approx(1.0)


class FakeFrame:
    def __init__(self, seq):
        self.seq, self.t, self.width, self.height = seq, 1000.0 + seq, W, H

    def bgr(self):
        return np.zeros((H, W, 3), np.uint8)


class FakeReader:
    def __init__(self, frames):
        self.frames = list(frames)

    def wait_new(self, after_seq=-1, timeout=1.0):
        return self.frames.pop(0) if self.frames else None


def test_loop_reports_each_frame_with_its_capture_time_and_goes_quiet_without_frames():
    it = FakeInterpreter([[0.0, 0.2, 1.0, 0.8]], [0], [0.9])
    reader = FakeReader([FakeFrame(1), None, FakeFrame(2)])
    sent = []
    calls = {"n": 0}

    def stop():
        calls["n"] += 1
        return calls["n"] > 3
    run(reader, PersonDetector(it), sent.append, hz=0, hfov=45.0, stop=stop)
    assert [m["t"] for m in sent] == [1001.0, 1002.0]          # nothing for the missing frame
    assert all(m["person_m"] is not None and m["person_m"] < 1.5 for m in sent)
    assert all(m["palm"] is False and "infer_ms" in m for m in sent)


# --- forward ultrasonic -----------------------------------------------------------------------
def test_rangefinder_lines():
    assert parse_line(b"d 412\n") == ("d", 0.412)
    assert parse_line(b"f -1\r\n") == ("f", -1.0)
    assert parse_line(b"f 750") == ("f", 0.75)
    for bad in (b"x 1", b"f", b"f abc", b""):
        assert parse_line(bad) is None


FLYING = VehicleState(connected=True, armed=True, mode="GUIDED", alt=1.0, battery_v=12.0,
                      flow_quality=200, landed=False)


def test_gate_refuses_moves_that_end_too_close():
    g = SafetyGate()
    wall = Perception(obstacle_m=1.5, obstacle_alive=True)
    assert g.check("move_body", {"fwd": 0.5}, FLYING, True, wall).ok          # ends 1.0 m off
    assert "ahead" in g.check("move_body", {"fwd": 1.0}, FLYING, True, wall).reason
    assert g.check("move_body", {"fwd": -1.0}, FLYING, True, wall).ok         # backing away
    # people: the estimate may be 30 % too far, so 2.0 m reads as 1.54 m of room
    person = Perception(person_m=2.0, detector_alive=True)
    assert g.check("move_body", {"fwd": 0.4}, FLYING, True, person).ok          # 1.14 m left
    assert "person" in g.check("move_body", {"fwd": 0.8}, FLYING, True, person).reason


def test_gate_can_require_the_forward_sensor():
    g = SafetyGate(SafetyConfig(require_obstacle_sensor=True))
    assert "obstacle sensor" in g.check("move_body", {"fwd": 0.5}, FLYING, True,
                                        Perception()).reason
    assert g.check("move_body", {"fwd": 0.5}, FLYING, True,
                   Perception(obstacle_alive=True)).ok


async def test_wall_ahead_stops_a_running_forward_move():
    cfg = SafetyConfig()
    sim = SimVehicle(cfg, speedup=4.0)
    srv = EdgeServer(sim, cfg, WatchdogConfig(), host="127.0.0.1", port=0, perception_port=None,
                     flow_port=None)
    await srv.start()
    edge = await EdgeClient("127.0.0.1", srv.port).connect()
    try:
        assert (await edge.run("takeoff", alt=1.0))["status"] == P.OK
        srv._on_flow(json.dumps({"fwd_m": -1}).encode())          # clear ahead
        cid, ack = await edge.send("move_body", fwd=2.5)
        assert ack["ok"]
        await asyncio.sleep(0.1)
        srv._on_flow(json.dumps({"fwd_m": 0.5}).encode())         # wall appears
        d = await edge.wait_done(cid, 10)
        assert d["status"] == P.ABORTED and "ahead" in d["reason"]
        # the reading goes stale after OBSTACLE_STALE_S: unknown again, not "wall"
        await asyncio.sleep(0.7)
        assert srv.exec.perception.obstacle_m is None
        assert not srv.exec.perception.obstacle_alive
    finally:
        await edge.close()
        await srv.close()
