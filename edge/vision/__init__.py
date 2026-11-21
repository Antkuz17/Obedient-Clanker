"""On-board person detection: forward camera (camgrab shared memory) -> TFLite -> the edge's
safety gate (UDP 127.0.0.1:7001, `{"person_m", "palm", "t", "infer_ms"}`).

This is the drone's reflex, not its understanding: it runs on the Pi with no network, every
frame it can, and only answers "how far is the nearest person?". The gate refuses forward moves
that would end within `person_close_m` of them and stops a running one. Finding and
understanding objects stays with the cloud vision model (station/localize.py).

Model: COCO SSD-MobileNet v1, quantised (300x300 uint8 in; boxes / classes / scores / count
out; class 0 = person). `python -m tools.fetch_person_model` puts it in edge/vision/models/,
and tools/deploy_pi.py copies it to the Pi with the rest of edge/. On QNX the runtime is the
`python3-tflite-runtime` apk (docs/qnx.md).

Distance: from the box and the camera's focal length.
- the whole body is in frame: 1.7 m / box height (the most reliable case)
- the box is cut off at the top or bottom (anyone closer than ~2.7 m with a 45 deg C270:
  the frame is only 0.62 m tall at 1 m): from the width instead, assuming narrow shoulders
  (0.4 m), so the guess errs *closer*, which is the safe side for a stop rule.
These are rough (+-30 %). Anything closer than the detector can see (a torso filling the frame)
is the forward ultrasonic sensor's job (tools/rangefinder.py `fwd_m`).

Fail-safe: if this process dies, stalls or its camera stops, the edge stops hearing from it,
marks the detector dead after 1 s and refuses forward moves (edge/safety.py).
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path

import numpy as np

try:
    import cv2
except ImportError:  # pragma: no cover - on the Pi and every laptop
    cv2 = None

MODEL_DIR = Path(__file__).parent / "models"
DEFAULT_MODEL = MODEL_DIR / "detect.tflite"
PERSON_CLASS = 0          # COCO SSD-MobileNet v1: output class 0 = "person" (labelmap line 2)
PERSON_HEIGHT_M = 1.7
SHOULDER_M = 0.4          # narrow on purpose: errs towards "closer"
EDGE_FRAC = 0.02          # a box within 2 % of the frame edge counts as cut off
C270_HFOV_DEG = 45.0      # same as edge/flow; measure the real camera and pass --hfov


@dataclass
class Detection:
    x0: float  # normalised 0-1, x right, y down
    y0: float
    x1: float
    y1: float
    score: float


def focal_px(width: int, hfov_deg: float = C270_HFOV_DEG) -> float:
    return (width / 2) / math.tan(math.radians(hfov_deg) / 2)


def person_distance(d: Detection, width: int, height: int,
                    hfov_deg: float = C270_HFOV_DEG) -> float:
    """Metres to a detected person (see the module docstring for the rules)."""
    f = focal_px(width, hfov_deg)
    h_px = max(1.0, (d.y1 - d.y0) * height)
    w_px = max(1.0, (d.x1 - d.x0) * width)
    cut_vert = d.y0 <= EDGE_FRAC or d.y1 >= 1.0 - EDGE_FRAC
    if not cut_vert:
        return PERSON_HEIGHT_M * f / h_px
    return SHOULDER_M * f / w_px


def nearest_person(dets: list[Detection], width: int, height: int,
                   hfov_deg: float = C270_HFOV_DEG) -> float | None:
    """Distance to the closest person, or None if nobody is in view."""
    if not dets:
        return None
    return min(person_distance(d, width, height, hfov_deg) for d in dets)


def load_interpreter(model_path: str | Path, threads: int = 2):
    """A TFLite interpreter from whichever runtime is installed: tflite_runtime (the QNX apk),
    ai_edge_litert (its new name on laptops), or full TensorFlow."""
    try:
        from tflite_runtime.interpreter import Interpreter
    except ImportError:
        try:
            from ai_edge_litert.interpreter import Interpreter
        except ImportError:
            try:
                from tensorflow.lite import Interpreter  # type: ignore
            except ImportError as e:
                raise RuntimeError("no TFLite runtime: on the Pi `apk add python3-tflite-runtime`"
                                   ", on a laptop `pip install ai-edge-litert`") from e
    it = Interpreter(model_path=str(model_path), num_threads=threads)
    it.allocate_tensors()
    return it


class PersonDetector:
    """SSD-style TFLite detector, person class only.

        det = PersonDetector(load_interpreter("edge/vision/models/detect.tflite"))
        people = det.detect(bgr)          # [Detection], score >= min_score
    """

    def __init__(self, interpreter, min_score: float = 0.5, person_class: int = PERSON_CLASS):
        if cv2 is None:
            raise RuntimeError("OpenCV (cv2) is required for person detection")
        self.it = interpreter
        self.min_score, self.person_class = min_score, person_class
        inp = self.it.get_input_details()[0]
        self.in_index = inp["index"]
        _, self.in_h, self.in_w, _ = (int(x) for x in inp["shape"])
        self.in_dtype = inp["dtype"]
        self.out = self._map_outputs(self.it.get_output_details())

    @staticmethod
    def _map_outputs(details: list[dict]) -> dict[str, int]:
        """SSD post-process outputs are boxes, classes, scores, count. Newer TFLite builds can
        list them in a different order, so go by name (`TFLite_Detection_PostProcess:N`) when
        the names say, else by shape, else by the standard order."""
        by_suffix = {}
        for d in details:
            name = d.get("name", "")
            if name.startswith("TFLite_Detection_PostProcess"):
                suffix = name.split(":")[1] if ":" in name else "0"
                by_suffix[suffix] = d["index"]
        if set(by_suffix) >= {"0", "1", "2", "3"}:
            return {"boxes": by_suffix["0"], "classes": by_suffix["1"],
                    "scores": by_suffix["2"], "count": by_suffix["3"]}
        boxes = [d for d in details if len(d["shape"]) == 3 and d["shape"][-1] == 4]
        count = [d for d in details if len(d["shape"]) == 1]
        if len(details) == 4 and len(boxes) == 1 and len(count) == 1:
            rest = [d for d in details if d is not boxes[0] and d is not count[0]]
            return {"boxes": boxes[0]["index"], "classes": rest[0]["index"],
                    "scores": rest[1]["index"], "count": count[0]["index"]}
        idx = [d["index"] for d in details]
        return {"boxes": idx[0], "classes": idx[1], "scores": idx[2], "count": idx[3]}

    def _input(self, bgr: np.ndarray) -> np.ndarray:
        rgb = cv2.cvtColor(cv2.resize(bgr, (self.in_w, self.in_h)), cv2.COLOR_BGR2RGB)
        if np.dtype(self.in_dtype) == np.uint8:
            return rgb[np.newaxis].astype(np.uint8)
        return (rgb[np.newaxis].astype(np.float32) - 127.5) / 127.5  # float models: [-1, 1]

    def detect(self, bgr: np.ndarray) -> list[Detection]:
        self.it.set_tensor(self.in_index, self._input(bgr))
        self.it.invoke()
        get = lambda k: np.asarray(self.it.get_tensor(self.out[k]))
        boxes, classes, scores = get("boxes")[0], get("classes")[0], get("scores")[0]
        n = int(np.asarray(get("count")).reshape(-1)[0]) if get("count").size else len(scores)
        out = []
        for i in range(min(n, len(scores))):
            if int(round(float(classes[i]))) != self.person_class or scores[i] < self.min_score:
                continue
            y0, x0, y1, x1 = (float(np.clip(v, 0.0, 1.0)) for v in boxes[i])  # SSD: ymin first
            if x1 > x0 and y1 > y0:
                out.append(Detection(x0, y0, x1, y1, float(scores[i])))
        return out
