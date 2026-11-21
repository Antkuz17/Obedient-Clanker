"""Download the person-detector model into edge/vision/models/ (git-ignored).

    python -m tools.fetch_person_model        # on the laptop; then python -m tools.deploy_pi

COCO SSD-MobileNet v1 quantised (Apache 2.0, from TensorFlow's model zoo): 300x300 uint8 input,
~4 MB, class 0 = person. The zip is checked against a pinned SHA-256 before anything is
extracted, and only the two expected files are written.
"""
from __future__ import annotations

import hashlib
import io
import sys
import urllib.request
import zipfile
from pathlib import Path

URL = ("https://storage.googleapis.com/download.tensorflow.org/models/tflite/"
       "coco_ssd_mobilenet_v1_1.0_quant_2018_06_29.zip")
SHA256 = "a809cd290b4d6a2e8a9d5dad076e0bd695b8091974e0eed1052b480b2f21b6dc"
FILES = ("detect.tflite", "labelmap.txt")
DEST = Path(__file__).resolve().parent.parent / "edge" / "vision" / "models"


def main() -> int:
    if all((DEST / f).exists() for f in FILES):
        print(f"already there: {DEST}")
        return 0
    print(f"downloading {URL}")
    with urllib.request.urlopen(URL, timeout=60) as r:
        data = r.read()
    got = hashlib.sha256(data).hexdigest()
    if got != SHA256:
        print(f"checksum mismatch: got {got}, want {SHA256}; not using it", file=sys.stderr)
        return 1
    DEST.mkdir(parents=True, exist_ok=True)
    with zipfile.ZipFile(io.BytesIO(data)) as z:
        for f in FILES:
            (DEST / f).write_bytes(z.read(f))
    labels = (DEST / "labelmap.txt").read_text().split()
    print(f"saved {', '.join(FILES)} to {DEST} (first labels: {labels[:3]})")
    return 0


if __name__ == "__main__":
    sys.exit(main())
