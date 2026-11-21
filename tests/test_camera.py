import os
import shutil
import subprocess
import uuid
from pathlib import Path

import numpy as np
import pytest

from edge.camera import FMT_NV12, FMT_YUYV, ShmFrameReader, ShmFrameWriter

ROOT = Path(__file__).resolve().parent.parent


def shm_name() -> str:
    return "cg" + uuid.uuid4().hex[:10]  # macOS caps shm names at 31 chars


@pytest.mark.parametrize("fmt", [FMT_NV12, FMT_YUYV])
def test_python_writer_reader_roundtrip(fmt):
    name = shm_name()
    w = ShmFrameWriter(name, 64, 48, fmt)
    r = ShmFrameReader(name, open_timeout=1)
    assert r.read() is None
    img = np.zeros((48, 64, 3), np.uint8)
    img[:, :32] = (0, 0, 255)       # red left half
    img[:, 32:] = (255, 0, 0)       # blue right half
    w.write_bgr(img, t=1234.5)
    f = r.wait_new(timeout=1)
    assert f.seq == 1 and f.t == pytest.approx(1234.5)
    bgr = f.bgr()
    assert bgr[24, 10, 2] > 200 and bgr[24, 50, 0] > 200
    assert f.gray().shape == (48, 64)
    w.write_bgr(img)
    assert r.wait_new(after_seq=1, timeout=1).seq == 2
    r.close()
    w.close()


@pytest.mark.skipif(shutil.which("cc") is None, reason="needs a C compiler")
@pytest.mark.parametrize("fmt", ["nv12", "yuyv"])
def test_camgrab_c_against_stub_camera(tmp_path, fmt):
    """Compile the real camgrab.c against a fake libcamapi that pads every row (stride > width)
    and check Python sees the exact pixels, with padding stripped and frames in order."""
    exe = tmp_path / "camgrab"
    subprocess.run(["cc", "-O2", f"-I{ROOT}/tests/camstub", "-o", str(exe),
                    str(ROOT / "edge/camgrab/camgrab.c"), str(ROOT / "tests/camstub/stub.c"),
                    "-lpthread"], check=True)
    name = shm_name()
    p = subprocess.Popen([str(exe), "-u", "5", "-n", name], stdout=subprocess.PIPE,
                         env={**os.environ, "CAMSTUB_FMT": fmt})
    try:
        r = ShmFrameReader(name, open_timeout=5)
        assert (r.width, r.height) == (640, 480)
        assert r.fmt == (FMT_NV12 if fmt == "nv12" else FMT_YUYV)
        f1 = r.wait_new(timeout=2)
        f2 = r.wait_new(after_seq=f1.seq, timeout=2)
        for f in (f1, f2):
            g = f.gray()
            k = int(g[0, 0])  # stub writes Y = (x + y + k) & 0xff
            yy, xx = np.mgrid[0:480, 0:640]
            assert np.array_equal(g, ((xx + yy + k) & 0xFF).astype(np.uint8))
        assert f2.seq > f1.seq and f2.t >= f1.t
        assert f1.bgr().shape == (480, 640, 3)
        r.close()
    finally:
        p.terminate()
        p.wait(5)
