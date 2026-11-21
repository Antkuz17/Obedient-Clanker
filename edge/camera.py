"""Shared-memory camera frames: the Python side of edge/camgrab/camgrab.c (layout v1 there).

On the Pi:   camgrab (C, libcamapi) writes;  ShmFrameReader reads.
On a laptop: ShmFrameWriter (tools/fake_camgrab.py, from a webcam or the sim) writes the same
             layout, so flow/vision/video code runs unchanged before it ever touches QNX.

    r = ShmFrameReader("camgrab_fwd")
    f = r.wait_new(timeout=1.0)          # Frame or None
    gray = f.gray()                      # HxW uint8 (cheap: no colour conversion)
    bgr = f.bgr()                        # HxWx3 uint8 (needs cv2)
"""
from __future__ import annotations

import mmap
import os
import struct
import time
from dataclasses import dataclass

import numpy as np

MAGIC = 0x42524743
HDR = 64
FMT_YUYV, FMT_NV12 = 1, 2
_HDR = struct.Struct("<IIIIIIQQI")  # magic version w h fmt frame_bytes seq t_us pid
_SEQ_OFF, _T_OFF = 24, 32


@dataclass
class Frame:
    seq: int          # frame number (increments by 1 per frame)
    t: float          # capture time, seconds since the epoch (matches time.time())
    width: int
    height: int
    fmt: int
    data: bytes

    def gray(self) -> np.ndarray:
        a = np.frombuffer(self.data, np.uint8)
        if self.fmt == FMT_NV12:
            return a[: self.width * self.height].reshape(self.height, self.width)
        return a.reshape(self.height, self.width * 2)[:, 0::2]

    def bgr(self) -> np.ndarray:
        import cv2
        a = np.frombuffer(self.data, np.uint8)
        if self.fmt == FMT_NV12:
            return cv2.cvtColor(a.reshape(self.height * 3 // 2, self.width),
                                cv2.COLOR_YUV2BGR_NV12)
        return cv2.cvtColor(a.reshape(self.height, self.width, 2), cv2.COLOR_YUV2BGR_YUY2)


def _shm_path(name: str) -> str | None:
    for d in ("/dev/shmem", "/dev/shm"):  # QNX, Linux
        if os.path.isdir(d):
            return os.path.join(d, name)
    return None  # macOS: no filesystem view; use _posixshmem


class _Region:
    """A named POSIX shared-memory region, mapped read/write."""

    def __init__(self, name: str, size: int | None = None):
        self._shm = None
        path = _shm_path(name)
        if path is not None:
            flags = os.O_RDWR | (os.O_CREAT if size else 0)
            fd = os.open(path, flags, 0o666)
            try:
                if size:
                    os.ftruncate(fd, size)
                # QNX rejects mmap length 0 ("whole file") with EINVAL; pass the real size.
                self.buf = mmap.mmap(fd, size or os.fstat(fd).st_size)
            finally:
                os.close(fd)
            self.path = path
        else:
            from multiprocessing import shared_memory
            kw = {"track": False} if "track" in shared_memory.SharedMemory.__init__.__code__.co_varnames else {}
            self._shm = shared_memory.SharedMemory(name=name, create=bool(size), size=size or 0,
                                                   **kw)
            self.buf = self._shm.buf
            self.path = name

    def close(self) -> None:
        if self._shm is not None:
            self.buf = None
            self._shm.close()
        else:
            self.buf.close()

    def unlink(self) -> None:
        if self._shm is not None:
            self._shm.unlink()
        elif os.path.exists(self.path):
            os.unlink(self.path)


class ShmFrameReader:
    """Reads camgrab's latest frame. If no new frame arrives for `reopen_after_s`, it re-opens
    the shared memory by name, so a restarted grabber (new region) is picked up instead of
    the reader staring at the old, orphaned mapping forever (review2 #8)."""

    def __init__(self, name: str = "camgrab_fwd", open_timeout: float = 5.0,
                 reopen_after_s: float = 0.5):
        self.name = name
        self.reopen_after_s = reopen_after_s
        self._open(open_timeout)
        self._last_new = time.monotonic()
        self._last_seq = -1
        self._last_reopen = 0.0
        self.reopens = 0

    def _open(self, open_timeout: float) -> None:
        name = self.name
        deadline = time.monotonic() + open_timeout
        while True:
            try:
                self._r = _Region(name)
                magic, ver, self.width, self.height, self.fmt, self.frame_bytes, *_ = \
                    _HDR.unpack_from(self._r.buf, 0)
                if magic == MAGIC and ver == 1:
                    break
                self._r.close()
            except (FileNotFoundError, ValueError, OSError):
                pass
            if time.monotonic() > deadline:
                raise TimeoutError(f"no camgrab frames in shared memory '{name}'; "
                                   "is camgrab (or tools/fake_camgrab.py) running?")
            time.sleep(0.1)

    def _maybe_reopen(self) -> None:
        now = time.monotonic()
        if now - self._last_new < self.reopen_after_s or now - self._last_reopen < 0.5:
            return
        self._last_reopen = now
        old = self._r
        try:
            self._open(open_timeout=0)
        except TimeoutError:
            return  # grabber not back yet; keep the old mapping and try again later
        try:
            old.close()
        except Exception:
            pass
        self.reopens += 1
        self._last_seq = -1  # a restarted grabber counts from 0 again

    def read(self, max_age: float | None = None) -> Frame | None:
        """Latest complete frame, or None if no frame has been written yet.
        max_age (opt-in): also None if the latest frame is older than this many seconds,
        so a consumer like the person detector never acts on a frozen picture."""
        f = self._read_once()
        if f is not None and f.seq != self._last_seq:
            self._last_seq, self._last_new = f.seq, time.monotonic()
        else:
            self._maybe_reopen()
            f = self._read_once() if self.reopens else f
        if f is not None and max_age is not None and time.time() - f.t > max_age:
            return None
        return f

    def _read_once(self) -> Frame | None:
        buf = self._r.buf
        for _ in range(50):
            s1 = struct.unpack_from("<Q", buf, _SEQ_OFF)[0]
            if s1 == 0:
                return None
            if s1 & 1:
                time.sleep(0.0005)
                continue
            data = bytes(buf[HDR:HDR + self.frame_bytes])
            t_us = struct.unpack_from("<Q", buf, _T_OFF)[0]
            if struct.unpack_from("<Q", buf, _SEQ_OFF)[0] == s1:
                return Frame(s1 // 2, t_us / 1e6, self.width, self.height, self.fmt, data)
        return None

    def wait_new(self, after_seq: int = -1, timeout: float = 1.0) -> Frame | None:
        """Block until a frame newer than after_seq exists (or timeout)."""
        deadline = time.monotonic() + timeout
        reopens = self.reopens
        while time.monotonic() < deadline:
            f = self.read()
            if self.reopens != reopens:  # new grabber: its seq restarted, so accept any frame
                after_seq, reopens = -1, self.reopens
            if f is not None and f.seq > after_seq:
                return f
            time.sleep(0.003)
        return None

    def close(self) -> None:
        self._r.close()


class ShmFrameWriter:
    """Same layout as camgrab.c, for laptops and tests."""

    def __init__(self, name: str, width: int, height: int, fmt: int = FMT_NV12):
        self.width, self.height, self.fmt = width, height, fmt
        self.frame_bytes = width * height * 3 // 2 if fmt == FMT_NV12 else width * height * 2
        try:
            _Region(name).unlink()
        except (FileNotFoundError, OSError, ValueError):
            pass
        self._r = _Region(name, HDR + self.frame_bytes)
        buf = self._r.buf
        buf[:HDR] = bytes(HDR)
        _HDR.pack_into(buf, 0, 0, 1, width, height, fmt, self.frame_bytes, 0, 0, os.getpid())
        struct.pack_into("<I", buf, 0, MAGIC)  # last: readers wait for the magic
        self._seq = 0

    def write_raw(self, data: bytes, t: float | None = None) -> None:
        buf = self._r.buf
        self._seq += 1
        struct.pack_into("<Q", buf, _SEQ_OFF, self._seq)          # odd: writing
        buf[HDR:HDR + self.frame_bytes] = data
        struct.pack_into("<Q", buf, _T_OFF, int((t or time.time()) * 1e6))
        self._seq += 1
        struct.pack_into("<Q", buf, _SEQ_OFF, self._seq)          # even: stable

    def write_bgr(self, bgr: np.ndarray, t: float | None = None) -> None:
        import cv2
        if bgr.shape[1] != self.width or bgr.shape[0] != self.height:
            bgr = cv2.resize(bgr, (self.width, self.height))
        if self.fmt == FMT_NV12:
            i420 = cv2.cvtColor(bgr, cv2.COLOR_BGR2YUV_I420)
            y = i420[: self.height]
            u = i420[self.height: self.height + self.height // 4].reshape(-1)
            v = i420[self.height + self.height // 4:].reshape(-1)
            uv = np.empty(u.size * 2, np.uint8)
            uv[0::2], uv[1::2] = u, v
            self.write_raw(y.tobytes() + uv.tobytes(), t)
        else:
            yuv = cv2.cvtColor(bgr, cv2.COLOR_BGR2YUV)
            out = np.empty((self.height, self.width, 2), np.uint8)
            out[:, :, 0] = yuv[:, :, 0]
            out[:, 0::2, 1] = yuv[:, 0::2, 1]
            out[:, 1::2, 1] = yuv[:, 1::2, 2]
            self.write_raw(out.tobytes(), t)

    def close(self, unlink: bool = True) -> None:
        if unlink:
            self._r.unlink()
        self._r.close()
