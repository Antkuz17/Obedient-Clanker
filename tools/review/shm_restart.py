"""Review 2 repro (docs/review2.md, bug 8): a frame reader never sees frames again after the
grabber is restarted, and read() keeps returning the old frame.

  .venv/bin/python -m tools.review.shm_restart
"""
import time

import numpy as np

from edge.camera import ShmFrameReader, ShmFrameWriter

w = ShmFrameWriter("review_cam", 64, 48)
img = np.zeros((48, 64, 3), np.uint8)
w.write_bgr(img)
r = ShmFrameReader("review_cam")
f = r.wait_new()
print("first frame seq", f.seq)
w.close(unlink=True)                       # grabber killed
w2 = ShmFrameWriter("review_cam", 64, 48)  # supervisor restarts it
for _ in range(5):
    w2.write_bgr(img)
    time.sleep(0.01)
print("reader sees a new frame after the restart:",
      r.wait_new(after_seq=f.seq, timeout=1.0) is not None)
old = r.read()
print("read() still returns the old frame, age s:", round(time.time() - old.t, 2), "seq", old.seq)
print("a fresh reader works:", ShmFrameReader("review_cam").wait_new() is not None)
w2.close()
