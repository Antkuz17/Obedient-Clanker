import time

from pymavlink.dialects.v20 import ardupilotmega as mav

from edge.mavlink_usb import StreamMav


def test_stream_mav_drains_a_chunk_without_waiting_per_message():
    """Regression: a 512-byte USB chunk holds several messages; each must parse without
    another blocking read, or telemetry lags further and further behind (found in SITL)."""
    enc = mav.MAVLink(None, srcSystem=1, srcComponent=1)
    chunk = b"".join(enc.attitude_encode(i, 0, 0, i / 100, 0, 0, 0).pack(enc)
                     for i in range(20))
    reads = [chunk]

    def read(timeout):
        if reads:
            return reads.pop()
        time.sleep(timeout)
        return b""

    m = StreamMav(read, lambda b: None, "test")
    t0 = time.monotonic()
    got = [m.recv_match(type="ATTITUDE", blocking=True, timeout=1) for _ in range(20)]
    assert all(got) and [g.time_boot_ms for g in got] == list(range(20))
    assert time.monotonic() - t0 < 0.2


def test_stream_mav_writes_go_through():
    out = []
    m = StreamMav(lambda t: b"", out.append, "test")
    m.mav.heartbeat_send(6, 8, 0, 0, 0)
    assert out and out[0][0] in (0xFD, 0xFE)
