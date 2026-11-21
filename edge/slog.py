"""One line to QNX's system log (slog2) through edge/slog2d (UDP 127.0.0.1:7020).

    from edge.slog import slog
    slog("edge safety_reject: move of 3.0 m exceeds the 2.0 m limit", "W")

Fire and forget: with no slog2d (laptop, sim, tests) the datagram just goes nowhere. Watch it
on the Pi with `slog2info -w -b hope`. Stdlib only.
"""
from __future__ import annotations

import socket

ADDR = ("127.0.0.1", 7020)
_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)


def slog(text: str, level: str = "I") -> None:
    """level: E error, W warning, N notice, I info."""
    try:
        _sock.sendto(f"{level[:1]} {text}"[:1000].encode(errors="replace"), ADDR)
    except OSError:
        pass
