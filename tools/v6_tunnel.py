"""Reach the drone when the phone hotspot gives this laptop no IPv4 (field 2026-11-22: the
iPhone hotspot handed the Mac only IPv6; the Pi still answered SSH on its IPv6 link-local).

  python -m tools.v6_tunnel fe80::2ecf:67ff:fed5:cc0e%en0 7400:7000 8080:8080
  python -m tools.deploy_pi --host 'fe80::2ecf:67ff:fed5:cc0e%en0' --stations 127.0.0.1 --gps --restart
  python -m tools.demo_laptop --edge 127.0.0.1:7400 --rng          # dashboard http://127.0.0.1:8100

Forwards local 127.0.0.1:<l> to the Pi's 127.0.0.1:<r> over the QNX login (tools/deploy_pi.ssh).
Through the tunnel the edge sees the station as 127.0.0.1, hence --stations 127.0.0.1. Local
7000 is taken by AirPlay on macOS, so the edge goes on 7400. To find the Pi's address, ping the
all-nodes address (ping6 -c 3 ff02::1%en0) and look in `ndp -an` for a MAC starting 2c:cf:67.
Reconnects if SSH drops. UDP (QNX stats) doesn't go through, so the dashboard's QNX timing card
stays empty.
"""
from __future__ import annotations

import select
import socket
import sys
import threading
import time

from tools.deploy_pi import ssh

state: dict = {"t": None}


def transport(host: str):
    t = state["t"]
    if t is None or not t.is_active():
        c = ssh(host, timeout=10)
        t = c.get_transport()
        t.set_keepalive(2)
        state["t"], state["c"] = t, c
        print("ssh connected", flush=True)
    return t


def pump(a, b) -> None:
    try:
        while True:
            r, _, _ = select.select([a, b], [], [], 30)
            for s in r:
                d = s.recv(65536)
                if not d:
                    return
                (b if s is a else a).sendall(d)
    except Exception:
        pass
    finally:
        for s in (a, b):
            try:
                s.close()
            except Exception:
                pass


def serve(host: str, lport: int, rport: int) -> None:
    srv = socket.socket()
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(("127.0.0.1", lport))
    srv.listen(16)
    print(f"127.0.0.1:{lport} -> pi:{rport}", flush=True)
    while True:
        cl, _ = srv.accept()
        try:  # one bad connection (or the Pi rebooting) must not end this listener
            cl.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            ch = transport(host).open_channel("direct-tcpip", ("127.0.0.1", rport),
                                              cl.getpeername())
        except Exception as e:
            print(f"channel to {rport} failed: {e}", flush=True)
            state["t"] = None
            try:
                cl.close()
            except Exception:
                pass
            continue
        threading.Thread(target=pump, args=(cl, ch), daemon=True).start()


def main() -> None:
    if len(sys.argv) < 3:
        raise SystemExit(__doc__)
    host = sys.argv[1]
    pairs = [tuple(int(x) for x in p.split(":")) for p in sys.argv[2:]]
    while True:
        try:
            transport(host)
            break
        except Exception as e:
            print("ssh not up yet:", e, flush=True)
            time.sleep(3)
    for lport, rport in pairs:
        threading.Thread(target=serve, args=(host, lport, rport), daemon=True).start()
    while True:
        time.sleep(1)


if __name__ == "__main__":
    main()
