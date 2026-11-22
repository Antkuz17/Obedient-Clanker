"""On a new network (e.g. the phone hotspot "QNX"): find the drone and let this laptop fly it.

  python -m tools.field_setup              # find the Pi, allow this laptop, restart the stack
  python -m tools.field_setup --demo       # ... in no-fly demo mode
  python -m tools.field_setup --find-only  # just print the Pi's address

Steps: this laptop's IP on the LAN that has a default route -> scan that /24 for an SSH server
whose MAC is a Raspberry Pi (or that answers the QNX login) -> `deploy_pi --stations <this IP>`
(edge accepts commands only from it; QNX stats come here) -> restart the stack -> wait for the
edge on :7000. Prints the `set QNX_HOST=...` / `EDGE_HOST` to use with the other tools.
"""
from __future__ import annotations

import argparse
import concurrent.futures as cf
import os
import socket
import subprocess
import sys
import time

PI_MAC_PREFIXES = ("2c-cf-67", "d8-3a-dd", "dc-a6-32", "e4-5f-01", "b8-27-eb", "28-cd-c1")


def my_ip() -> str:
    """The address this laptop uses to reach the internet (the hotspot's LAN)."""
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(("8.8.8.8", 80))  # no packet is sent for UDP connect
        return s.getsockname()[0]
    finally:
        s.close()


def ssh_hosts(prefix: str) -> list[str]:
    def probe(i: int) -> str | None:
        ip = f"{prefix}.{i}"
        s = socket.socket()
        s.settimeout(1.0)
        try:
            s.connect((ip, 22))
            return ip
        except OSError:
            return None
        finally:
            s.close()
    with cf.ThreadPoolExecutor(128) as ex:
        return [ip for ip in ex.map(probe, range(1, 255)) if ip]


def is_pi(ip: str) -> bool:
    try:
        arp = subprocess.run(["arp", "-a", ip], capture_output=True, text=True, timeout=5).stdout
    except (OSError, subprocess.TimeoutExpired):
        arp = ""
    if any(p in arp.lower().replace(":", "-") for p in PI_MAC_PREFIXES):
        return True
    try:  # no ARP match (phone hotspots can hide it): try the QNX login (tools/deploy_pi.ssh)
        from tools.deploy_pi import ssh
        c = ssh(ip, timeout=5)
        _, o, _ = c.exec_command("uname -s", timeout=10)
        ok = "QNX" in o.read().decode()
        c.close()
        return ok
    except Exception:
        return False


def main() -> int:
    ap = argparse.ArgumentParser(prog="tools.field_setup")
    ap.add_argument("--demo", action="store_true", help="restart in no-fly demo mode")
    ap.add_argument("--gps", action="store_true", help="edge in GPS mode (outdoors)")
    ap.add_argument("--find-only", action="store_true")
    ap.add_argument("--also", default="", help="more laptop IPs allowed to command (comma list)")
    a = ap.parse_args()
    me = my_ip()
    prefix = me.rsplit(".", 1)[0]
    print(f"this laptop: {me}; scanning {prefix}.1-254 for the drone ...", flush=True)
    pis = [ip for ip in ssh_hosts(prefix) if ip != me and is_pi(ip)]
    if not pis:
        print("drone not found. Is the Pi on (it takes ~40 s to join Wi-Fi), and are the laptop "
              "and the Pi on the same hotspot ('QNX' / 'everywhere')?")
        return 1
    pi = pis[0]
    print(f"drone (QNX Pi) at {pi}")
    if a.find_only:
        print(f"\n  set QNX_HOST={pi}   (PowerShell: $env:QNX_HOST='{pi}'; $env:EDGE_HOST='{pi}')")
        return 0
    stations = ",".join([me] + [x for x in a.also.split(",") if x.strip()])
    env = {**os.environ, "QNX_HOST": pi}
    # sync this laptop's code, allow this laptop, then restart the stack (all in tools/deploy_pi)
    cmd = [sys.executable, "-m", "tools.deploy_pi", "--host", pi, "--stations", stations,
           "--restart"]
    if a.demo:
        cmd.append("--demo")
    if a.gps:
        cmd.append("--gps")
    subprocess.run(cmd, env=env, check=True)
    print("waiting for the edge on :7000 ...", flush=True)
    end = time.time() + 90
    while time.time() < end:
        try:
            socket.create_connection((pi, 7000), timeout=2).close()
            print(f"\nready. Drone at {pi}, this laptop {me} may command it"
                  + (" (DEMO MODE)" if a.demo else "") + ".\n"
                  f"  PowerShell: $env:QNX_HOST='{pi}'; $env:EDGE_HOST='{pi}'\n"
                  f"  then: python -m tools.system_check / python -m tools.auto_hover --host {pi}")
            return 0
        except OSError:
            time.sleep(2)
    print("edge didn't come up within 90 s: python -m tools.deploy_pi --host "
          f"{pi} --run 'tail logs/supervisor.out'")
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
