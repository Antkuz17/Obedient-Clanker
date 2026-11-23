"""Copy edge/, shared/ and tools/ to the QNX Pi (and optionally rebuild the C programs).

  python -m tools.deploy_pi                      # sync code to ~/hope on the Pi
  python -m tools.deploy_pi --build              # also rebuild camgrab and hbwd on the Pi
  python -m tools.deploy_pi --host 192.168.68.131 --run "python3 -m tools.preflight"

Uses the QNX quick-start login (qnxuser/qnxuser) unless QNX_USER / QNX_PASS are set. Needs
paramiko on the laptop (pip install paramiko). Only files that changed (size or mtime) are sent.
"""
from __future__ import annotations

import argparse
import os
import stat
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
SYNC = ("edge", "shared", "tools")
SKIP_DIRS = {"__pycache__", "sitl", "review", ".pytest_cache"}
REMOTE = "/data/home/qnxuser/hope"
PYPATH = f"PYTHONPATH={REMOTE}:/data/home/qnxuser/.local/lib/python3.14/site-packages"


def ssh(host: str, timeout: float = 10.0):
    """An SSH session on the Pi with the QNX login (QNX_USER / QNX_PASS, see the docstring)."""
    import paramiko
    c = paramiko.SSHClient()
    c.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    c.connect(host, username=os.environ.get("QNX_USER", "qnxuser"),
              password=os.environ.get("QNX_PASS", "qnxuser"), look_for_keys=False,
              allow_agent=False, timeout=timeout)
    return c


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default=os.environ.get("QNX_HOST", "192.168.68.131"))
    ap.add_argument("--build", action="store_true", help="rebuild camgrab, hbwd and slog2d on the Pi")
    ap.add_argument("--run", help="then run this command in ~/hope on the Pi and print its output")
    ap.add_argument("--autostart", choices=("on", "off"),
                    help="on: install tools/onboot.sh as /boot/onboot (stack starts at power-on); "
                         "off: remove it")
    ap.add_argument("--stations", help="comma-separated laptop IPs allowed to command the drone; "
                    "writes ~/hope/supervisor_local.json from tools/supervisor_pi.json (the first "
                    "IP also gets the QNX stats). Restart the stack to apply")
    ap.add_argument("--gps", action="store_true",
                    help="with --stations: edge in GPS mode (outdoors, after fc_profile --gps)")
    ap.add_argument("--demo", action="store_true",
                    help="with --stations: run the edge in no-fly demo mode (edge/demo.py); "
                         "rerun --stations without it to fly again")
    ap.add_argument("--restart", action="store_true",
                    help="restart the whole drone stack (after --stations / --build)")
    ap.add_argument("--force", action="store_true",
                    help="with --restart: restart even if the flight controller says it is armed")
    ap.add_argument("--reboot-fc", action="store_true",
                    help="reboot the flight controller (clears a latched battery failsafe; only "
                         "while disarmed). The relay reconnects by itself")
    a = ap.parse_args()
    c = ssh(a.host)
    sf = c.open_sftp()

    def mkdirs(path: str) -> None:
        try:
            sf.stat(path)
        except IOError:
            mkdirs(path.rsplit("/", 1)[0])
            sf.mkdir(path)

    sent = 0
    for top in SYNC:
        for root, dirs, files in os.walk(ROOT / top):
            dirs[:] = [d for d in dirs if d not in SKIP_DIRS]
            rel = Path(root).relative_to(ROOT).as_posix()
            mkdirs(f"{REMOTE}/{rel}")
            for f in files:
                if f.endswith(".pyc"):
                    continue
                local = Path(root) / f
                remote = f"{REMOTE}/{rel}/{f}"
                st = local.stat()
                try:
                    rs = sf.stat(remote)
                    if rs.st_size == st.st_size and int(rs.st_mtime) >= int(st.st_mtime):
                        continue
                except IOError:
                    pass
                sf.put(str(local), remote)
                sf.utime(remote, (st.st_atime, st.st_mtime))
                if os.access(local, os.X_OK) or f.endswith(".sh"):
                    sf.chmod(remote, stat.S_IRWXU | stat.S_IRGRP | stat.S_IXGRP)
                sent += 1
    print(f"synced {sent} changed file(s) to {a.host}:{REMOTE}")
    # the boot auto-start runs start_sup.sh: keep it the same as tools/start_pi.sh
    c.exec_command(f"cd {REMOTE} && cp tools/start_pi.sh start_sup.sh && chmod +x start_sup.sh",
                   timeout=30)[1].read()

    def run(cmd: str) -> str:
        _, out, err = c.exec_command(f"cd {REMOTE} && {cmd}", timeout=300)
        text = out.read().decode(errors="replace") + err.read().decode(errors="replace")
        print(text, end="")
        return text

    if a.stations:
        import json
        ips = [ip.strip() for ip in a.stations.split(",") if ip.strip()]
        cfg = json.loads((ROOT / "tools" / "supervisor_pi.json").read_text())
        for child in cfg:
            cmd, out = child["cmd"], []
            i = 0
            while i < len(cmd):
                if cmd[i] == "--station-ip":
                    for ip in ips:
                        out += ["--station-ip", ip]
                    i += 2
                    continue
                out.append(cmd[i].replace("LAPTOP_IP", ips[0]))
                i += 1
            if a.demo and child["name"] == "edge":
                out.append("--demo")
            if a.gps and child["name"] == "edge":
                out.append("--gps")  # FC on GPS (fc_profile --gps): no fake origin / flow check
            child["cmd"] = out
        with sf.open(f"{REMOTE}/supervisor_local.json", "w") as f:
            f.write(json.dumps(cfg, indent=2))
        with sf.open(f"{REMOTE}/qnx_stats_to", "w") as f:  # start_pi.sh: supervisor events
            f.write(f"{ips[0]}:7010\n")
        print(f"wrote supervisor_local.json: stations {ips}"
              + (", DEMO MODE (never flies)" if a.demo else "") + (", GPS" if a.gps else "")
              + " (restart the stack to apply)")

    if a.autostart:
        pw = os.environ.get("QNX_PASS", "qnxuser")
        if a.autostart == "on":
            run("cp tools/start_pi.sh start_sup.sh && chmod +x start_sup.sh && "
                f"echo {pw} | sudo -S sh -c 'cp tools/onboot.sh /boot/onboot && sed -i "
                "\"s/\r$//\" /boot/onboot' 2>&1 | grep -v Password; ls -la /boot/onboot")
        else:
            run(f"echo {pw} | sudo -S rm -f /boot/onboot 2>&1 | grep -v Password; echo removed")

    if a.build:
        run("cc -O2 -o edge/camgrab/camgrab edge/camgrab/camgrab.c -lcamapi && echo camgrab ok;"
            " [ -f edge/hbwd/hbwd.c ] && cc -O2 -o edge/hbwd/hbwd edge/hbwd/hbwd.c -lsocket && echo hbwd ok;"
            " cc -O2 -o edge/slog2d/slog2d edge/slog2d/slog2d.c /usr/lib/libslog2.so.1 -lsocket && echo slog2d ok")
    if a.reboot_fc:  # MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN; ArduPilot refuses it while armed
        run(f"env {PYPATH} python3 -c \"from tools import fclink; m = fclink.connect(fclink.TAP); "
            "m.mav.command_long_send(1, 1, 246, 0, 1, 0, 0, 0, 0, 0, 0); print('FC reboot sent')\"")
    if a.restart:
        # Restarting kills the safety core, the C watchdog and the USB relay together. Under a
        # flying drone that leaves only the FC's 2 s failsafe, with no rangefinder: never do it
        # while the FC says armed (another laptop running field_setup mid-flight did exactly this).
        state = run(f"env {PYPATH} python3 -c \"from tools import fclink; "
                    "print('FC_ARMED=%s' % fclink.is_armed())\"")
        if "FC_ARMED=True" in state and not a.force:
            raise SystemExit("the flight controller is ARMED: not restarting the drone stack. "
                             "Land and disarm first (or --force if you are sure it is on the "
                             "ground).")
        if "FC_ARMED=False" not in state:
            print("could not ask the flight controller whether it is armed (relay down?); "
                  "restarting anyway")
        pw = os.environ.get("QNX_PASS", "qnxuser")
        run(f"echo {pw} | sudo -S sh -c 'slay -f camgrab; slay -f hbwd; slay -f slog2d; "
            f"slay -f python3; sleep 1; nohup {REMOTE}/start_sup.sh >/dev/null 2>&1 </dev/null &' "
            "2>&1 | grep -v Password; echo stack restarting")
    if a.run:
        run(f"env {PYPATH} {a.run}")


if __name__ == "__main__":
    main()
