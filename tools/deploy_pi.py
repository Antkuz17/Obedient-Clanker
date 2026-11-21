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


def main() -> None:
    import paramiko
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default=os.environ.get("QNX_HOST", "192.168.68.131"))
    ap.add_argument("--build", action="store_true", help="rebuild camgrab and hbwd on the Pi")
    ap.add_argument("--run", help="then run this command in ~/hope on the Pi and print its output")
    a = ap.parse_args()
    c = paramiko.SSHClient()
    c.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    c.connect(a.host, username=os.environ.get("QNX_USER", "qnxuser"),
              password=os.environ.get("QNX_PASS", "qnxuser"), look_for_keys=False,
              allow_agent=False, timeout=10)
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

    def run(cmd: str) -> None:
        _, out, err = c.exec_command(f"cd {REMOTE} && {cmd}", timeout=300)
        print(out.read().decode(errors="replace") + err.read().decode(errors="replace"), end="")

    if a.build:
        run("cc -O2 -o edge/camgrab/camgrab edge/camgrab/camgrab.c -lcamapi && echo camgrab ok;"
            " [ -f edge/hbwd/hbwd.c ] && cc -O2 -o edge/hbwd/hbwd edge/hbwd/hbwd.c && echo hbwd ok")
    if a.run:
        run(f"env {PYPATH} {a.run}")


if __name__ == "__main__":
    main()
