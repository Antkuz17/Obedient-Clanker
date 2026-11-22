"""End-to-end system check from the laptop: is every link in the chain wired up?

  python -m tools.system_check                 # PASS / FAIL per link, read-only (never arms)

Links checked: Pi reachable (SSH) -> supervisor children alive -> FC link (preflight) ->
camera frame rates -> on-board person detector -> video stream -> laptop -> edge protocol
(telemetry + a gate rejection, drone stays landed) -> Vultr models (if a key is set) ->
cloud server /health (if CLOUD_URL is set). Uses QNX_HOST (default 192.168.68.131).
"""
from __future__ import annotations

import asyncio
import json
import os
import time
import urllib.request
from pathlib import Path

HOST = os.environ.get("QNX_HOST", os.environ.get("EDGE_HOST", "192.168.68.131"))
REMOTE = "/data/home/qnxuser/hope"
PYPATH = f"PYTHONPATH={REMOTE}:/data/home/qnxuser/.local/lib/python3.14/site-packages"
CHILDREN = ("usb_relay", "rangefinder", "hbwd", "edge", "camgrab_fwd", "camgrab_down", "flow",
            "vision", "video", "blackbox", "camrec")
results: list[tuple[str, bool, str]] = []


def check(name: str, ok: bool, info: str = "") -> None:
    results.append((name, bool(ok), info))
    print(f"{'PASS' if ok else 'FAIL'}  {name:<28} {info}", flush=True)


def env_file() -> dict:
    out = {}
    p = Path(__file__).resolve().parent.parent / ".env"
    if p.exists():
        for line in p.read_text().splitlines():
            if "=" in line and not line.lstrip().startswith("#"):
                k, v = line.split("=", 1)
                out[k.strip()] = v.strip()
    return out


def log_age_s(line: str, now: str) -> float | None:
    """Seconds between a Python log line's timestamp and `now` (both the Pi's local clock)."""
    import datetime as dt
    try:
        f = "%Y-%m-%d %H:%M:%S"
        return (dt.datetime.strptime(now, f) - dt.datetime.strptime(line[:19], f)).total_seconds()
    except ValueError:
        return None


def ssh_checks() -> bool:
    try:
        import paramiko
        c = paramiko.SSHClient()
        c.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        c.connect(HOST, username="qnxuser", password=os.environ.get("QNX_PASS", "qnxuser"),
                  look_for_keys=False, allow_agent=False, timeout=6)
    except Exception as e:
        check("Pi reachable (SSH)", False, f"{HOST}: {e}")
        return False
    check("Pi reachable (SSH)", True, HOST)

    def run(cmd: str, t: int = 90) -> str:
        _, o, e = c.exec_command(f"cd {REMOTE} && {cmd}", timeout=t)
        return o.read().decode(errors="replace") + e.read().decode(errors="replace")

    # as root: qnxuser can't see the arguments of root's processes (usb_relay, rangefinder)
    pw = os.environ.get("QNX_PASS", "qnxuser")
    procs = run(f"echo {pw} | sudo -S pidin -F '%a' ar 2>/dev/null")
    names = {"usb_relay": "tools.usb_relay", "rangefinder": "tools.rangefinder",
             "hbwd": "hbwd", "edge": "-m edge ", "camgrab_fwd": "camgrab_fwd",
             "camgrab_down": "camgrab_down", "flow": "edge.flow", "vision": "edge.vision",
             "video": "edge.video", "blackbox": "tools.blackbox",
             "camrec": "tools.camrec"}
    missing = [k for k in CHILDREN if names[k] not in procs]
    check("drone processes running", not missing,
          "all 10" if not missing else "missing: " + ", ".join(missing))

    fps = run(f"env {PYPATH} python3 -c \"import struct,time,mmap,os\n"
              "for n in ('camgrab_fwd','camgrab_down'):\n"
              "    try:\n"
              "        fd=os.open('/dev/shmem/'+n,os.O_RDONLY); b=mmap.mmap(fd,os.fstat(fd).st_size,prot=mmap.PROT_READ)\n"
              "        s1=struct.unpack_from('<Q',b,24)[0]; time.sleep(2); s2=struct.unpack_from('<Q',b,24)[0]; print(n,(s2-s1)/4)\n"
              "    except Exception as e: print(n,-1)\"")
    for line in fps.split("\n"):
        parts = line.split()
        if len(parts) == 2 and parts[0].startswith("camgrab"):
            f = float(parts[1])
            check(f"camera {parts[0][8:]}", f >= 8, f"{f:.0f} fps" if f >= 0 else "no frames")

    # edge.vision logs "running: X Hz, infer Y ms" every 10 s while it works, and
    # "no new frame" when the camera stalls; the newest of the two decides
    vis = [l for l in run("tail -60 logs/vision.log 2>/dev/null").splitlines()
           if "running:" in l or "no new frame" in l or "can't start" in l]
    last = vis[-1] if vis else ""
    age = log_age_s(last, run("date '+%Y-%m-%d %H:%M:%S'").strip())
    info = last.split("vision ", 1)[-1][:70] if last else "no log yet (wait 10 s after start)"
    check("on-board person detector", "running:" in last and age is not None and age < 25,
          f"{info} ({age:.0f} s ago)" if age is not None else info)

    pf = run(f"env {PYPATH} python3 -m tools.preflight", t=120)
    go = ">>> GO <<<" in pf
    nogo = [l[6:].strip() for l in pf.splitlines() if l.startswith("NO-GO")]
    check("preflight (FC + sensors)", go, "ALL GO" if go else "; ".join(nogo)[:150])
    return True


async def edge_checks() -> None:
    from shared.client import CommandRejected, EdgeClient
    try:
        edge = await asyncio.wait_for(EdgeClient(HOST).connect(), 6)
    except Exception as e:
        check("laptop -> edge protocol", False, f"{e} (is this laptop in --station-ip?)")
        return
    await asyncio.sleep(1.5)
    t = edge.telemetry or {}
    check("edge telemetry", bool(t), f"state={t.get('state')} mode={t.get('mode')} "
          f"range={t.get('range_m')} flow_q={t.get('flow_quality')} batt={t.get('battery_v')}")
    if t.get("armed"):
        check("edge safety gate", False, "drone is ARMED: skipping the gate test")
        return
    try:
        await edge.run("move_body", fwd=-0.5, timeout=5)
        check("edge safety gate", False, "a move while landed was NOT refused")
    except CommandRejected as e:
        demo = t.get("mode") == "GUIDED-DEMO" or "indoors" in str(e)  # edge --demo refuses all
        check("edge safety gate", demo or "not flying" in str(e) or "take off" in str(e),
              ("DEMO MODE, refused: " if demo else "refused: ") + str(e)[:70])


def http_checks(env: dict) -> None:
    t0 = time.time()
    try:
        r = urllib.request.urlopen(f"http://{HOST}:8080/forward.jpg", timeout=5)
        n = len(r.read())
        check("video stream", n > 5000, f"{n} bytes in {1000 * (time.time() - t0):.0f} ms, "
              f"pose age {r.headers.get('X-Pose-Age')}")
    except Exception as e:
        check("video stream", False, str(e))
    key = env.get("VULTR_INFERENCE_KEY") or os.environ.get("VULTR_INFERENCE_KEY")
    if key:
        try:  # a real 1-token call: /v1/models answers even with a dead key
            body = json.dumps({"model": env.get("BRAIN_MODEL", "glm-5.3-flash"),
                               "messages": [{"role": "user", "content": "hi"}],
                               "max_tokens": 1}).encode()
            req = urllib.request.Request("https://api.vultrinference.com/v1/chat/completions",
                                         data=body, headers={"Authorization": f"Bearer {key}",
                                                             "Content-Type": "application/json"})
            t0 = time.time()
            json.load(urllib.request.urlopen(req, timeout=20))
            check("Vultr AI (API key)", True, f"brain answered in {1000 * (time.time() - t0):.0f} ms")
        except Exception as e:
            check("Vultr AI (API key)", False, f"{e} (regenerate the key, update .env, restart)")
    else:
        check("Vultr AI (API key)", False, "no VULTR_INFERENCE_KEY in .env")
    cloud = env.get("CLOUD_URL") or os.environ.get("CLOUD_URL")
    if cloud:
        url = cloud.replace("wss://", "https://").replace("ws://", "http://").rstrip("/") + "/health"
        try:
            h = json.load(urllib.request.urlopen(url, timeout=8))
            check("cloud server /health", True, json.dumps(h)[:120])
        except Exception as e:
            check("cloud server /health", False, f"{url}: {e}")
    else:
        check("cloud server /health", False, "no CLOUD_URL in .env (cloud not deployed yet)")


def main() -> int:
    env = env_file()
    print(f"system check against the drone at {HOST}\n")
    if ssh_checks():
        asyncio.run(edge_checks())
    http_checks(env)
    bad = [n for n, ok, _ in results if not ok]
    print("\nALL LINKS OK" if not bad else f"\n{len(bad)} link(s) need attention: {', '.join(bad)}")
    return 0 if not bad else 1


if __name__ == "__main__":
    raise SystemExit(main())
