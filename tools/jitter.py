"""Control-loop jitter benchmark for the QNX demo (PRD: "jitter under 2 ms at 100% CPU").

Runs a periodic loop shaped like the edge safety loop (absolute deadlines, 50 Hz) and measures
how late each wake-up is, optionally while every core is pegged by a CPU hog, with and without
SCHED_FIFO. Stdlib only, so it runs on the Pi's Python 3.14.

  python3 -m tools.jitter --seconds 10                  # one run, normal priority, idle
  python3 -m tools.jitter --hog --prio 50               # CPU hog on all cores, loop at FIFO 50
  sudo python3 -m tools.jitter --compare --out jitter.json   # the 4 demo cases, JSON for the graph
  sudo python3 -m tools.jitter --compare --udp <laptop>:7010   # ... and onto the dashboard

Each wake-up's lateness = actual wake time - scheduled deadline (ms). The demo claim is the
difference between "hog, no prio" (tail blows up) and "hog, FIFO" (stays flat).
The hog runs at normal priority, so FIFO only helps if it was really applied: the result says
`prio_applied` (setting it needs root on QNX and Linux; macOS has no SCHED_FIFO).
"""
from __future__ import annotations

import argparse
import json
import multiprocessing as mp
import os
import sys
import time


def _hog() -> None:
    try:
        os.nice(5)  # explicitly below the loop's normal priority: a fair fight for "no prio"
    except OSError:
        pass
    x = 0
    while True:
        x = (x * 1103515245 + 12345) & 0x7FFFFFFF


def start_hogs(n: int | None = None) -> list[mp.Process]:
    ps = [mp.Process(target=_hog, daemon=True) for _ in range(n or os.cpu_count() or 1)]
    for p in ps:
        p.start()
    return ps


def set_fifo(prio: int) -> bool:
    if not prio or not hasattr(os, "sched_setscheduler"):
        return False
    try:
        os.sched_setscheduler(0, os.SCHED_FIFO, os.sched_param(prio))
        return True
    except (OSError, PermissionError):
        return False


def set_normal() -> None:
    if hasattr(os, "sched_setscheduler"):
        try:
            os.sched_setscheduler(0, os.SCHED_OTHER, os.sched_param(0))
        except OSError:
            pass


def measure(seconds: float, hz: float = 50.0) -> list[float]:
    """Lateness of each wake-up in ms, using absolute deadlines (no drift)."""
    period = 1.0 / hz
    late: list[float] = []
    nxt = time.monotonic() + period
    end = nxt + seconds
    while nxt < end:
        d = nxt - time.monotonic()
        if d > 0:
            time.sleep(d)
        late.append((time.monotonic() - nxt) * 1000.0)
        nxt += period
        if time.monotonic() > nxt + period:  # badly overrun: resync, count it once
            nxt = time.monotonic() + period
    return late


def summarize(late: list[float]) -> dict:
    s = sorted(late)
    def pct(p: float) -> float:
        return round(s[min(len(s) - 1, int(p / 100 * len(s)))], 3)
    return {"n": len(s), "p50_ms": pct(50), "p99_ms": pct(99), "p999_ms": pct(99.9),
            "max_ms": round(s[-1], 3), "over_2ms": sum(v > 2.0 for v in s)}


def histogram(late: list[float], edges=(0.1, 0.25, 0.5, 1, 2, 5, 10, 20, 50), width=40) -> str:
    counts = [0] * (len(edges) + 1)
    for v in late:
        i = next((k for k, e in enumerate(edges) if v < e), len(edges))
        counts[i] += 1
    top = max(counts) or 1
    labels = [f"<{e:g}" for e in edges] + [f">={edges[-1]:g}"]
    rows = []
    for lab, c in zip(labels, counts):
        bar = "#" * (round(c / top * width) if c else 0)
        rows.append(f"  {lab:>6} ms | {bar:<{width}} {c}")
    return "\n".join(rows)


def run_case(seconds: float, hz: float, hog: bool, prio: int) -> dict:
    hogs = start_hogs() if hog else []
    try:
        time.sleep(0.3 if hog else 0)  # let the hogs get going
        applied = set_fifo(prio)
        late = measure(seconds, hz)
    finally:
        set_normal()
        for p in hogs:
            p.terminate()
        for p in hogs:
            p.join(2)
    return {"hog": hog, "prio": prio, "prio_applied": applied, "hz": hz, "seconds": seconds,
            "platform": sys.platform, "cpus": os.cpu_count(), **summarize(late),
            "late_ms": [round(v, 3) for v in late]}


HIST_EDGES = (0.1, 0.25, 0.5, 1, 2, 5, 10, 20, 50)


def bucket_counts(late: list[float], edges=HIST_EDGES) -> list[int]:
    counts = [0] * (len(edges) + 1)
    for v in late:
        counts[next((k for k, e in enumerate(edges) if v < e), len(edges))] += 1
    return counts


def udp_summary(r: dict) -> dict:
    """One case, small enough for a datagram: the summary + histogram counts, no samples."""
    return {"src": "jitter", "t": time.time(), "kind": "jitter_case",
            **{k: v for k, v in r.items() if k != "late_ms"},
            "edges_ms": list(HIST_EDGES), "counts": bucket_counts(r["late_ms"])}


def send_udp(addr: str, msg: dict) -> None:
    import socket
    host, _, port = addr.rpartition(":")
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        try:
            sock.sendto(json.dumps(msg).encode(), (host or "127.0.0.1", int(port)))
        except OSError as e:
            print(f"udp send failed: {e}", file=sys.stderr)


def report(r: dict) -> str:
    name = f"{'hog' if r['hog'] else 'idle'}, " + (
        f"FIFO {r['prio']}" if r["prio_applied"] else
        ("normal prio" if not r["prio"] else f"FIFO {r['prio']} NOT applied (need root)"))
    return (f"{name}: p50 {r['p50_ms']} ms, p99 {r['p99_ms']} ms, max {r['max_ms']} ms, "
            f"{r['over_2ms']}/{r['n']} wake-ups over 2 ms\n" + histogram(r["late_ms"]))


def main(argv: list[str] | None = None) -> None:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--seconds", type=float, default=10.0)
    ap.add_argument("--hz", type=float, default=50.0)
    ap.add_argument("--hog", action="store_true", help="peg every core while measuring")
    ap.add_argument("--prio", type=int, default=0, help="SCHED_FIFO priority (0 = normal)")
    ap.add_argument("--compare", action="store_true",
                    help="idle/hog x normal/FIFO (uses --prio, default 50)")
    ap.add_argument("--out", help="write results (with every sample) as JSON")
    ap.add_argument("--udp", help="send each case's summary to host:port (station :7010 -> dashboard)")
    a = ap.parse_args(argv)
    if a.compare:
        prio = a.prio or 50
        cases = [(False, 0), (False, prio), (True, 0), (True, prio)]
    else:
        cases = [(a.hog, a.prio)]
    results = []
    for hog, prio in cases:
        r = run_case(a.seconds, a.hz, hog, prio)
        print(report(r), flush=True)
        results.append(r)
        if a.udp:
            send_udp(a.udp, udp_summary(r))
    if a.out:
        with open(a.out, "w") as f:
            json.dump(results, f)
        print(f"wrote {a.out}")


if __name__ == "__main__":
    main(sys.argv[1:])
