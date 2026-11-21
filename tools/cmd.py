"""Interactive protocol REPL.  python -m tools.cmd [host]

  > takeoff alt=1
  > move_body fwd=1 right=-0.5
  > yaw deg=90 relative=true
  > t            (print telemetry)
  > land
"""
from __future__ import annotations

import asyncio
import json
import sys

from shared import protocol as P
from shared.client import EdgeClient


def parse(line: str) -> tuple[str, dict]:
    op, *kv = line.split()
    args = {}
    for item in kv:
        k, _, v = item.partition("=")
        args[k] = {"true": True, "false": False}.get(v.lower(), None)
        if args[k] is None:
            args[k] = float(v)
    return op, args


async def main(host: str) -> None:
    edge = await EdgeClient(host, on_event=lambda e: print(f"\n!! {e['kind']}: {e['detail']}")
                            ).connect()
    print(f"connected to {host}:{P.EDGE_PORT}. ops: {', '.join(P.OPS)}; 't' = telemetry")
    loop = asyncio.get_running_loop()
    while True:
        line = (await loop.run_in_executor(None, sys.stdin.readline)).strip()
        if not line:
            continue
        if line in ("q", "quit", "exit"):
            break
        if line == "t":
            print(json.dumps(edge.telemetry, indent=1))
            continue
        try:
            op, args = parse(line)
            cid, ack = await edge.send(op, **args)
        except (ValueError, asyncio.TimeoutError) as e:
            print("error:", e)
            continue
        if not ack["ok"]:
            print("REJECTED:", ack["reason"])
            continue
        print("accepted", cid)

        async def report(cid=cid):
            d = await edge.wait_done(cid, timeout=120)
            print(f"\n{cid} {d['status']} {d['reason']}")
        asyncio.create_task(report())
    await edge.close()


if __name__ == "__main__":
    asyncio.run(main(sys.argv[1] if len(sys.argv) > 1 else "127.0.0.1"))
