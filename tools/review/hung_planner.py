"""Review 2 bug 7, with the edge in its OWN process (as on the drone). sim_exp's E9 runs the
edge and the station in one event loop, so freezing the station also froze the edge's watchdog.

  .venv/bin/python -m tools.review.hung_planner
"""
import asyncio, subprocess, sys, time

from shared.client import EdgeClient


async def main():
    edge_proc = subprocess.Popen([sys.executable, "-m", "edge", "--sim", "--log", "",
                                  "--port", "7130", "--video-port", "8130",
                                  "--perception-port", "7131", "--flow-port", "7132",
                                  "--no-video"], stdout=subprocess.DEVNULL,
                                 stderr=subprocess.DEVNULL)
    try:
        await asyncio.sleep(2)
        edge = await EdgeClient("127.0.0.1", 7130).connect()
        await edge.run("takeoff", alt=1.0)
        print("flying:", edge.telemetry.get("state"))
        print("station planner hangs for 10 s (event loop blocked)...")
        time.sleep(10)
        await asyncio.sleep(0.5)
        for e in edge.events:
            print("   event:", e["kind"], e["detail"])
        print("edge state after the hang:", edge.telemetry.get("state"),
              "armed:", edge.telemetry.get("armed"), "alt:", edge.telemetry.get("alt"))
        await edge.close()
    finally:
        edge_proc.terminate()

asyncio.run(main())
