"""Async client for edge protocol v1. Used by the planner, the dashboard bridge, the CLI and tests.

    async with EdgeClient("qnxpi.local") as edge:
        await edge.run("takeoff", alt=1.0)          # waits for done, returns the done message
        await edge.run("move_body", fwd=1.0)
        print(edge.telemetry["n"])

Heartbeats are sent automatically while connected (that's what keeps the watchdog happy).
"""
from __future__ import annotations

import asyncio
import itertools
import time
from typing import Callable

from . import protocol as P


class CommandRejected(Exception):
    pass


class EdgeClient:
    def __init__(self, host: str = "127.0.0.1", port: int = P.EDGE_PORT,
                 on_event: Callable[[dict], None] | None = None):
        self.host, self.port = host, port
        self.on_event = on_event
        self.telemetry: dict = {}
        self.events: list[dict] = []
        self._ids = itertools.count(1)
        self._acks: dict[str, asyncio.Future] = {}
        self._dones: dict[str, asyncio.Future] = {}
        self._tasks: list[asyncio.Task] = []
        self.heartbeat = True  # set False to simulate a dead station
        self._w: asyncio.StreamWriter | None = None

    async def connect(self) -> "EdgeClient":
        r, self._w = await asyncio.open_connection(self.host, self.port)
        self._tasks = [asyncio.create_task(self._rx(r)), asyncio.create_task(self._hb())]
        return self

    async def close(self) -> None:
        for t in self._tasks:
            t.cancel()
        if self._w:
            self._w.close()

    async def __aenter__(self):
        return await self.connect()

    async def __aexit__(self, *exc):
        await self.close()

    def _send(self, msg: dict) -> None:
        self._w.write(P.encode(msg))

    async def _hb(self) -> None:
        while True:
            if self.heartbeat:
                self._send(P.heartbeat(time.time()))
            await asyncio.sleep(1.0 / P.HEARTBEAT_HZ)

    async def _rx(self, r: asyncio.StreamReader) -> None:
        while line := await r.readline():
            m = P.decode(line)
            t = m["type"]
            if t == P.TELEMETRY:
                self.telemetry = m
            elif t == P.ACK and (f := self._acks.pop(m["id"], None)) and not f.done():
                f.set_result(m)
            elif t == P.DONE and (f := self._dones.get(m["id"])) and not f.done():
                f.set_result(m)
            elif t == P.EVENT:
                self.events.append(m)
                if self.on_event:
                    self.on_event(m)

    async def send(self, op: str, **args) -> tuple[str, dict]:
        """Send a command and wait for its ack. Returns (id, ack). Doesn't wait for done."""
        cid = f"c{next(self._ids)}"
        loop = asyncio.get_running_loop()
        self._acks[cid] = loop.create_future()
        self._dones[cid] = loop.create_future()
        self._send(P.cmd(cid, op, **args))
        ack = await asyncio.wait_for(self._acks[cid], 5.0)
        if not ack["ok"]:
            self._dones.pop(cid, None)
        return cid, ack

    async def wait_done(self, cid: str, timeout: float = 60.0) -> dict:
        try:
            return await asyncio.wait_for(self._dones[cid], timeout)
        finally:
            self._dones.pop(cid, None)

    def is_done(self, cid: str) -> bool:
        f = self._dones.get(cid)
        return f is None or f.done()

    async def run(self, op: str, timeout: float = 60.0, **args) -> dict:
        """Send, raise CommandRejected if the safety gate says no, else wait for done."""
        cid, ack = await self.send(op, **args)
        if not ack["ok"]:
            raise CommandRejected(ack["reason"])
        return await self.wait_done(cid, timeout)
