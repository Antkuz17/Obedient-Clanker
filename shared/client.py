"""Async client for edge protocol v1. Used by the planner, the dashboard bridge, the CLI and tests.

    async with EdgeClient("192.168.x.y") as edge:
        await edge.run("takeoff", alt=1.0)          # waits for done, returns the done message
        await edge.run("move_body", fwd=1.0)
        print(edge.telemetry["n"])

The heartbeat and the socket reader run on their **own threads** (review A10), so a slow STT or
LLM call that blocks the caller's event loop can't make the drone hover or land. But the
heartbeat only beats while the event loop has checked in within `liveness_s` (review2 #7): a
planner that is truly hung stops the heartbeat, and the edge's watchdog takes over. Received
messages are handed to the event loop with call_soon_threadsafe.
"""
from __future__ import annotations

import asyncio
import itertools
import socket
import threading
import time
from typing import Callable

from . import protocol as P


class CommandRejected(Exception):
    pass


class EdgeClient:
    def __init__(self, host: str = "127.0.0.1", port: int = P.EDGE_PORT,
                 on_event: Callable[[dict], None] | None = None, liveness_s: float = 3.0):
        self.host, self.port = host, port
        self.on_event = on_event
        self.telemetry: dict = {}
        self.events: list[dict] = []
        self._ids = itertools.count(1)
        self._acks: dict[str, asyncio.Future] = {}
        self._dones: dict[str, asyncio.Future] = {}
        self.heartbeat = True  # set False to simulate a dead station
        self.liveness_s = liveness_s
        self._loop_alive = time.monotonic()
        self.connected = False
        self._sock: socket.socket | None = None
        self._wlock = threading.Lock()
        self._stop = threading.Event()
        self._loop: asyncio.AbstractEventLoop | None = None

    async def connect(self) -> "EdgeClient":
        self._loop = asyncio.get_running_loop()
        self._sock = await self._loop.run_in_executor(
            None, lambda: socket.create_connection((self.host, self.port), timeout=5))
        self._sock.settimeout(None)
        self._sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self.connected = True
        self._loop_alive = time.monotonic()
        self._pet_task = asyncio.create_task(self._pet())
        threading.Thread(target=self._rx, daemon=True, name="edge-rx").start()
        threading.Thread(target=self._hb, daemon=True, name="edge-hb").start()
        return self

    async def _pet(self) -> None:
        """Proves the event loop (the planner) is alive; the heartbeat thread checks it."""
        while not self._stop.is_set():
            self._loop_alive = time.monotonic()
            await asyncio.sleep(0.1)

    async def close(self) -> None:
        self._stop.set()
        if getattr(self, "_pet_task", None):
            self._pet_task.cancel()
        self.connected = False
        if self._sock:
            try:
                self._sock.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            self._sock.close()

    async def __aenter__(self):
        return await self.connect()

    async def __aexit__(self, *exc):
        await self.close()

    def _send(self, msg: dict) -> None:
        with self._wlock:
            self._sock.sendall(P.encode(msg))

    def _hb(self) -> None:
        period = 1.0 / P.HEARTBEAT_HZ
        while not self._stop.is_set():
            if self.heartbeat and time.monotonic() - self._loop_alive < self.liveness_s:
                try:
                    self._send(P.heartbeat(time.time()))
                except OSError:
                    break
            self._stop.wait(period)

    def _rx(self) -> None:
        f = self._sock.makefile("rb")
        try:
            for line in f:
                try:
                    m = P.decode(line)
                except P.ProtocolError:
                    continue
                try:
                    self._loop.call_soon_threadsafe(self._dispatch, m)
                except RuntimeError:  # event loop closed (caller exited): stop quietly
                    break
        except (OSError, ValueError):
            pass
        finally:
            self.connected = False

    def _dispatch(self, m: dict) -> None:
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
        # hold the future locally: the ack can arrive (and _dispatch pop it) while the send is
        # still in the executor, so a later self._acks[cid] lookup raised KeyError
        ack_f = self._acks[cid] = loop.create_future()
        self._dones[cid] = loop.create_future()
        try:
            await loop.run_in_executor(None, self._send, P.cmd(cid, op, **args))
            ack = await asyncio.wait_for(ack_f, 5.0)
        except BaseException:
            self._acks.pop(cid, None)
            self._dones.pop(cid, None)
            raise
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
