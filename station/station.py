"""The laptop station: edge link + planner + the outbound link to the Vultr cloud.

Cloud mode (CLOUD_URL set): the laptop dials out to `wss://<cloud>/ws/station` (NAT-safe),
streams telemetry (5 Hz), video frames (~4 fps), edge events and memory frames up, and runs
the `actions` the cloud's brain sends down. Replies arrive as mp3 and play on the laptop's
speaker (the Nest Mini). Reconnects with backoff; the edge link is separate, so the cloud
dying never touches the heartbeat.

Local mode (`--local`, no cloud): the brain, STT and TTS run in this process; commands come
from the terminal (text, or Enter for the push-to-talk mic).
"""
from __future__ import annotations

import asyncio
import base64
import itertools
import json
import logging
import math
import time

from shared.client import EdgeClient

from .ai import AI
from .audio import Speaker
from .brain import Brain
from .camera import EdgeCamera, Frame
from .config import MissingKey, Settings
from .planner import Planner

log = logging.getLogger("station")

TELEMETRY_UP_HZ = 5
FRAME_UP_FPS = 4
MEMORY_EVERY_M = 0.5       # ingest a memory frame after moving this far
MEMORY_EVERY_DEG = 30.0    # ... or turning this much
MEMORY_EVERY_S = 15.0      # ... or this long
CLOUD_LOST_LAND_S = 30.0   # cloud gone this long while airborne and idle -> land (review2)
EDGE_RETRY_S = (1.0, 5.0)  # edge reconnect backoff: first, max
EDGE_SILENT_S = 3.0        # no telemetry this long = link dead (Pi lost power, Wi-Fi gone)


class Station:
    def __init__(self, s: Settings, *, local: bool = False, stub_locate: bool = False,
                 speaker: bool = True, ai: AI | None = None):
        self.s = s
        self.local = local
        self.ai = ai or AI(s)
        self.speaker = Speaker(speaker)
        self.edge = EdgeClient(s.edge_host, s.edge_port, on_event=self._on_edge_event)
        self.camera = EdgeCamera(s.edge_host, s.video_port)
        if stub_locate:
            from .localize_stub import locate
        else:
            from .localize import locate
        self.planner = Planner(self.edge, self.camera, locate=locate,
                               ai=self.ai if s.vultr_key else None, memory=self.memory_search,
                               say=self.say, on_photo=self.on_photo)
        self.brain = Brain(self.ai if s.vultr_key else None)
        self.ws = None
        self._req = itertools.count(1)
        self._mem_wait: dict[int, asyncio.Future] = {}
        self._mem_last: tuple[float, float, float, float] | None = None
        self._local_memory = None
        self.cloud_lost_land_s = CLOUD_LOST_LAND_S
        self._cloud_down_since: float | None = time.monotonic()
        self._cloud_landed = False  # one landing per outage
        self._edge_ready = asyncio.Event()

    # --- cloud link --------------------------------------------------------------------------
    async def _send_cloud(self, msg: dict) -> bool:
        if self.ws is None:
            return False
        try:
            await self.ws.send(json.dumps(msg))
            return True
        except Exception:
            return False

    async def cloud_loop(self) -> None:
        import websockets

        url = f"{self.s.cloud_url.rstrip('/')}/ws/station?token={self.s.station_token}"
        backoff = 1.0
        while True:
            try:
                async with websockets.connect(url, ping_interval=15, ping_timeout=20,
                                              max_size=8 * 2**20) as ws:
                    self.ws, backoff = ws, 1.0
                    self._cloud_down_since, self._cloud_landed = None, False
                    log.info("cloud connected")
                    async for raw in ws:
                        await self._from_cloud(json.loads(raw))
            except asyncio.CancelledError:
                raise
            except Exception as e:
                log.warning("cloud link down (%s); retrying in %.0f s", e, backoff)
            if self.ws is not None or self._cloud_down_since is None:
                self._cloud_down_since = time.monotonic()
            self.ws = None
            await asyncio.sleep(backoff)
            backoff = min(backoff * 2, 15.0)

    async def _from_cloud(self, m: dict) -> None:
        t = m.get("type")
        if t == "actions":
            log.info("actions %s", m.get("actions"))
            self.planner.submit(m.get("actions") or [])
        elif t == "audio":
            self.speaker.play(base64.b64decode(m.get("mp3", "")))
        elif t == "memory_result" and (f := self._mem_wait.pop(m.get("req"), None)):
            if not f.done():
                f.set_result(m.get("hits") or [])

    def _airborne(self) -> bool:
        t = self.edge.telemetry or {}
        return bool(self.edge.connected and t.get("armed") and t.get("alt", 0.0) > 0.2)

    async def cloud_watchdog(self) -> None:
        """Review2: if the cloud dies, nothing can tell a hovering drone to come down. After
        CLOUD_LOST_LAND_S with no cloud, airborne and no plan running: say so and land."""
        while True:
            await asyncio.sleep(0.5)
            since = self._cloud_down_since
            if (since is None or self._cloud_landed or self.planner.busy
                    or time.monotonic() - since < self.cloud_lost_land_s or not self._airborne()):
                continue
            self._cloud_landed = True
            log.warning("cloud lost for %.0f s while hovering: landing", time.monotonic() - since)
            await self.say("I've lost the cloud, so I'm landing.")
            self.planner.submit([{"kind": "land"}])

    # --- edge link ---------------------------------------------------------------------------
    async def edge_loop(self) -> None:
        """Keep the edge link up. On a drop: cancel the plan (the edge's watchdog hovers, then
        lands, on its own) and reconnect with backoff, with a fresh client."""
        backoff = EDGE_RETRY_S[0]
        was_up = False
        last_tel, last_tel_t = None, time.monotonic()
        while True:
            if self.edge.connected:
                await asyncio.sleep(0.2)
                # A powered-off Pi never closes the socket: telemetry (10 Hz) going quiet is
                # the only sign. The client replaces the dict on every telemetry message.
                if self.edge.telemetry is not last_tel:
                    last_tel, last_tel_t = self.edge.telemetry, time.monotonic()
                elif was_up and time.monotonic() - last_tel_t > EDGE_SILENT_S:
                    log.warning("no telemetry for %.0f s", EDGE_SILENT_S)
                    await self.edge.close()
                continue
            if was_up:
                was_up = False
                self._edge_ready.clear()
                self.planner.cancel()
                log.warning("edge link lost")
                await self.say("I've lost the link to the drone. It will hover, then land on "
                               "its own.")
            c = EdgeClient(self.s.edge_host, self.s.edge_port, on_event=self._on_edge_event)
            try:
                await c.connect()
            except OSError as e:
                log.info("edge %s:%d not reachable (%s); retry in %.0f s", self.s.edge_host,
                         self.s.edge_port, e, backoff)
                await asyncio.sleep(backoff)
                backoff = min(backoff * 2, EDGE_RETRY_S[1])
                continue
            old, self.edge = self.edge, c
            self.planner.edge = c
            try:
                await old.close()
            except Exception:
                pass
            backoff, was_up = EDGE_RETRY_S[0], True
            last_tel_t = time.monotonic()
            self._edge_ready.set()
            log.info("edge connected at %s:%d", self.s.edge_host, self.s.edge_port)

    def _on_edge_event(self, ev: dict) -> None:
        asyncio.ensure_future(self._send_cloud(ev))

    async def uplink_loop(self) -> None:
        """Telemetry at 5 Hz; a frame every 1/FRAME_UP_FPS s; memory frames on movement."""
        frame_every = 1.0 / FRAME_UP_FPS
        next_frame = 0.0
        while True:
            await asyncio.sleep(1.0 / TELEMETRY_UP_HZ)
            if self.edge.telemetry:
                await self._send_cloud(self.edge.telemetry)
            now = time.monotonic()
            if self.ws is None or now < next_frame:
                continue
            next_frame = now + frame_every
            try:
                fr = await self.camera.frame()
            except Exception:
                continue
            b64 = base64.b64encode(fr.jpeg).decode()
            await self._send_cloud({"type": "frame", "jpeg": b64, "pose": fr.pose.as_dict()})
            if fr.pose.age <= 0.5 and self._memory_due(fr, now):  # stale pose: bad memory
                await self._send_cloud({"type": "memory_frame", "jpeg": b64,
                                        "pose": fr.pose.as_dict()})

    def _memory_due(self, fr: Frame, now: float) -> bool:
        p = fr.pose
        if self._mem_last is not None:
            n, e, yaw, t = self._mem_last
            dyaw = abs((p.yaw - yaw + 180) % 360 - 180)
            if (math.hypot(p.n - n, p.e - e) < MEMORY_EVERY_M and dyaw < MEMORY_EVERY_DEG
                    and now - t < MEMORY_EVERY_S):
                return False
        self._mem_last = (p.n, p.e, p.yaw, now)
        return True

    # --- planner hooks -----------------------------------------------------------------------
    async def say(self, text: str) -> None:
        log.info("jarvis: %s", text)
        if await self._send_cloud({"type": "say", "text": text}):
            return
        print(f"  jarvis: {text}")
        try:
            self.speaker.play(await self.ai.speak(text))
        except Exception as e:
            log.debug("local TTS failed: %s", e)

    async def memory_search(self, q: str) -> list[dict]:
        if self.ws is not None:
            req = next(self._req)
            fut = asyncio.get_running_loop().create_future()
            self._mem_wait[req] = fut
            await self._send_cloud({"type": "memory_search", "q": q, "req": req})
            try:
                return await asyncio.wait_for(fut, 10.0)
            finally:
                self._mem_wait.pop(req, None)
        if self._local_memory is None:
            from cloud.memory import Memory
            self._local_memory = Memory(self.s.data_dir, self.ai)
        return await self._local_memory.search(q)

    async def on_photo(self, fr: Frame, caption: str) -> None:
        b64 = base64.b64encode(fr.jpeg).decode()
        if not await self._send_cloud({"type": "photo", "jpeg": b64, "pose": fr.pose.as_dict(),
                                       "caption": caption}):
            path = self.s.data_dir / "photos" / f"{int(time.time())}.jpg"
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_bytes(fr.jpeg)
            print(f"  photo saved: {path}")

    # --- local mode --------------------------------------------------------------------------
    async def handle_text(self, text: str) -> None:
        r = await self.brain.think(text, self.edge.telemetry)
        print(f"  [{r.source}] {r.actions}")
        await self.say(r.reply)
        self.planner.submit(r.actions)

    async def local_repl(self, mic: bool = False) -> None:
        loop = asyncio.get_running_loop()
        print("Type a command (or press Enter to talk)." if mic else "Type a command.")
        while True:
            line = await loop.run_in_executor(None, input, "> ")
            if not line.strip() and mic:
                from .audio import record_wav
                wav = await loop.run_in_executor(None, record_wav)
                try:
                    line = await self.ai.transcribe(wav, "clip.wav")
                except MissingKey as e:
                    print(f"  {e}")
                    continue
                print(f"  heard: {line}")
            if line.strip() in ("quit", "exit"):
                self.planner.submit([{"kind": "land"}])
                return
            if line.strip():
                await self.handle_text(line)

    # --- run ---------------------------------------------------------------------------------
    async def run(self, mic: bool = False) -> None:
        tasks = [asyncio.create_task(self.edge_loop())]
        try:
            if self.local or not self.s.cloud_url:
                await self._edge_ready.wait()
                await self.local_repl(mic)
                await self.planner.wait()
            else:
                tasks += [asyncio.create_task(self.cloud_loop()),
                          asyncio.create_task(self.uplink_loop()),
                          asyncio.create_task(self.cloud_watchdog())]
                await asyncio.gather(*tasks)
        finally:
            for t in tasks:
                t.cancel()
            await self.edge.close()
            await self.camera.aclose()
            await self.ai.aclose()
