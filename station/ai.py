"""Clients for the hosted AI: Vultr Serverless Inference (OpenAI-compatible) and ElevenLabs.

One class, one httpx.AsyncClient. Swappable: point VULTR_INFERENCE_BASE at any
OpenAI-compatible server. Tests pass `transport=httpx.MockTransport(...)`.

    ai = AI(settings())
    msg = await ai.chat([...], tools=[...])          # -> OpenAI "message" dict
    text = await ai.vision("What does it say?", jpeg)
    vecs = await ai.embed(["a red chair"])
    text = await ai.transcribe(wav_bytes, "clip.wav")
    mp3 = await ai.speak("On it.")                   # ElevenLabs, then Vultr TTS, else b""
"""
from __future__ import annotations

import asyncio
import base64
import logging
import shutil
import time

import httpx

from .config import MissingKey, Settings

log = logging.getLogger("ai")
ELEVEN_BASE = "https://api.elevenlabs.io/v1"


class AI:
    def __init__(self, s: Settings, transport: httpx.AsyncBaseTransport | None = None,
                 timeout: float = 20.0):
        self.s = s
        self.http = httpx.AsyncClient(timeout=timeout, transport=transport)
        self.last_latency: dict[str, float] = {}

    async def aclose(self) -> None:
        await self.http.aclose()

    # --- Vultr -------------------------------------------------------------------------------
    def _vultr_headers(self) -> dict:
        return {"Authorization": f"Bearer {self.s.need('vultr_key', 'VULTR_INFERENCE_KEY')}"}

    async def _post(self, kind: str, path: str, **kw) -> httpx.Response:
        t0 = time.monotonic()
        r = await self.http.post(f"{self.s.vultr_base}{path}", headers=self._vultr_headers(), **kw)
        self.last_latency[kind] = time.monotonic() - t0
        r.raise_for_status()
        return r

    async def chat(self, messages: list[dict], tools: list[dict] | None = None,
                   model: str | None = None, temperature: float = 0.4,
                   max_tokens: int = 300) -> dict:
        """One chat completion; returns choices[0].message. Falls back to the backup model."""
        body = {"model": model or self.s.brain_model, "messages": messages,
                "temperature": temperature, "max_tokens": max_tokens}
        if tools:
            body["tools"] = tools
            body["tool_choice"] = "auto"
        try:
            r = await self._post("chat", "/chat/completions", json=body)
        except (httpx.HTTPStatusError, httpx.TransportError) as e:
            if model or not self.s.brain_backup_model:
                raise
            log.warning("brain model failed (%s); trying %s", e, self.s.brain_backup_model)
            body["model"] = self.s.brain_backup_model
            r = await self._post("chat", "/chat/completions", json=body)
        return r.json()["choices"][0]["message"]

    async def vision(self, prompt: str, jpeg: bytes, system: str = "",
                     max_tokens: int = 400) -> str:
        url = "data:image/jpeg;base64," + base64.b64encode(jpeg).decode()
        msgs = ([{"role": "system", "content": system}] if system else []) + [{
            "role": "user",
            "content": [{"type": "text", "text": prompt},
                        {"type": "image_url", "image_url": {"url": url}}]}]
        body = {"model": self.s.vision_model, "messages": msgs, "temperature": 0.1,
                "max_tokens": max_tokens}
        r = await self._post("vision", "/chat/completions", json=body)
        return (r.json()["choices"][0]["message"].get("content") or "").strip()

    async def embed(self, texts: list[str]) -> list[list[float]]:
        r = await self._post("embed", "/embeddings",
                             json={"model": self.s.embed_model, "input": texts})
        data = sorted(r.json()["data"], key=lambda d: d.get("index", 0))
        return [d["embedding"] for d in data]

    async def rerank(self, query: str, docs: list[str]) -> list[float]:
        """Relevance of each doc to the query (Cohere/Jina-style /rerank). Higher = better."""
        r = await self._post("rerank", "/rerank", json={
            "model": self.s.rerank_model, "query": query, "documents": docs})
        out = [0.0] * len(docs)
        for x in r.json().get("results", []):
            out[x["index"]] = float(x.get("relevance_score", x.get("score", 0.0)))
        return out

    async def transcribe(self, audio: bytes, filename: str = "clip.webm") -> str:
        wav = await to_wav16k(audio)
        name = "clip.wav" if wav is not audio else filename
        r = await self._post("stt", "/audio/transcriptions",
                             files={"file": (name, wav)},
                             data={"model": self.s.stt_model, "language": "en"})
        return (r.json().get("text") or "").strip()

    # --- voice out ---------------------------------------------------------------------------
    async def speak(self, text: str) -> bytes:
        """mp3 bytes. ElevenLabs first, Vultr TTS second, b"" if neither works (text only)."""
        if not text:
            return b""
        if self.s.eleven_key and self.s.eleven_voice:
            try:
                t0 = time.monotonic()
                r = await self.http.post(
                    f"{ELEVEN_BASE}/text-to-speech/{self.s.eleven_voice}",
                    params={"output_format": "mp3_44100_128", "optimize_streaming_latency": 3},
                    headers={"xi-api-key": self.s.eleven_key, "accept": "audio/mpeg"},
                    json={"text": text, "model_id": self.s.eleven_model})
                self.last_latency["tts"] = time.monotonic() - t0
                r.raise_for_status()
                return r.content
            except httpx.HTTPError as e:
                log.warning("ElevenLabs failed (%s); trying Vultr TTS", e)
        try:
            r = await self._post("tts", "/audio/speech", json={
                "model": self.s.vultr_tts_model, "input": text, "voice": "default",
                "response_format": "mp3"})
            return r.content
        except (httpx.HTTPError, MissingKey) as e:
            log.warning("no TTS available (%s); text only", e)
            return b""


async def to_wav16k(audio: bytes) -> bytes:
    """Phone audio (webm/opus, iOS mp4/AAC) -> 16 kHz mono wav via ffmpeg. No ffmpeg: as-is."""
    if audio[:4] == b"RIFF" or not shutil.which("ffmpeg"):
        return audio
    p = await asyncio.create_subprocess_exec(
        "ffmpeg", "-loglevel", "error", "-i", "pipe:0", "-ac", "1", "-ar", "16000",
        "-f", "wav", "pipe:1", stdin=asyncio.subprocess.PIPE, stdout=asyncio.subprocess.PIPE)
    out, _ = await p.communicate(audio)
    return out if p.returncode == 0 and out else audio
