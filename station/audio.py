"""Laptop audio: play replies on the default output (the Nest Mini over Bluetooth), and an
optional push-to-talk mic (fallback for the phone dashboard).

Playback is queued so replies never talk over each other. Players, first found wins:
ffplay, afplay (macOS), mpg123, then PowerShell's MediaPlayer on Windows.
"""
from __future__ import annotations

import asyncio
import io
import logging
import os
import shutil
import sys
import tempfile
import wave

log = logging.getLogger("audio")


def _player(path: str) -> list[str] | None:
    if shutil.which("ffplay"):
        return ["ffplay", "-nodisp", "-autoexit", "-loglevel", "quiet", path]
    if shutil.which("afplay"):
        return ["afplay", path]
    if shutil.which("mpg123"):
        return ["mpg123", "-q", path]
    if sys.platform == "win32":
        ps = ("Add-Type -AssemblyName presentationCore;"
              "$p=New-Object System.Windows.Media.MediaPlayer;"
              f"$p.Open([uri]'{path}');$p.Play();Start-Sleep -m 300;"
              "while(-not $p.NaturalDuration.HasTimeSpan){Start-Sleep -m 50};"
              "Start-Sleep -m ([int]$p.NaturalDuration.TimeSpan.TotalMilliseconds)")
        return ["powershell", "-NoProfile", "-Command", ps]
    return None


class Speaker:
    def __init__(self, enabled: bool = True):
        self.enabled = enabled
        self.q: asyncio.Queue[bytes] = asyncio.Queue()
        self._task: asyncio.Task | None = None

    def play(self, mp3: bytes) -> None:
        if not self.enabled or not mp3:
            return
        if self._task is None:
            self._task = asyncio.create_task(self._run())
        self.q.put_nowait(mp3)

    async def _run(self) -> None:
        while True:
            mp3 = await self.q.get()
            fd, path = tempfile.mkstemp(suffix=".mp3")
            os.write(fd, mp3)
            os.close(fd)
            cmd = _player(path)
            if not cmd:
                log.warning("no audio player found (install ffmpeg); replies are text only")
                self.enabled = False
                return
            try:
                p = await asyncio.create_subprocess_exec(*cmd)
                await p.wait()
            except OSError as e:
                log.warning("playback failed: %s", e)
            finally:
                try:
                    os.unlink(path)
                except OSError:
                    pass


def record_wav(seconds_max: float = 8.0, rate: int = 16000) -> bytes:
    """Blocking push-to-talk: records until Enter (or seconds_max). Needs `sounddevice`."""
    import numpy as np
    import sounddevice as sd  # optional: uv pip install sounddevice

    chunks: list = []
    stream = sd.InputStream(samplerate=rate, channels=1, dtype="int16",
                            callback=lambda d, *_: chunks.append(d.copy()))
    with stream:
        input("  recording... press Enter to send ")
    pcm = np.concatenate(chunks) if chunks else np.zeros((0, 1), np.int16)
    pcm = pcm[: int(seconds_max * rate)]
    buf = io.BytesIO()
    with wave.open(buf, "wb") as w:
        w.setnchannels(1)
        w.setsampwidth(2)
        w.setframerate(rate)
        w.writeframes(pcm.tobytes())
    return buf.getvalue()
