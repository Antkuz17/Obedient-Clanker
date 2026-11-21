"""Forward-camera frames from the edge video server (`/forward.jpg` + `X-Pose`, `/camera.json`)."""
from __future__ import annotations

from dataclasses import dataclass

import httpx

from shared import protocol as P


@dataclass
class Pose:
    n: float
    e: float
    alt: float
    yaw: float
    t: float
    age: float = 0.0  # X-Pose-Age: s since the video process last heard from the core

    @classmethod
    def parse(cls, header: str, age: str | None = None) -> "Pose":
        n, e, alt, yaw, t = (float(x) for x in header.split(","))
        return cls(n, e, alt, yaw, t, float(age) if age else 0.0)

    def as_dict(self) -> dict:
        return {"n": self.n, "e": self.e, "alt": self.alt, "yaw": self.yaw, "t": self.t}


@dataclass
class Frame:
    jpeg: bytes
    pose: Pose


class EdgeCamera:
    def __init__(self, host: str, port: int = P.VIDEO_PORT,
                 transport: httpx.AsyncBaseTransport | None = None):
        self.base = f"http://{host}:{port}"
        self.http = httpx.AsyncClient(timeout=3.0, transport=transport)
        self._K: dict | None = None

    async def K(self) -> dict:
        if self._K is None:
            r = await self.http.get(f"{self.base}/camera.json")
            r.raise_for_status()
            self._K = r.json()
        return self._K

    async def frame(self) -> Frame:
        r = await self.http.get(f"{self.base}/forward.jpg")
        r.raise_for_status()
        return Frame(r.content, Pose.parse(r.headers["X-Pose"], r.headers.get("X-Pose-Age")))

    async def aclose(self) -> None:
        await self.http.aclose()
