"""Visual memory: pose-tagged frames -> caption + embedding -> SQLite; search by text.

    mem = Memory(data_dir, ai)
    await mem.ingest(jpeg, {"n", "e", "alt", "yaw", "t"})
    hits = await mem.search("my keys")   # [{id, caption, n, e, alt, yaw, t, jpeg_url, score}]

Captions come from the vision model (station.vision "caption"), embeddings from
EMBED_MODEL. Vectors live in SQLite as float32 blobs and are searched with numpy: a hackathon
has hundreds of frames, not millions, so no vector DB. Score = cosine similarity, plus a bonus
when a query word appears in the caption (embeddings are fuzzy about small objects like keys),
plus a small recency bonus so "where did I leave X" answers with the latest sighting.
"""
from __future__ import annotations

import asyncio
import re
import sqlite3
import time
from pathlib import Path

import numpy as np

from station import vision

SCHEMA = """CREATE TABLE IF NOT EXISTS frames (
  id INTEGER PRIMARY KEY, t REAL, n REAL, e REAL, alt REAL, yaw REAL,
  caption TEXT, kind TEXT DEFAULT 'auto', emb BLOB)"""
STOP = {"the", "a", "an", "my", "i", "did", "where", "is", "are", "leave", "left", "put", "of",
        "to", "in", "on", "it", "have", "you", "seen", "see"}


def _words(s: str) -> set[str]:
    return {w.rstrip("s") for w in re.findall(r"[a-z]+", s.lower())} - STOP


def _row(r) -> dict:
    return {"id": r[0], "t": r[1], "n": r[2], "e": r[3], "alt": r[4], "yaw": r[5],
            "caption": r[6], "kind": r[7], "jpeg_url": f"/memory/img/{r[0]}.jpg"}


class Memory:
    def __init__(self, data_dir: Path, ai, min_score: float = 0.25):
        self.dir = Path(data_dir)
        (self.dir / "frames").mkdir(parents=True, exist_ok=True)
        self.db = sqlite3.connect(self.dir / "memory.db", check_same_thread=False)
        self.db.execute(SCHEMA)
        self.ai = ai
        self.min_score = min_score
        self._lock = asyncio.Lock()

    def count(self) -> int:
        return self.db.execute("SELECT COUNT(*) FROM frames").fetchone()[0]

    def jpeg_path(self, fid: int) -> Path:
        return self.dir / "frames" / f"{int(fid)}.jpg"

    async def ingest(self, jpeg: bytes, pose: dict, caption: str = "", kind: str = "auto") -> dict:
        if not caption:
            caption = await vision.ask(self.ai, "caption", jpeg)
        try:
            emb = np.asarray((await self.ai.embed([caption]))[0], np.float32)
            emb = (emb / (np.linalg.norm(emb) or 1.0)).tobytes()
        except Exception:  # no key / model down: keep the photo, just not searchable
            if kind != "photo":
                raise
            emb = None
        async with self._lock:
            cur = self.db.execute(
                "INSERT INTO frames (t, n, e, alt, yaw, caption, kind, emb) VALUES (?,?,?,?,?,?,?,?)",
                (float(pose.get("t") or time.time()), float(pose["n"]), float(pose["e"]),
                 float(pose.get("alt", 0)), float(pose.get("yaw", 0)), caption, kind,
                 emb))
            self.db.commit()
            fid = cur.lastrowid
        self.jpeg_path(fid).write_bytes(jpeg)
        return {"id": fid, "caption": caption, "jpeg_url": f"/memory/img/{fid}.jpg"}

    async def search(self, q: str, k: int = 5) -> list[dict]:
        rows = self.db.execute("SELECT id, t, n, e, alt, yaw, caption, kind, emb FROM frames"
                               " WHERE emb IS NOT NULL").fetchall()
        if not rows:
            return []
        qv = np.asarray((await self.ai.embed([q]))[0], np.float32)
        qv /= np.linalg.norm(qv) or 1.0
        sims = np.stack([np.frombuffer(r[8], np.float32) for r in rows]) @ qv
        qw = _words(q)
        t_max = max(r[1] for r in rows)
        out = []
        for r, s in zip(rows, sims):
            bonus = 0.3 if qw and qw & _words(r[6]) else 0.0
            recency = 0.05 * (1.0 - min(1.0, (t_max - r[1]) / 3600))
            score = float(s) + bonus + recency
            if score >= self.min_score:
                out.append({**_row(r), "score": round(score, 3)})
        out.sort(key=lambda d: -d["score"])
        out = out[:20]
        if len(out) > 1 and hasattr(self.ai, "rerank"):
            try:  # cross-encoder over the captions; keyword/recency bonus kept as a tiebreak
                rr = await self.ai.rerank(q, [h["caption"] for h in out])
                for h, s in zip(out, rr):
                    h["score"] = round(s + 0.1 * (h["score"] - out[-1]["score"]), 3)
                out.sort(key=lambda d: -d["score"])
            except Exception:
                pass  # no reranker on this key: first-stage order stands
        return out[:k]

    def clear(self) -> int:
        """Forget everything (e.g. sim rehearsal frames before the real demo). Returns count."""
        n = self.count()
        self.db.execute("DELETE FROM frames")
        self.db.commit()
        for p in (self.dir / "frames").glob("*.jpg"):
            p.unlink(missing_ok=True)
        return n

    def recent(self, kind: str | None = None, limit: int = 20) -> list[dict]:
        sql, args = "SELECT id, t, n, e, alt, yaw, caption, kind FROM frames", ()
        if kind:
            sql, args = sql + " WHERE kind = ?", (kind,)
        rows = self.db.execute(sql + " ORDER BY id DESC LIMIT ?", (*args, limit)).fetchall()
        return [_row(r) for r in rows]
