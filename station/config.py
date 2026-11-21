"""Station and cloud settings, read from the environment and a git-ignored `.env`.

Keys we don't have yet (see docs/TODO.md "CRITICAL: keys"): everything still imports and the
tests run with mocked HTTP; calls that need a missing key raise MissingKey with the variable
name, and the brain falls back to its rule parser so the drone can still be flown by voice text.
"""
from __future__ import annotations

import os
from dataclasses import dataclass, field
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent


class MissingKey(RuntimeError):
    def __init__(self, var: str):
        super().__init__(f"{var} is not set (put it in .env; see .env.example)")
        self.var = var


def load_dotenv(path: Path = ROOT / ".env") -> None:
    """Tiny .env reader (KEY=value lines). Real environment variables win."""
    if not path.exists():
        return
    for line in path.read_text().splitlines():
        line = line.strip()
        if not line or line.startswith("#") or "=" not in line:
            continue
        k, v = line.split("=", 1)
        os.environ.setdefault(k.strip(), v.strip().strip('"').strip("'"))


def _env(name: str, default: str = "") -> str:
    return os.environ.get(name) or default  # blank lines in .env mean "use the default"


@dataclass
class Settings:
    # Vultr Serverless Inference (OpenAI-compatible)
    vultr_key: str = field(default_factory=lambda: _env("VULTR_INFERENCE_KEY"))
    vultr_base: str = field(default_factory=lambda: _env(
        "VULTR_INFERENCE_BASE", "https://api.vultrinference.com/v1"))
    brain_model: str = field(default_factory=lambda: _env("BRAIN_MODEL", "glm-5.3-flash"))
    brain_backup_model: str = field(default_factory=lambda: _env(
        "BRAIN_BACKUP_MODEL", "qwen3.8-flash-next"))
    vision_model: str = field(default_factory=lambda: _env("VISION_MODEL", "qwen3.8-27b"))
    stt_model: str = field(default_factory=lambda: _env("STT_MODEL", "whisper-large-v3-turbo"))
    # streaming speech-to-text while the talk button is held, e.g. voxtral-mini-4b-realtime
    stt_stream_model: str = field(default_factory=lambda: _env(
        "STT_STREAM_MODEL", ""))  # off: measured slower, see STATUS
    embed_model: str = field(default_factory=lambda: _env("EMBED_MODEL", "qwen3-embedding-4b"))
    rerank_model: str = field(default_factory=lambda: _env("RERANK_MODEL", "bge-reranker-v2-m3"))
    vultr_tts_model: str = field(default_factory=lambda: _env("VULTR_TTS_MODEL", "chatterbox"))
    # ElevenLabs (voice out). Empty key -> Vultr TTS -> text only.
    eleven_key: str = field(default_factory=lambda: _env("ELEVENLABS_API_KEY"))
    eleven_voice: str = field(default_factory=lambda: _env("ELEVENLABS_VOICE_ID"))
    eleven_model: str = field(default_factory=lambda: _env("ELEVENLABS_MODEL", "eleven_flash_v2_5"))
    # Links
    edge_host: str = field(default_factory=lambda: _env("EDGE_HOST", "127.0.0.1"))
    edge_port: int = field(default_factory=lambda: int(_env("EDGE_PORT", "7000")))
    video_port: int = field(default_factory=lambda: int(_env("VIDEO_PORT", "8080")))
    cloud_url: str = field(default_factory=lambda: _env("CLOUD_URL", ""))  # wss://host
    station_token: str = field(default_factory=lambda: _env("STATION_TOKEN", "dev-token"))
    # Cloud only: the pilot PIN. Without it a dashboard viewer can watch and press STOP but
    # can't make the drone take off, move or land (review3 #4). Unset: only localhost may fly.
    control_pin: str = field(default_factory=lambda: _env("CONTROL_PIN"))
    data_dir: Path = field(default_factory=lambda: Path(_env("DATA_DIR", str(ROOT / "data"))))

    def need(self, attr: str, var: str) -> str:
        v = getattr(self, attr)
        if not v:
            raise MissingKey(var)
        return v


def settings() -> Settings:
    load_dotenv()
    return Settings()
