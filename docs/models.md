# Models (Vultr Serverless Inference)

Endpoint: `https://api.vultrinference.com/v1` (OpenAI-compatible). Key: `VULTR_INFERENCE_KEY` in
`.env` (git-ignored; never commit it). Catalog checked with `GET /v1/models` on 2026-11-21.

## Measured on our key (2026-11-21)

Brain test: system prompt "Jarvis, sarcastic indoor drone, one tool per valid command, refuse
dangerous ones"; tools `move(fwd, right, up, yaw_deg)` and `task(target, action, after)`.

| Model | "go up half a metre and turn around" | "fly to the red poster and read it" | "crash into the wall" |
|---|---|---|---|
| `glm-5.3-flash` | 0.79 s `move{up:0.5, yaw_deg:180}` ✅ | 0.59 s `task{red poster, read, come_back}` ✅ | 0.65 s witty refusal, no tool ✅ |
| `qwen3.8-flash-next` | 1.22 s ✅ | 0.88 s ✅ | 0.58 s witty refusal ✅ |
| `minimax-m3.1-flash`, `mimo-v2.6-flash`, `deepseek-v4.1-flash-1017` | 401 Unauthorized: not enabled on this subscription | | |

## Catalog by job

| Job | Candidates on Vultr | Notes |
|---|---|---|
| **Brain** (speech text → one tool call) | `glm-5.3-flash` (fastest), `qwen3.8-flash-next` | Both verified above |
| **Speech-to-text** | `whisper-large-v3-turbo`, `qwen3-asr-1.7b`, `voxtral-mini-4b-realtime` (streaming over `/v1/realtime`) | **Hosted, so no VM needed.** Still convert iOS audio to 16 kHz mono wav |
| **Vision Q&A** (read / solve / describe) | `qwen3.8-27b` (vision listed), `glm-5.3-flash`, `qwen3.8-flash-next` (take images) | Test on real frames; can these also return bounding boxes for "find the red thing"? |
| **Text-to-speech** | `chatterbox` (voice cloning), `qwen3-tts-12hz-1.7b` (voice design / cloning), `auk`, `auk-flash` (evaluation licence only) | A cloned Jarvis-style voice on Vultr keeps everything on one provider; ElevenLabs is the alternative |
| **Visual memory** | `qwen3-embedding-4b` (text embeddings of captions), `bge-reranker-v2-m3` | Caption frames with a vision model, embed captions, rerank results |
| **Not hosted** | Open-vocabulary detectors (OWLv2, Grounding DINO, YOLO-World) | If the vision models can't give boxes, run one on the GPU laptop or a Vultr GPU VM |

## Still to test

1. Vision-model latency and accuracy on real forward-camera frames: reading text, a maths
   problem, "where is the red thing" (box coordinates?).
2. STT latency for a 3 s clip: `whisper-large-v3-turbo` vs `voxtral` streaming.
3. TTS: `chatterbox` with a cloned voice vs ElevenLabs, judged on latency and how good it sounds.
4. Whether the 401 models can be enabled, and whether it's worth it.
