# Devpost draft: the cloud half (voice, brain, perception, memory, dashboard)

Paste into the Devpost sections; the drone lead writes the QNX / on-board half.

## What it does (cloud half)

Talk to the drone in plain English, from your own phone. "Fly to the door and take a photo,
then come back." "What's written on the whiteboard?" "Where did I leave my keys? Take me
there." Jarvis, the drone's sarcastic voice, answers aloud, flies the mission, and refuses
anything dangerous with attitude. Anyone in the room can open the dashboard, watch the live
camera and telemetry, and hit STOP.

## How we built it

- **Vultr Serverless Inference does all the thinking.** Speech-to-text
  (`whisper-large-v3-turbo`), the brain (`glm-5.3-flash` with tool calling, about 0.7 s per
  command, `qwen3.8-flash-next` as a backup), vision (`qwen3.8-27b`) for finding objects and
  reading or solving what the camera sees, and `qwen3-embedding-4b` +
  `bge-reranker-v2-m3` for visual memory. Voice out is ElevenLabs.
- **A Vultr VM** runs the web app (FastAPI behind Caddy with HTTPS): the phone dashboard,
  push-to-talk, the visual memory store, and a websocket relay to the laptop next to the
  drone. The laptop dials out to Vultr, so it works behind any venue NAT.
- **The brain speaks in tools, not in motor commands.** The LLM picks from eight tools
  (`takeoff`, `move`, `mission`, `look`, `recall`, ...). A planner on the laptop turns them into
  small primitives the drone's safety gate can check one at a time. The LLM is never trusted:
  if it ever asks for something unsafe, the gate on the drone rejects it and Jarvis tells you
  why.
- **Finding "the door" with no object detector.** Vultr doesn't host one, so we ask the
  vision model for a bounding box, turn the box centre into a bearing with the camera
  intrinsics, and estimate range from the object's typical size or from where it meets the
  floor. Range from one frame is rough (±30 % or worse), so the drone flies half way, looks
  again and corrects: in simulation that cut the stopping error from 0.89 m to 0.34 m.
  Shaky sightings need a second look before the drone moves.
- **Visual memory.** While flying, frames tagged with the drone's position are captioned by
  the vision model, embedded and stored. "Where are my keys?" searches the captions (embedding
  similarity + keyword match + recency, then a reranker), answers with where and how long ago,
  and can fly you back to the spot.
- **Built to fail safely.** No key or LLM down → a rule-based parser still flies the demo
  commands. Cloud gone for 30 s while hovering → the laptop lands the drone. Laptop gone → the
  drone's own watchdog lands it. A stale position from the video process → no target is
  placed from it.

## Challenges

- No hosted open-vocabulary detector: we made a VLM give us boxes and did the geometry
  ourselves, then closed the loop with re-looks during the approach.
- Phones only allow the microphone over HTTPS, and iPhones record AAC: we serve over Caddy
  with Let's Encrypt and convert everything to 16 kHz mono WAV before Whisper.
- Keeping voice-to-motion under 3 s: hosted Whisper, a fast tool-calling model, and speaking
  the reply while the drone starts moving.

## What we learned

LLMs are good at intent and bad at metres. Give them a small tool vocabulary, keep geometry
and safety in code you can test, and assume every answer can be wrong.
