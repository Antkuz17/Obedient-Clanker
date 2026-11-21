# Vultr demo runbook (cloud lead)

The QNX demo is the drone lead's ([qnx.md](qnx.md)). This one shows "Vultr thinks and
remembers". About 90 seconds. Rehearse it 3× on the sim (`python -m station.dev`) before doing it
on the drone.

## Before judges arrive (10 min)

1. `python -m station.check_ai`: everything PASS (or understood). Note the voice latency line.
2. `curl https://<domain>/health` → `"station": true`, `"keys": {"vultr": true, "elevenlabs": true}`.
3. Laptop: `python -m station --edge <pi ip>`; the Nest Mini is the default speaker (or the laptop's).
4. Open the dashboard on **your phone** and on the laptop. Hold-to-talk once ("what do you see?")
   so the phone has granted mic permission. Anyone with the dashboard can fly it (there is no
   pilot PIN), so keep the URL to people you trust.
5. Walk the drone (or carry it, props off) round the room once so visual memory has frames:
   put the keys somewhere visible first. `/memory/recent` should list a dozen or more frames.
6. Have a QR code of `https://<domain>` ready so judges can open it on their own phones.

## The script

| # | Say / do | What they see | Track point |
|---|---|---|---|
| 1 | Hand a judge the QR code. "This is the drone's dashboard, on Vultr. Anyone can watch, fly it and hit STOP." | Live video, telemetry, flight log on their phone | Vultr deployment, multi-user |
| 2 | Hold to talk: **"Take off."** | Jarvis answers aloud, drone climbs to 1 m | Whisper + LLM on Vultr Serverless Inference |
| 3 | **"Fly to the door and take a photo, then come back."** | Turns until it finds the door, flies to 1 m short, looks again on the way, photo appears on the phone, flies home | VLM localization, planner |
| 4 | Point at the whiteboard: **"What's written there?"** (or "solve that") | Reads / solves it aloud | VLM tasks |
| 5 | **"Where did I leave my keys?"** | "I saw keys about 2 metres south-west of the takeoff spot, 3 minutes ago." Then **"Take me there."** | Visual memory: captions + embeddings + rerank on Vultr |
| 6 | **"Fly up ten metres."** / **"Crash into the wall."** | Sassy refusal; if the LLM ever lets one through, the edge's safety gate rejects it and Jarvis reads out why | "AI suggests, QNX decides" |
| 7 | Judge presses **STOP** on their phone mid-move | Drone holds | Safety from any viewer |
| 8 | "If Vultr goes down, the drone doesn't care: the laptop lands it after 30 s, and the drone's own watchdog lands it if the laptop dies." | (optional: kill the cloud service) | Failure design |

## If something breaks

| Problem | Do this |
|---|---|
| Voice is slow or Whisper fails | Type into the dashboard's command box: same brain, same flow |
| LLM down / key out of credit | The rule-based fallback still handles "take off", "fly to the door…", "where are my keys" |
| Vision can't find the door | Say "turn right 45 degrees" until it's in view, then retry; or demo on the sim screen |
| Cloud unreachable | `python -m station --edge <pi> --local --mic`: brain, speech and voice run on the laptop |
| Drone grounded | `python -m station.dev` and run the whole script on the sim on the big screen |
