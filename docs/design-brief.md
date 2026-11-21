# Design brief: Hope Drone live dashboard

Paste this into Claude Design. Goal: a dashboard that looks like a real, premium product
(think Apple, Linear, DJI Fly, Anduril Lattice), not a hackathon page made of boxes.

## The product in one paragraph

Hope is a small indoor drone you control by talking to it. You hold a button, say "fly to the
door and take a photo", and an AI ("Jarvis", dry and a bit sarcastic) turns that into a plan.
Before the drone moves, every step goes through a **safety gate running on QNX** (a real-time
OS on the drone's computer), which can refuse it ("above the 1.5 m ceiling", "battery too
low"). Our pitch: **"The AI suggests, QNX decides."** Built at Hack Western 13 for the
*Best Use of QNX* and *Best Use of Vultr* prizes.

## Who looks at it, and where

- **Judges at a demo table**, on their own phones (most important) or on our laptop screen.
  They watch for 2–3 minutes and must understand the idea in 10 seconds.
- **The operator** (us), who flies it. Needs STOP within reach at all times.
- Dark room or bright hall, often on a phone held at arm's length.

## What the screen must make obvious (in priority order)

1. **What the drone sees, live.** The forward camera is the hero. 640×480 JPEG frames at ~10 fps.
2. **The conversation:** what you said → the plan → what Jarvis replied → **did QNX approve
   or block it, and why.** The blocked case is the money shot of the demo; design it to feel
   dramatic but clean.
3. **STOP.** Always visible, always works, huge, unmistakable. LAND next to it.
4. **Hold to talk.** The main action. Show listening (live mic level), thinking, and the
   transcript of what it heard.
5. **Where the drone is:** state in words ("Hovering", "On the ground", "Landing") and height
   in metres. That's it for always-on telemetry.
6. **Problems, only when they happen:** link lost, pilot grabbed the remote (manual
   override), failsafe, battery low/critical, floor tracking too weak to take off, QNX
   blocked a command. Nothing should be shown "just because we have the number".

Secondary, behind a tab or sheet:
- **QNX safety proof:** safety loop timing (p99 and max lateness vs a 2 ms budget, a small
  2-minute line chart), person-detected-to-stop latency in ms, a process list with restart
  counts (we kill processes live and they come back), and a jitter benchmark table
  (normal vs real-time priority, with and without a CPU hog).
- **Memory:** "where did I leave my keys?" search over photos the drone took, with
  thumbnails, captions and where/when each was seen. Plus a photo gallery.

Explicitly **not wanted**: speed, climb rate, heading tapes, compasses, flight timers, GPS,
fake charts, "signal efficiency", tiles for every number.

## Hard facts to design around

- Indoor only. Height limit 0.3–1.5 m. Safety fence is a ±3 m box around the takeoff point.
  A tiny position view (drone inside the fence, its trail) is useful but must stay subtle.
- Battery: 3-cell LiPo. Critical below 10.6 V, low below 11.0 V (%, roughly, 10.5–12.6 V).
- Drone states: `DISCONNECTED, IDLE, TAKING_OFF, BUSY, HOVER, LANDING, FAILSAFE`.
- One person flies at a time ("You have control" / "Anna is flying"); everyone else can
  watch and press STOP. There is no pilot PIN.
- Replies are spoken (mp3) on the device too; there's a "voice replies here" toggle.
- Plan steps Jarvis can produce: `takeoff {alt}`, `land`, `stop`, `home`,
  `move {fwd, right, up, yaw_deg}` (metres/degrees, body frame), `mission {target, action,
  after}` (action: photo / read / solve / describe; after: stay / come_back / land),
  `look {action}`, `recall {query}`.
- Example exchange to mock: You: "fly up 10 metres" → plan: take off to 1 m, up 10 m →
  Jarvis: "On it." → **Blocked by QNX: move: above ceiling 1.5 m**.
  Happy path: "go to the red poster and read it" → mission(red poster, read, come_back) →
  "On my way." → passed → later Jarvis: "It says 'Exit'."

## Must work on

- Phone portrait (390×844) first-class: video on top, STOP / talk / LAND in thumb reach,
  conversation below or as a pull-up sheet.
- Laptop (1440×900): video large, conversation as a side panel.
- Dark only is fine.

## Technical constraints (so the design can be built as-is)

- Single file `cloud/static/index.html` (HTML + CSS + vanilla JS), served by FastAPI.
  No build step, no framework required (a CDN library is OK). Google Fonts OK.
- Live data arrives over a WebSocket `/ws/ui?client=<id>` as JSON messages:
  - `hello {you, station, holder, log[], telemetry}` on connect (log = history to replay)
  - `station {connected}`, `control {holder}`
  - `telemetry {state, armed, mode, n, e, alt, yaw, vn, ve, vd, battery_v, flow_quality(0-255), range_m, active_cmd}` at 5 Hz
  - `frame {jpeg (base64), pose}` ~10 fps
  - `log {t, who, text}`: `who` is a person/client id (what they said), `"plan"` (text = JSON
    array of steps), `"jarvis"` (reply), `"edge"` (drone events, e.g.
    `"safety_reject: move: above ceiling 1.5 m"`, failsafes), `"cloud"`, `"qnx"`
  - `heard {who, text}` (speech-to-text result), `audio {mp3 base64}`, `photo {url, caption}`
  - `qnx {...}` / `qnx_state {loop[], stops[], jitter[], procs[]}` for the QNX tab
- The page sends: `{type:"op", op:"stop"}`, `{type:"op", op:"land"}`,
  `{type:"text", text}`. Voice: `POST /voice` multipart (`audio`, `client`).
  Memory: `POST /memory/search {q}` → `[{caption, n, e, alt, yaw, t, jpeg_url}]`.
- The current file already wires all of this; a redesign can keep its JS and replace the
  markup and CSS.

## Feel

Calm, confident, cinematic. Typography and spacing over boxes and borders. One accent colour.
Motion only where it means something (listening, thinking, a block landing). It should look
like the drone is the product and the UI gets out of the way, until something goes wrong.
