# Status

Last updated: 2026-11-21. Update this at the end of every session.

**2026-11-21: pilot PIN removed.** `CONTROL_PIN`, the PIN check and lockout (`cloud/app.py`), the
dashboard PIN dialog, and the `deploy.sh` PIN generation are gone. Any dashboard viewer (local or
through the proxy) can now type, talk, LAND and STOP. The review-fix rows below that mention the
PIN are history. The dev station token is still localhost-only.

## Done: station + cloud (Person A, 2026-11-21; includes B's old localization / VLM / memory)

All written as if the keys exist; **no key has been used yet** (see TODO "CRITICAL"). Verified with
mocked HTTP (`tests/test_station.py`, 27 tests) and a live no-key smoke run: sim edge + cloud +
station, typed "fly to the door and take a photo, then come back" → rule brain → planner flew
to 1 m short of the door, took the photo, came home.

| Piece | Files | Verified how |
|---|---|---|
| AI client: Vultr chat/tools, vision, embeddings, STT (ffmpeg → 16 kHz wav); ElevenLabs → Vultr TTS → text | `station/ai.py`, `station/config.py` | mocked HTTP; **never called with a real key** |
| Brain: Jarvis prompt, 8 tools, refusals, rule fallback with no key / LLM down. **Spoken "stop" / "land" (short utterances only) skip the LLM** (`is_urgent`); `/voice` logs speech-to-text and brain latency per command | `station/brain.py`, `cloud/app.py` | unit tests (LLM mocked) |
| Voice latency: **fast path** (`fast_parse`: whole utterance = simple clauses with explicit distances inside limits, e.g. "forward 1 m and turn left 90", skips the LLM) and optional **streamed STT** (`/ws/voice` → `voxtral-mini-4b-realtime`, off unless `STT_STREAM_MODEL` is set) | `station/brain.py`, `station/ai.py` `StreamingSTT`, `cloud/app.py`, `cloud/static/index.html` | **end to end on the live server with real speech, 2026-11-21 14:20**: upload path release→command 0.5–0.8 s for simple commands (fast path), 1.5 s via the LLM. Streaming measured *slower*: a steady ~1.4 s after release (the realtime model lags real-time audio), so it is off by default |
| Planner: actions → gated primitives, scan until found, approach 1 m short, 3 m legs, fence clamp, tasks, recall, stop pre-empts | `station/planner.py` | e2e vs the sim edge |
| `locate()`: VLM box → bearing/range → NED; sim ground-truth stub | `station/localize.py`, `station/localize_stub.py` | geometry recovers sim object positions to 0.15 m; **VLM box quality untested** |
| Closed-loop approach, second-look confirmation, "where I'm pointing", memory rerank | `station/planner.py`, `station/vision.py`, `cloud/memory.py`, `station/ai.py` | sim tests (biased locate: 0.89 → 0.34 m error); rerank mocked, **`/rerank` endpoint on Vultr unverified** |
| Vision tasks read / solve / describe / caption | `station/vision.py` | mocked |
| Visual memory: SQLite + numpy cosine + keyword + recency | `cloud/memory.py` | unit test with a fake embedder |
| Cloud: `/voice`, `/text`, `/ws/station` (token), `/ws/ui`, control holder, STOP/LAND for all, memory API, flight log | `cloud/app.py` | TestClient test + smoke run |
| Phone dashboard | `cloud/static/index.html` | served in the smoke run; **not opened on a phone yet** |
| Station: edge link, WSS uplink (telemetry 5 Hz, frames 4 fps, memory frames on movement), audio out, `--local` REPL / mic | `station/station.py`, `station/audio.py`, `station/__main__.py` | smoke run (cloud mode) |
| QNX card on the dashboard: loop jitter graph, jitter benchmark, supervisor restarts, stop latency (Pi → UDP :7010 → station → cloud; opt-in `edge --stats-udp`) | `cloud/app.py` (`QnxState`, `GET /qnx`), `cloud/static/index.html`, `station/station.py` (`qnx_listen`), `edge/server.py`, `tools/jitter.py --udp` | 3 tests + live run: `station.dev` + a supervisor (kill -9 → restart row) + `tools.jitter --udp` all reached `/qnx`; dashboard JS run against that state. **Not opened in a browser yet** |
| Deploy guide (VM, systemd, Caddy HTTPS) | `cloud/deploy.md`, `.env.example` | not deployed yet |
| Station link safety: edge reconnect (also when the Pi goes silent: no telemetry for 3 s), cancels the plan on a drop; **cloud lost 30 s while hovering idle → says so and lands** (review2 "nobody owns the cloud died") | `station/station.py` | tests: edge server killed and restarted; cloud outage lands the sim |
| Station honours `X-Pose-Age` from the separate video process: a pose older than 0.5 s can't place a target (mission refused, said aloud) and isn't stored in memory | `station/camera.py`, `station/planner.py`, `station/station.py` | test + live run vs `python -m edge.video --camera sim` (core killed → age 2 s) |
| Key checker `python -m station.check_ai`, VM deploy script `cloud/deploy.sh`, sim stack `python -m station.dev`, demo runbook, Devpost draft | `station/check_ai.py`, `cloud/deploy.sh`, `station/dev.py`, `docs/demo.md`, `docs/devpost-cloud.md` | checker vs mocked Vultr (test); deploy.sh syntax only (**no VM yet**); dev stack run + photo mission |
| Dashboard websocket path | `cloud/static/index.html` | JS syntax check + a Python client speaking the page's protocol: 5 Hz telemetry, ~2.5 fps frames, log, reply, LAND. **Not rendered in a browser yet** (Chrome extension wasn't connected) |

## Done (Person A)

| Piece | Files | Verified how |
|---|---|---|
| Protocol v1 contract | `shared/protocol.py` | unit tests |
| Async client (heartbeats, `run()` / `send()` / `wait_done()`) | `shared/client.py` | used by all e2e tests |
| Safety gate (alt window, fence, step, battery, flow quality, person ahead) | `edge/safety.py`, `edge/config.py` | unit tests |
| Executor: one task at a time, supersede, `stop` wins, landing beats `stop`, timeouts | `edge/executor.py` | e2e tests vs the sim |
| Server: TCP protocol, 10 Hz telemetry, 50 Hz safety loop, watchdog (hover 1 s, land 5 s), battery auto-land, perception UDP :7001, flow UDP :7002 → FC | `edge/server.py` | e2e tests vs the sim |
| Kinematic sim vehicle | `edge/sim.py` | e2e tests |
| Video server + sim camera (fake room: door, whiteboard, keys, chair, plant) | `edge/video.py` | manual: `/forward.jpg`, `X-Pose`, `/camera.json` |
| MAVLink vehicle (GUIDED, arm, takeoff, position targets, origin, telemetry, `OPTICAL_FLOW` forwarding, GCS heartbeat, FC pre-arm reasons in `done`) | `edge/mav.py` | **ArduPilot SITL** (Copter stable, `tools/sitl_flight.py`): takeoff, fwd/yaw/right moves, gate rejection, return home, watchdog hover → land → disarm all pass with altitude held within 0.15 m. `OPTICAL_FLOW` forwarding is **not** tested (SITL has no flow). Not run on the real FC yet |
| SITL harness | `tools/sitl/Dockerfile`, `tools/sitl_flight.py` | see above |
| **USB MAVLink transport** (`--mav usb`): pymavlink `StreamMav` over Anton's pyusb CDC-ACM code | `edge/mavlink_usb.py` | Full SITL flight passes through `StreamMav` (`sitl_flight rawtcp:127.0.0.1:5760`) + regression test. The `UsbCdc` libusb calls are copied from the verified probe but **not yet run inside the edge on the Pi** |
| **Camera grabber** (C, `libcamapi` → shared memory, seqlock, NV12/YUYV, strips stride padding, `-l` lists units) | `edge/camgrab/camgrab.c` | Compiled with `-Wall -Wextra` and run on the Mac against a stub camera library (`tests/camstub/`) that pads rows: Python sees exact pixels in order. **Not compiled on QNX yet**, and the stub copies the *real* header only as far as camgrab uses it (see below) |
| Shared-memory frame reader/writer + laptop stand-in | `edge/camera.py`, `tools/fake_camgrab.py` | unit tests; manual: sim video → fake_camgrab → shm → `edge --camera shm:NAME` → `/forward.jpg` |
| Fail-safe: forward moves refused when the person detector is silent (real drone only; `--no-person-check` to override) | `edge/safety.py`, `edge/server.py` | unit test |
| Entry point + `SCHED_FIFO` flag | `edge/__main__.py` | manual smoke test with `--sim` |
| Protocol REPL | `tools/cmd.py` | not run interactively yet |

**Review fixes (docs/review.md section A):** A1, A2, A3, A5, A6, A7, A8, A9 (relay + `--station-ip`),
A10, A11, A12, A13 are done; every one has a unit test or a SITL check. **Not done: A4**
(set the FC params on the real board: `FS_GCS_ENABLE=5`, `FS_GCS_TIMEOUT=2`,
`FS_EKF_ACTION=1`; SITL proves FS_GCS works).

SITL checks (`tools/sitl_flight.py`, all PASS, also through `tools/usb_relay.py`): takeoff,
moves, yaw, gate rejection, return to the takeoff point, watchdog hover → land, and **a hung
edge event loop makes ArduPilot's own GCS failsafe land the drone**. FC reboot under the relay:
the edge re-sends stream rates + origin and position comes back.

The test suite has 174 passing tests (`.venv/bin/python -m pytest -q`), plus `tools/sitl_flight.py`, which
must print ALL PASS.

Fixed after SITL: the stream transport handed pymavlink whole chunks, so messages queued up
and telemetry lagged further and further behind (the drone armed but never took off). It now
hands over only the bytes pymavlink asks for. Relative moves now build on the last *commanded* target, not the measured
position (altitude used to sag about 0.1 m per command).

## Still open from the review (Person A)

- **A4** FC params on the board (above). Review C: flow-only SITL (`GPS1_TYPE=0`,
  `SIM_FLOW_ENABLE=1`, `FLOW_TYPE=10`); a `DISTANCE_SENSOR` input path for height; the video
  server as its own process (needs pose from the core, e.g. UDP on localhost).
- Review D: the watchdog/heartbeat loop in **C** at `SCHED_FIFO` with a jitter histogram; the
  supervisor.
- Kept shared memory rather than the suggested stdout pipe: the forward camera has two
  readers (video + TFLite), and a pipe serves one.

## Next for Person A, in order

1. ~~SITL run of `edge/mav.py`~~ done.
2. **Planner** (`station/planner.py`): `mission(where, what, after)` → primitive sequence using
   `shared/client.py`. Include scan-until-found with a stub `locate()`
   (`station/localize_stub.py`, reading `edge.video.SIM_WORLD`).
3. **LLM** (`station/brain.py`): tool-call schema for `mission()`, personality, refusals.
   Provider: Vultr Serverless Inference (OpenAI-compatible); keep it swappable.
4. **Voice loop**: mic → VAD → Whisper → brain → TTS → Nest Mini.
5. **Vultr cloud + dashboard** (`cloud/`).
6. **QNX port** (Pi facts in qnx.md, "Verified on our Pi"): copy `edge/` and `shared/` to the Pi
   and run with `--rt-prio`.
   - **FC link: USB via the relay.** On the Pi, props **off**:
     `sudo env PYTHONPATH=$HOME/.local/lib/python3.14/site-packages python3 -m tools.usb_relay`
     then (as qnxuser) `python3 -m edge --mav udpin:127.0.0.1:14555 --station-ip <laptop>
     --no-video`; connect `python -m tools.cmd <pi ip>` from the laptop, then check telemetry,
     arm/disarm, the Wi-Fi pull (hover → land) and `slay` of the core (FC lands).
   - **Camera grabber: written, needs building on the Pi.** `apk add qnx-sf-base-dev`, then
     `cc -O2 -o camgrab edge/camgrab/camgrab.c -lcamapi`, `./camgrab -l` to list units, and
     `./camgrab -u 5 -n camgrab_fwd`. Check that `python3 -c "from edge.camera import
     ShmFrameReader as R; print(R('camgrab_fwd').wait_new().gray().shape)"` prints
     `(480, 640)`. Then the edge video uses it by default on `--mav`. Two cameras: run two
     camgrabs (`-u 5 -n camgrab_fwd`, `-u <other unit> -n camgrab_down`); whether QNX gives the
     second C270 its own unit is still open (flow.md Q3).
   - **Supervisor: done** (`tools/supervisor.py`, no HAM). `python3 -m tools.supervisor
     tools/supervisor_pi.json` starts usb_relay (root), edge and camgrab (as qnxuser, SCHED_FIFO
     prio from the config), restarts any that die, and logs JSON events with `spawn_ms`
     (death -> respawn) and `ready_ms` (death -> serving again: new shm frame / TCP accepting).
     Crash loops back off to 5 s. `--events-udp host:port` streams events (dashboard: not wired
     yet). Laptop demo (`tools/supervisor_laptop.json`): `kill -9` camgrab -> respawn 3.5 ms,
     new frame 208 ms; kill edge -> 3.9 ms, serving 119 ms. Add flow/vision entries when they
     exist. On the Pi: replace `LAPTOP_IP`, run as root, then `slay camgrab` for the demo.
   - **Jitter benchmark: done** (`tools/jitter.py`). On the Pi, as root:
     `python3 -m tools.jitter --compare --out jitter.json` runs idle/hog x normal/FIFO 50 and
     prints p50/p99/max lateness + a histogram; the JSON (every sample) feeds the dashboard graph.
     macOS baseline (no FIFO): p50 ~2.5-3.4 ms, p99 5-12 ms, max up to 34 ms under the hog.
     Not yet: the edge's own `EdgeServer.loop_stats` streamed live.
   - **Video as its own process: done** (`python -m edge.video --camera shm:camgrab_fwd`, core
     `--video-proc 127.0.0.1:7003` streams its pose as JSON each safety tick). Same endpoints and
     `X-Pose`; adds `X-Pose-Age` (large = core gone, don't trust the pose for `locate()`). In the
     Pi supervisor config at FIFO 10, so under load it's the one that suffers, not the core.
   - **C heartbeat watchdog: done** (`edge/hbwd/hbwd.c`, review D). The core pets it every safety
     tick over UDP :14560 (`edge --hbwd 127.0.0.1:14560`); hbwd (50 Hz, SCHED_FIFO, pinned)
     sends the FC's GCS heartbeat through the relay's write-only aux inlet
     (`usb_relay --aux 127.0.0.1:14556`). Pets missing 250 ms → heartbeat stops, and if the last
     pet said armed in GUIDED/LAND it sends LAND (3×, 100 ms apart); a pilot mode is left alone
     (FC failsafe is the backstop). Prints JSON events and its own lateness histogram every
     second (the jitter graph of the real safety loop). Verified: `tests/test_hbwd.py` (pymavlink
     decodes its frames) and `tools/sitl_hbwd.py` against ArduPilot with the on-drone process
     layout: edge frozen by SIGSTOP while hovering → FC in LAND **273 ms** later with the FC's
     own failsafe off, landed + disarmed, heartbeats resume when the core comes back.

## Limits removed for flying outdoors (2026-11-21, cloud lead's call; **drone lead: please review**)

`edge/config.py`: `alt_max`, `fence_half`, `max_step`, `max_takeoff_alt` are now `None` (no
limit); the gate and planner skip a limit that is `None`, so setting a number restores it. The
0.3 m floor stays (a move can't fly into the ground; takeoff and land are unaffected). The brain
prompt no longer says "indoors, 6 x 6 m box, 1.5 m"; the voice fast path has no distance or
height cap. Limit tests now opt into the old values; new tests check there are none by default.
Not changed: the dashboard's fence mini-map (`index.html`, being redesigned) still draws ±3 m.

Then, same day: **every limit we set is gone.**
- **2 m height cap** (it came from flow + a 2 m ultrasonic with the GPS off): `edge --gps` (no
  fake EKF origin, no optical-flow requirement; on in `supervisor_pi.json`) +
  `tools/fc_profile.py --outdoor --apply --reboot` (ArduPilot's GPS defaults: `GPS1_TYPE=1`,
  `EK3_SRC1_POSXY/VELXY/VELZ=3`, baro height, compass yaw). **Not applied to the FC yet.**
- **Speed:** the edge now writes `WP_SPD=20`, `WP_SPD_UP=10` (ArduPilot's maxima) instead of
  0.5 / 0.3 m/s; descent `WP_SPD_DN=1.5` (ArduPilot's default; faster risks vortex ring state).
  Real top speed is set by the airframe and the FC's `ANGLE_MAX` (default, not ours). No cap
  on the scan op's turn rate.
- Timeouts (edge + planner) now come from `edge.config.flight_time` with assumed speeds
  (≤ 2 m/s across, ≤ 1 m/s up/down), so faster settings never shorten them; the sim has its
  own speeds (`SimVehicle.SPEED_*`).
- Dashboard: the ±3 m fence map is now a track around the takeoff point that zooms to fit;
  the "can't see the floor" warning is gone.
- **Kept on purpose (they react to danger, they don't limit where it flies):** stop/refuse
  forward moves near a person (1 m) or an echo (0.7 m); refuse forward moves while the person
  detector is down (real drone); battery refuse 10.6 V / land 10.2 V; station link lost →
  hover 1 s, land 5 s; cloud lost 30 s while idle in the air → land; no EKF position → no
  takeoff / land; pilot's switch always wins; the 0.3 m floor for moves. Each is one line in
  `edge/config.py` or `station/station.py` if the team wants it different.
- Not re-run: SITL (`tools/sitl_*`), which now fly at the new speeds.

## On-board person detector + forward obstacle sensor (2026-11-21 afternoon, cloud lead with Claude; **drone lead: please review before flying**)

Suite: 185 passed (11 new in `tests/test_vision.py`). **Not run with the real model or on the
Pi yet**: no TFLite runtime on the laptop (installing one was blocked here); the Pi has
`tflite-runtime` 2.21.

| Piece | Files | Verified how |
|---|---|---|
| Person detector: camgrab_fwd → COCO SSD-MobileNet v1 (quantised, class 0 = person) → nearest-person distance → UDP :7001 with frame time + inference ms, up to 10 Hz, 2 threads. Silent when frames stop (the gate then refuses forward moves). Exit 2 if the model/runtime is missing (supervisor retries) | `edge/vision/`, `tools/fetch_person_model.py` (pinned SHA-256), `supervisor_pi.json` (prio 41) | fake interpreter: output mapping by name / shape, score + class filter, uint8 / float input, the loop; start-up failure exit code |
| Distance: full body → 1.7 m / box height; cut off top or bottom (anyone within ~2.7 m on a 45° C270) → 0.4 m shoulders / box width, erring closer. ±30 % | `edge/vision/__init__.py` | unit tests |
| Gate: forward moves must **end** ≥ `person_close_m` (1.0) from a person and ≥ `obstacle_close_m` (0.7) from a forward echo; a running forward move stops when either gets too close. `--require-obstacle-sensor` makes a silent forward sensor block forward moves | `edge/safety.py`, `edge/executor.py`, `edge/server.py` (`fwd_m` on :7002, stale after 0.5 s), `edge/config.py`, `edge/__main__.py` | unit + sim tests |
| Forward HC-SR04 on the same Arduino (D11/D12), pinged alternately with the down one, each ~20 Hz | `tools/hcsr04/hcsr04.ino`, `tools/rangefinder.py` | line parsing test; **sketch not compiled or uploaded** |

Known limits: palm stop is not done (`palm` always false). An unwired forward sensor reads "clear". A
mission to a *person* now has its approach end right at the 1 m person limit, so it may be
refused; that is the gate doing its job.

## Fourth review fixes (2026-11-21 afternoon, cloud lead with Claude; **drone lead: please review the `edge/` rows before flying**)

Suite: 174 passed (11 new tests). Not run on SITL or the real FC.

| # | Problem | Fix | Checked how |
|---|---|---|---|
| 1 | **Pilot PIN brute-forceable**: 4 digits, no attempt limit (9,000 POSTs to `/text`) | `cloud/app.py`: 5 wrong PINs per address, then locked 30 s, doubling (max 32 min); empty PIN never counts; `deploy.sh` makes an **8-digit** PIN and replaces one shorter than 6. Non-ASCII PIN no longer 500s | test |
| 2 | No-PIN commands were spoken (ElevenLabs credits) and broadcast to every viewer and the laptop speaker | refusal goes to the caller only; `/memory/search` limited to 10/min per address (still open to viewers) | tests |
| 3 | One slow phone stalled the station relay and every other viewer (sends awaited in turn) | `cloud/app.py` `Viewer`: own sender task per viewer; frames + telemetry latest-wins, other messages queued, dropped at 500 behind. Station `memory_search` no longer blocks the relay | test |
| 4 | Two tabs in one browser shared a client id; the first froze | `Hub.viewer_id`: unique ids | test |
| 5 | QNX stats UDP :7010 took datagrams from anyone on the Wi-Fi | `station/station.py`: only EDGE_HOST and localhost | test |
| 6 | Memory frames (caption + embedding, paid) every 15 s while sitting on the ground | only while airborne | — |
| 7 | Planner set its fence centre even when the takeoff was refused; STOP/LAND tasks were unreferenced (GC could drop them) | `station/planner.py` | suite |
| 8 | `edge/executor.py`: a bad-flow streak from one move carried into the next (one blurry frame could abort it) | streak reset when no move runs | test |
| 9 | `edge/executor.py` + `server.py`: a failsafe LAND fired while disarmed latched and refused every takeoff until restart; the 1 s tick-failure landing fired only once per edge run | cleared when there is nothing to land; re-armed after a good tick | test |
| 10 | `edge/mav.py`: a failed arm was reported only after the full climb timeout (~11 s) | `takeoff_failed` flag; the executor stops waiting | test (sim stand-in) |
| 11 | `edge/server.py`: safety loop slept 20 ms *after* each tick (ran below 50 Hz, tick time counted as jitter) | absolute deadlines, no catch-up bursts | test: 5 ms tick still ~50 Hz |

Not changed: the real drone still refuses forward moves with no person detector (`edge/vision/` doesn't exist; `supervisor_pi.json` doesn't pass `--no-person-check`). Decide before the demo.

## Third review fixes (2026-11-21, cloud lead with Claude; **drone lead: please review before flying**)

A third fresh-eyes review found these; all are fixed, uncommitted at the time of writing. Suite:
132 passed. SITL (private containers): `sitl_flight`, `sitl_flowonly`, `sitl_hbwd` ALL PASS.

| # | Problem | Fix | Checked how |
|---|---|---|---|
| 1 | **Height came from the drifting baro zero at boot**, NAV_TAKEOFF climbs from the ground at arming. With +0.96 m of drift, takeoff "finished" on the ground, the drone then climbed to a reading of 1.75 m (above the 1.5 m ceiling) and the next `move_body fwd=0.5` flew it into the ground | `edge/mav.py`: the ground level is tracked while disarmed / ON_GROUND and frozen in flight; `alt` and position targets are relative to it | SITL with `SIM_BARO_DRIFT` ±0.03 m/s: edge height = FC height above the arming point, takeoff and moves hold 0.80 m both ways; unit test |
| 2 | **The fence moved to wherever the drone last took off** | `edge/executor.py`: home = first takeoff of this edge run, kept; `edge/safety.py`: takeoff refused outside the area ("restart the edge to reset"); planner clamps around the same point | sim test; gate test |
| 3 | **A door filling the frame read as 2.3-2.9 m away** at any closer range, and the drone flew towards it | `station/localize.py`: a cut-off box gives only an upper bound; floor range used when the bottom is visible; otherwise `upper_bound` and the planner just faces it | unit tests at 1.2/1.6/2.0 m |
| 4 | **Anyone with the dashboard URL could fly it**; the control lock trusted the browser's name. Behind Caddy everyone also looked like localhost (the `dev-token` check too) | `CONTROL_PIN` (cloud `.env`): needed for commands, voice and LAND; STOP stays open to all. No PIN: only this machine, never via the proxy. `deploy.sh` makes one up if unset and prints it. `/memory/reset` (PIN) | TestClient tests |
| 5 | Pi config ran flow on the forward camera | `supervisor_pi.json`: `camgrab_down` (unit 6, 320×240) → flow. **Needs the second C270/C920** and unit 6 set up as in qnx.md | config only |
| 6 | **One exception ended the 50 Hz safety loop** (watchdog gone, core still accepting commands) | `edge/server.py`: the loop catches and continues; hbwd is petted only after a *complete* tick (so a persistent failure lands via hbwd); without hbwd it lands after 1 s of failures | sim tests; `sitl_hbwd` (LAND 267 ms) |
| 7 | **Station/dashboard LAND overrode the pilot** in AltHold | `edge/executor.py`: land/stop/hover refused while the pilot has another mode (pilot always wins) | SITL: refused in ALT_HOLD, mode stayed; land works after hand-back |
| 8 | Flow and height reached the FC only via the Python core: a core crash = blind landing. Flight log writes could stall the loop | `edge/fcdirect.py`: flow and rangefinder send MAVLink 2 straight to the relay aux inlet (`--fc 127.0.0.1:14556`, on in `supervisor_pi.json`); the core only records quality. Flight log on a writer thread | unit tests decode the frames. **Not run against a real FC: check the FC sees `OPTICAL_FLOW` and `DISTANCE_SENSOR` with the core stopped** |
| 9 | No-key rule parser moved on stray words ("take off right now" slid right, "calm down" descended) | `station/brain.py`: a direction needs a distance or a movement verb; "back up a bit" = back 0.5 m | unit tests |
| — | HFOV 60° was wrong (spec is 55° *diagonal* ≈ 45° horizontal) | 45° in `edge/flow`, `edge/video.py`; scan step 40°. **Still measure it** | — |
| — | One blurry frame aborted a move and the plan | flow must be bad for 0.5 s | sim tests |
| — | Supervisor died with the SSH session | ignores SIGHUP; `--child-logs DIR` | test |

Still open from that review: no person/palm detector (forward moves stay refused on the real
drone, by design), no motor kill switch on the transmitter, no FC-side fence.

## Review follow-ups (from [review.md](review.md), 2026-11-21)

Before any real flight:
- [x] A1: `land`/`stop` abort an in-progress takeoff (an abort flag in the takeoff thread)
- [x] A2: LAND resent until the mode reads LAND; FAILED on timeout
- [x] A3 (SITL: hung loop → FC lands): FC heartbeat sent from `safety_tick`; thread exceptions → `connected=False`
- [ ] A4 (**board params + transmitter, by hand**): FC params `FS_GCS_ENABLE=5`, `FS_GCS_TIMEOUT=2`, `FS_EKF_ACTION=1`; pilot switch AltHold / Land / GUIDED
- [x] A5–A7 (A5 SITL reboot test): USB reconnect; EKF health + GUIDED check in the gate; airborne from armed + alt when `EXTENDED_SYS_STATE` is stale
- [x] A8: forward moves refused when the person detector is dead (`40f6f54`)
- [x] A9 (`tools/usb_relay.py`, `--station-ip`; SITL flight through it): root USB relay → UDP `127.0.0.1:14555`, core unprivileged, station-only connections
- [x] A10–A13: station heartbeat thread; one MAVLink writer; home = takeoff point; `pos_tolerance` 0.3
- [ ] Flow-only SITL test (no GPS, no rangefinder) before relying on baro height
- [ ] Decide models and the LLM command schema (PRD "Still to decide")

## Second review (2026-11-21)

[review2.md](review2.md): a fresh-eyes review of the design and the code. 13 findings, 8 reproduced
with scripts in `tools/review/`. **Fixed (A, 2026-11-21): #1-#8, #10-#13**; #14 fixed by B.
Verified: 45 tests; Anton's repros rerun unchanged (`sim_exp`, `sitl_exp`, `tools/review/hung_planner.py`
with the edge in its own process); `tools.sitl_flight` ALL PASS 8 of 9 runs.
- Speed limits: set on every connect as `WP_SPD*` (ArduPilot 4.7, m/s) or `WPNAV_SPEED*` (older,
  cm/s), read back; takeoff is refused until confirmed. A re-setup after a telemetry gap keeps them.
- **Open: intermittent SITL failure** (1 of 9): re-takeoff after the watchdog landing aborted with
  "position estimate lost". The failsafe now logs the EKF report age and flags; next failure tells
  us stale telemetry (our side) vs bad EKF flags (may be the same root as #9).
- **#9: not a problem in SITL.** `tools.sitl_flowonly` (flow + rangefinder, no GPS) ALL PASS with the
  edge's normal position check, after the origin retry and scaled move tolerance (both from
  Anton's flow-only run). Recheck on the real board.
- SITL tally tonight: `tools.sitl_flight` ALL PASS 14 of 15 runs; the one failure is the open item above.

## Open questions / known gaps

- **camgrab vs the real `camera_api.h`:** `rpi_cam.cpp` (QNX's workshop) confirms
  `camera_open`, frame types, `camera_set/get_vf_property`, `camera_start_viewfinder` and
  `framedesc.nv12.{width,height,uv_offset}` / `framedesc.ycbycr.{width,height}`. From memory
  and **unconfirmed**: `nv12.stride`, `nv12.uv_stride`, `ycbycr.stride` and
  `camera_get_supported_cameras()` (only used by `-l`). If the compile fails on those, check
  `/usr/include/camera/camera_api.h` on the Pi.

- Is `flow_rate_x/y` (vs `flow_x/y` pixels) what ArduPilot 4.7's MAV flow backend reads? The
  docs say rate. Confirm in SITL or in `AP_OpticalFlow_MAV.cpp`.
- SITL needs about 20 s after boot for the EKF ("Need Position Estimate"); the flight script
  retries takeoff. On the real drone the same error means no origin or no flow; see
  flight-setup.md.
- `MavVehicle` reports `flow_quality` from what the Pi last forwarded. With no flow process
  running, it's 0, so the gate refuses takeoff. That's intended on the real drone; for SITL
  without flow, lower `flow_quality_min` or feed fake flow.
- The FC speed limit comes from `MAV_CMD_DO_CHANGE_SPEED` at takeoff. Also set
  `WPNAV_SPEED=50`, `WPNAV_SPEED_UP=30`, `WPNAV_SPEED_DN=30` on the FC.
- Set `FS_GCS_ENABLE` on the FC so the FC itself lands if the edge process dies (the edge
  heartbeats as sysid 255).
- Sim camera HFOV is 60°; measure the real C270 and update `edge/video.py:HFOV_DEG`.
- Only one station can connect at a time; the newest connection wins. The cloud dashboard
  must go through the station (or be the station), not connect separately.
