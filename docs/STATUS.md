# Status

Last updated: 2026-11-22. Update this at the end of every work session. Open work is in
[TODO.md](TODO.md); the detailed session-by-session history is in git (`git log -- docs/STATUS.md`).

## What works

**On the real drone** (outdoors, GPS sideways + rangefinder height, by voice from the dashboard,
safety pilot on the transmitter):

- Takeoff to a hover held by the rangefinder; landing on command.
- A full 360° turn, then hovering; "land" said mid-flight.
- Sideways moves of up to 2 m with a photo on the way, on a full pack. On a tired pack it
  sinks while tilting (about 80% throttle to hover) and the guard lands it.
- Gate refusals for height, battery and distance limits, read out by Jarvis.
- QNX side, measured on the drone: priority plan, C watchdog (worst wake-up under 1 ms),
  supervisor restarts, fault tests with the drone disarmed ([qnx.md](qnx.md)).

**In the simulator only** (`python -m station.dev`, 312 tests):

- The whole voice → brain → planner → gate → vehicle chain, including missions to a named
  object, closed-loop approach, visual memory and recall.
- Link loss: hover after 1 s, land after 5 s. Frozen safety core: C watchdog lands in about
  270 ms (ArduPilot SITL).
- The person check stopping or refusing a forward move (detector output faked).

**Not working / not built:**

- Indoor position hold on optical flow: quality fell to 0 at hover throttle. Indoors is demo
  mode (the gate judges every command; nothing flies).
- Outdoors, optical flow is computed and sent to the flight controller but not used: position
  is GPS, the gate doesn't check flow, and there is no fallback to it.
- Palm stop, the forward obstacle sensor's hardware, a public cloud deployment.

## Latest changes: review fixes (2026-11-22)

**`edge/` and `tools/` rows need a SITL run and the drone lead's review before flying.**
Not run: ArduPilot SITL, the real drone.

| # | Problem | Fix | Checked how |
|---|---|---|---|
| 1 | "Test four" (or test 4 armed) swallowed stop / land sentences; "a test for me" counted as test four | `station/brain.py`: a land or stop in the sentence is sent, nothing else | tests + phrase probe |
| 2 | The test-4 refusal said "there's a person in my path" with nobody detected | brain + `tools/voice_flights.py`: says forward is locked and that it sees nobody; banner only on a real detection | tests |
| 3 | The rules brain (LLM down) obeyed negations ("don't take off" took off); "abort", "halt" did nothing | negated clauses are dropped; every urgent word acts | tests + probe |
| 4 | Pi power loss = landing without the rangefinder | **No code fix** (needs the HC-SR04 wired to the flight controller) | — |
| 5 | The rangefinder height guard ended after an edge reconnect and never restarted | `station/planner.py`: waits for telemetry; the next takeoff / move adopts a flight with no guard | sim tests |
| 6 | Takeoff armed without checking the FC accepted GUIDED | `edge/mav.py`: waits up to 3 s for GUIDED, else fails the takeoff without arming | unit tests with a fake link; **not SITL** |
| 7 | Height corrections pushed the commanded height up without limit when the drone didn't follow | `station/planner.py`: net corrections capped at ±0.45 m per target | sim test |
| 8 | `deploy_pi --restart` / `field_setup` restarted the stack under a flying drone | refuses while the FC says armed (`--force` overrides) | **read only: needs the Pi** |
| 9 | Transmitter-loss and low-battery actions were set nowhere | `tools/fc_profile.py`: `FS_THR_ENABLE=3`, `BATT_FS_LOW_ACT=1`, `BATT_LOW_VOLT=10.5`. **The preflight says NO-GO until they are applied (`fc_profile --apply`, pilot's OK)** | test |
| 10 | Forward steps of 0.2 m or less skipped the person check | `edge/config.py` `forward_check_min_m` 0.02, used by the gate and the executor | test |

## Known limits

- Heavy for 3S: about 80% throttle to hover, so little thrust is left to stop a sink.
- Root cause of one uncommanded sink + yaw in GUIDED is still unknown; the laptop guard lands
  on both.
- The person detector watches only the path ahead; its distance estimate is uncalibrated.
- The laptop's rangefinder guard needs the laptop and the link; the edge's own height checks
  use the flight controller's estimate.
- If the Pi loses power the flight controller lands without the rangefinder.
- The Pi uses the QNX image's default logins: change them before using a shared network.
