# Second review: fresh eyes (2026-11-21)

A review done from the repo alone, at commit `684d231`, with no briefing beyond a two-line
description of the project. Phase 1 looks at the design; Phase 2 checks the code against it.
No code was changed. It follows [review.md](review.md) and does not repeat what that one fixed.

**For the partner reviewing this:** every finding says how it was checked. "Verified" means it
was reproduced by running something, and the script is in `tools/review/` so you can run it
yourself. "Read only" means it comes from reading the code and should be confirmed before
anyone acts on it. Tick the boxes at the bottom as you confirm or reject each one.

## Verdict

The safety architecture is well thought out and the edge code is careful. But the real drone
cannot yet hold position: the piece that gives it a position does not exist, and its core
assumption is untested. Three bugs in the flight-controller path matter with unguarded props.

## How to reproduce

```sh
.venv/bin/python -m pytest -q                      # 36 passed
.venv/bin/python -m tools.review.sim_exp           # bugs 4-7, kinematic sim, no Docker
.venv/bin/python -m tools.review.shm_restart       # bug 8
# bugs 1-3: a private SITL on port 5770, so the usual hope-sitl container is left alone
docker run --rm -d --name hope-sitl-review -p 5770:5760 hope-sitl
.venv/bin/python -m tools.review.sitl_exp
docker rm -f hope-sitl-review
```

The SITL container runs ArduPilot with default parameters. Numbers on the real board depend on
the board's parameters.

---

# Phase 1: the design

## One thing to settle first

The README says position comes from a camera tracking ArUco markers, and that is also how the
project was described to the reviewer. PRD decision D1 replaced that with optical flow from a
downward webcam, with ArUco as "the fallback behind the same interface". There is no ArUco code
and no such interface: ArUco would need a different MAVLink message
(`VISION_POSITION_ESTIMATE`) and different flight-controller settings, while the edge only has
`send_optical_flow`. The team is not sharing one picture of how the drone knows where it is.

## Spoken command to movement

The chain is coherent on paper: phone, Vultr (speech, LLM), laptop "station" (planner), Pi
(safety gate), flight controller. The Pi-side contract (`shared/protocol.py`) is clear and
tested. The weak spots:

- **Everything above the Pi is unwritten.** There is no `station/` or `cloud/` directory, so no
  planner, voice loop, dashboard or cloud link.
- **The LLM command schema is undecided.** The model test in [models.md](models.md) used `move`
  and `task` tools, while the PRD says a single `mission(where, what, after)` call.
- **Who talks to the Pi is ambiguous.** The header of `shared/protocol.py` says the station is
  Person B's; the PRD gives the planner to Person A.
- **"Fly to the door" needs a range to the door.** That comes from an object detector that
  Vultr does not host, feeding a `locate()` function that is not written.
- **Nobody owns "the cloud died".** The design says the current move finishes; after that the
  drone hovers until the battery limit lands it.

## Safety story, failure by failure

| What dies | What the design says | What would actually happen today |
|---|---|---|
| Cloud | Current move finishes | The drone hovers indefinitely; only `tools.cmd` or the pilot can land it |
| Laptop or Wi-Fi | Hover at 1 s, land at 5 s | Works in both simulators. On the real drone, hover and land both need working positioning |
| Laptop planner hangs, process alive | Not covered | The heartbeat keeps going from its own thread, so the watchdog never fires (bug 7) |
| Vision process | The supervisor restarts it; the gate blocks moves | No supervisor exists. New forward moves are blocked, but a move already in progress carries on (bug 4) |
| Flow process | The gate refuses moves, the drone lands | Same gap: a running move continues, and the drone flies on inertial guesswork until the flight controller notices |
| Edge core on the Pi | The flight controller lands itself | Only if three failsafe parameters are set on the real board. They are not yet (STATUS item A4) |
| Pi loses power | Same | Same dependency, and flow dies with it, so the landing has no position hold |
| Everything | The pilot flips Land | The switch is still Stabilize/Land. After any land command the edge fights the pilot (bug 1) |

Two structural points:

- Position sensing passes through the Python core (flow process → UDP → core → MAVLink). A core
  crash removes the heartbeat and the position at the same moment.
- The person check only looks forward. "Move back 1 metre", a demo command, flies blind. One
  fix: always turn to face the direction of travel before moving.

## Indoor positioning

The plan is optical flow for horizontal speed, the barometer for height, and the compass for
heading. Weak points, most serious first:

- **Flow without a rangefinder is untested.** Flow measures angles; turning them into speed
  needs height. Barometer height indoors under prop wash is easily off by 0.3 m or more, inside
  a 0.3 to 1.5 m altitude window.
- **Compass heading indoors.** Building steel and motor currents can swing it, which makes
  position hold circle.
- **Flow only gives speed, so position drifts.** The geofence and "return home" are measured
  against a drifting estimate.
- **On the ground the camera sits a few centimetres from the floor.** The gate needs good flow
  quality before takeoff, so takeoff may be refused.

## Assumptions nobody has verified

- The drone flies at all: every drone part in [hardware.md](hardware.md) is unticked and no
  manual hover has happened.
- Flow-only position hold works (every simulator pass so far used simulated GPS).
- QNX will run two USB cameras at once, and power them.
- The camera grabber compiles on QNX.
- The Pi and laptop can reach each other on the venue Wi-Fi.
- The Vultr vision models can return object locations.

One assumption is now closed: ArduPilot's `AP_OpticalFlow_MAV.cpp` on master does read the
`flow_rate_x/y` fields the code sends, whenever they are non-zero (STATUS open question).

## Is it realistic with a day left?

Not the full pitch. Reaching "fly to the door" on the real drone needs about ten unbuilt or
unverified things to work in sequence. The plan's own fallback is the realistic target: the
safety and watchdog story on QNX with the drone tethered or on the bench, and voice driving the
simulator. Commit to that now and treat free flight as a bonus.

---

# Phase 2: the code

## Where it matches the design

The gate, the one-task-at-a-time executor, the watchdog, the battery and lost-position
landings, the USB relay and the station allowlist all do what the PRD says. All 36 tests pass,
and the earlier review fixes (A1 to A13 except A4) are present.

## Bugs, most serious first

### 1. After a land command, the edge overrides the safety pilot
`edge/mav.py:143-151` re-sends LAND every 0.5 s until the drone disarms, whatever mode the
pilot selects. In SITL, switching to AltHold mid-landing was undone 0.5 s later:
`[(0.0, 'LAND'), (0.05, 'ALT_HOLD'), (0.5, 'LAND')]`. This contradicts the file's own rule that
the pilot always wins. **Verified** (`sitl_exp`, E1).

### 2. The speed limits are not enforced
The config says 0.5 m/s horizontal and 0.3 m/s vertical (`edge/config.py:14-15`). The only
enforcement is one `DO_CHANGE_SPEED` sent before takeoff (`edge/mav.py:250`). In SITL a 2.5 m
move peaked at **2.13 m/s**, the takeoff climbed at **1.21 m/s**, and a 0.4 m climb hit
**0.56 m/s**. The real limits must be set as flight-controller parameters and then measured.
STATUS mentions the parameters, but they are on no to-do list. **Verified on SITL defaults**
(`sitl_exp`, E2).

### 3. "Stop" just after arming does not stop the takeoff
If `stop` arrives after `NAV_TAKEOFF` is sent but before lift-off, `edge/executor.py:165-168`
does nothing because the drone counts as not airborne. The takeoff is reported aborted, `stop`
reports ok, and the drone climbed to 1.08 m anyway. `land` does work in that window.
**Verified** (`sitl_exp`, E3).

### 4. A dead vision or flow process does not stop a move in progress
`edge/server.py:197-199` only marks the detector dead; nothing cancels the running move. A
forward move continued 2 m after vision died. **Verified for vision** (`sim_exp`, E6). Flow is
the same pattern (`edge/safety.py:101` only checks new commands): **read only**.

### 5. After the pilot hands control back, the next relative move starts from the old spot
The executor keeps its last commanded target (`edge/executor.py:84-85`). After a simulated
takeover that moved the drone 2 m, "forward 0.5" flew it back to the old position plus 0.5.
**Verified** (`sim_exp`, E7).

### 6. The drone cannot back away from a close person
The gate allows the retreat, but `edge/server.py:209-212` aborts any running move when a person
is near. A 1 m retreat was aborted after 3 cm. **Verified** (`sim_exp`, E8);
`tests/test_edge.py:215-218` asserts this behaviour.

### 7. A hung planner looks alive
The heartbeat thread in `shared/client.py:76-84` keeps beating while the station's main loop is
frozen. After a 7 s freeze there was no failsafe. The A10 fix cured false alarms and created
this blind spot; the planner should have to "pet" the heartbeat. **Verified** (`sim_exp`, E9).

### 8. Restarting the camera grabber silently freezes its readers
`edge/camgrab/camgrab.c:171` deletes and recreates the shared memory, and readers keep the old
mapping. `read()` in `edge/camera.py:117-131` then returns the last frame forever with no age
check, so a vision process could keep reporting "no person". This breaks the "kill it and it
restarts" demo. **Verified on the Mac with the Python writer** (`shm_restart`); not run on QNX.

### 9. Takeoff may be refused on the real flow-only drone
`edge/mav.py:53,228-229` requires the "position is good" flag. Before arming with flow only,
ArduPilot may report just a "predicted" one. **Read only**, from memory of ArduPilot; check it
in the flow-only SITL run.

### 10. The flight-controller heartbeat has little margin
It is sent once a second (`edge/mav.py:125`) against a planned `FS_GCS_TIMEOUT=2`, so two late
beats land the drone. **Read only.**

### 11. A timed-out takeoff can leave the drone armed on the ground while telemetry says IDLE
`edge/executor.py:187-190`. **Read only.**

### 12. One low battery reading triggers auto-land
`edge/server.py:194` has no debounce; voltage sag under throttle could end a demo flight.
**Read only.**

### 13. The Pi has default passwords written in the docs
[qnx.md](qnx.md) lists the login and root passwords, and the plan prefers venue Wi-Fi. Anyone
on that network could log in as root. `--station-ip` is also optional
(`edge/__main__.py:45`). **Read only.**

## Design with no code behind it

- Optical flow, person and palm detection, the supervisor, the C watchdog loop and the jitter
  graph.
- A height sensor input.
- The whole station and cloud.
- Gestures and visual memory.
- The ArUco fallback.

## Partner cross-check

| # | Finding | Reproduced? | Agree? | Notes |
|---|---|---|---|---|
| 1 | LAND retry overrides the pilot | Read (`mav.py` land retry ignores mode) | ✅ | Fixable: stop retrying once the pilot picks a non-GUIDED/LAND mode |
| 2 | Speed limits not enforced | Read (only one `DO_CHANGE_SPEED`) | ✅ | Fixable: FC params `WPNAV_SPEED=50`, `WPNAV_SPEED_UP=30`, `WPNAV_SPEED_DN=30`, then measure |
| 3 | Stop before lift-off ignored | Read (`stop` doesn't call `cancel_takeoff`; `land` does) | ✅ | Fixable: one line, same as `land` |
| 4 | Dead vision/flow doesn't stop a running move | ✅ `sim_exp` E6 on Windows | ✅ | Fixable: cancel the running move (hold) when vision or flow goes stale |
| 5 | Stale target after pilot hand-back | ✅ E7 | ✅ | Fixable: reset the commanded target to the measured position on GUIDED re-entry |
| 6 | Can't back away from a person | ✅ E8 | ✅ | Fixable: abort only moves with a forward component |
| 7 | Hung planner looks alive | ✅ E9 | ✅ | Fixable: the heartbeat thread beats only if the main loop has "petted" it in the last ~0.5 s |
| 8 | Grabber restart freezes readers | Read (`camgrab.c:171` unlinks + recreates) | ✅ | Fixable: don't unlink if the region exists; readers reopen when frame age > 0.5 s |
| 9 | Flow-only takeoff may be refused | n/a | Plausible | Check in the flow-only SITL run; accept the predicted-position flag before arming if so |
| 10 | Heartbeat margin | Read (1 Hz vs `FS_GCS_TIMEOUT=2`, now set on the real FC) | ✅ | Fixable: send at 5 Hz |
| 11 | Armed on the ground after a failed takeoff | n/a | Plausible | Fixable: disarm on takeoff timeout if still on the ground |
| 12 | Battery auto-land has no debounce | n/a | ✅ | Fixable: require about 3 s below the threshold |
| 13 | Default passwords, optional allowlist | n/a | ✅ | Fixable: change the Pi passwords before the venue; make `--station-ip` required with `--mav` |
| 14 | **New:** client ack race (`shared/client.py`): a fast ack raised `KeyError` | ✅ 5 tests failed on Windows | ✅ | **Fixed** (local future reference); suite 34 passed + 2 SITL skipped |
| — | Positioning: flow (PRD) or ArUco (README)? | n/a | ✅ Flow | D1 stands; README is stale. ArUco has no code; treat it as an idea, not a fallback |
| — | Demo target: bench/tethered QNX + sim voice | n/a | Partly | Aim for real flight; keep the bench/tethered QNX demo ready as the fallback (D15). Not cutting scope now |


**Person A cross-check (2026-11-21):** every finding is a local, fixable bug, with nothing that changes the architecture. The deep risk is unchanged: flow-only positioning has never been tested (run the flow-only SITL, then the real-drone sign check).
