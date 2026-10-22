# HamBot — Backend Abstraction Handoff

Status: **step 1 implemented; 4 of 6 §7 boxes verified on hardware (hambot-07). Demos + Ctrl-C still open. Not yet tagged.** Baseline is `v0.2.0`; step 1 lands on branch `step-control-tick` as `robot_systems` 0.3.0. See §8.

This document exists so a session starting cold can pick up the backend
abstraction work without re-deriving the design or re-litigating decisions that
are already settled. Read it end to end before writing code — several of the
obvious-looking moves here are wrong for reasons recorded below.

---

## 1. Why this work exists

The near-term goal is **not** new hardware. It is a stable student-facing Python
API with a swappable backend underneath, so that the simulator, the current
LEGO Build HAT robot, and any future motor controller all present the identical
interface to student code.

The trigger question was "should we replace the LEGO Build HAT with an OpenCR
board?" The answer we landed on is: **that question comes last, not first.** It
is a backend swap behind an interface that does not exist yet. Build the
interface; the hardware decision gets easier and cheaper afterwards.

The user's own framing: the LEGO large motors are *"great for the most part"*
and have adequate torque and encoders; DYNAMIXELs are *"just of better
quality."* That is a real but modest gain, and not on its own a reason to
rebuild the fleet.

---

## 2. Current state of the repo

### Layout

```
packages/robot_systems/   # core student library — the thing being refactored
packages/hambot_oled/     # SSD1306 network display (unaffected)
packages/hambot_link/     # console control listener + camera stream
demos/                    # student-facing scripts, symlinked to ~/Desktop/Demos
deploy/                   # Pi setup scripts
```

### The coupling is much smaller than it looks

`robot.py` is 879 lines with 49 public methods, but:

- `from buildhat import Motor` appears **once**, at `robot.py:2`.
- Every Build HAT call is inside the `HamBot` class.
- The entire backend surface to reimplement is: `Motor(port)`,
  `set_speed_unit_rpm()`, `.start(speed)`, `.stop()`, `.run_for_seconds()`,
  `.run_for_rotations()`, `.run_to_position()`, `.get_position()`.

Most of those 49 methods are a facade of per-motor wrappers that can sit
unchanged on top of a different backend.

### What students actually use is smaller still

Grepping `demos/` and `packages/hambot_link/src/hambot_link/teleop.py`, the
entire consumed API is:

`set_left_motor_speed` · `set_right_motor_speed` · `run_motors_for_seconds` ·
`stop_motors` · encoder getters · `get_heading` · `get_range_image` ·
`camera` · `disconnect_robot`

**Preserving these is the hard compatibility constraint.** Everything else has
room to move.

### Known weak points in the current code

These are not incidental bugs — the redesign is partly motivated by them.

| Location | Problem |
|---|---|
| `robot.py:130` `_read_motor_delta` | Hand-rolled ±180° wrap handling for the Build HAT position sensor. Disappears with any 32-bit multi-turn encoder. |
| `lidar.py` | ~~`scan_data` slots are never expired. A stale reading from 30 s ago is indistinguishable from a fresh one.~~ **Fixed in step 1** — per-slot monotonic write times and a `stale_after_s` window. |
| `imu.py` | Threaded poller with cached getters — same staleness class of problem, though it does track `_last_update_s`. |
| `robot.py` generally | ~~Free-running background threads; no guarantee that two getters called in sequence describe the same instant.~~ **Fixed in step 1** — `step()` latches one coherent `Snapshot` and the getters read from it. |

---

## 3. The finalized architecture

Three layers. Each has a different rate of change and a different owner.

```
  ┌─────────────────────────────────────────────┐
  │  HamBot  — student API. Webots-shaped.      │   never changes
  │  bot.step(dt); bot.set_left_motor_speed(30) │
  ├─────────────────────────────────────────────┤
  │  Backend protocols                          │   the new seam
  │  DriveBackend / ScanSource / HeadingSource  │
  ├─────────────────────────────────────────────┤
  │  buildhat │ opencr │ ros2-daemon │ webots   │   swappable
  └─────────────────────────────────────────────┘
```

### 3.1 `step()` — the highest-value change, and the cheapest

`HamBot` is already a Webots controller in shape: one object, devices as
attributes, plain method calls. The one thing it lacks is Webots' **tick**.

Webots controllers are synchronous:

```python
while robot.step(timestep) != -1:
    d = lidar.getRangeImage()
    left.setVelocity(v)
```

HamBot is free-running, which is why sensor readings can be stale and mutually
inconsistent. Adding a tick fixes staleness, makes loop timing deterministic,
and makes student code portable to and from simulation:

```python
while bot.step(0.05):        # False on shutdown, mirroring Webots' -1
    heading = bot.get_heading()      # same instant
    scan    = bot.get_range_image()  # same instant
```

**Do this first.** It costs days, not months, and every later layer depends on
its semantics.

Requirements:

- `step(dt)` blocks until the next period boundary, then refreshes all sensors
  into one coherent snapshot.
- Getters read from the local snapshot. No I/O per getter.
- Getters must still work **without** `step()` — implicit refresh if the
  snapshot is stale — so intro-week `while True: ... sleep(0.1)` code keeps
  running unchanged.
- Setters coalesce locally and flush on the next `step()`.

**Decided in step 1 — overrun policy: slip.** Recorded below as originally
written; the reasoning stands and the implementation follows the
recommendation. Documented in `packages/robot_systems/docs/step.md`.

**Open decision — overrun policy.** In a simulator `step()` is authoritative and
cannot be late. On hardware it can. When the loop body takes 200 ms and `dt` is
50 ms, either:

- **skip** to the next boundary (real-time, silently drops work), or
- **slip** (never drops, but drifts off wall-clock).

Recommendation for a teaching platform: **slip, and expose an overrun
counter/flag** so the student is told their loop is too slow rather than
silently observing dropped cycles. Not finally decided — make the call and
document it, because it is most of the semantic content of `step()`.

### 3.2 The backend seam

```python
class DriveBackend(Protocol):
    def set_wheel_velocity(self, side: str, rpm: float) -> None: ...
    def read_encoders(self) -> dict[str, float]: ...   # radians
    def stop(self) -> None: ...
```

Same idea for `ScanSource` and `HeadingSource`. `HamBot.__init__` selects a
backend from config or an env var.

Planned implementations:

| Backend | Notes |
|---|---|
| `buildhat` | Wraps today's behavior. A pure refactor — demos must stay green. |
| `webots` | **The sleeper win.** Same student file runs in sim and on hardware. Cheapest backend to write, since Webots' API is already the target shape. |
| `ros2` | Via the daemon below. |
| `opencr` | Serial. Last, and only if the bench spike justifies it. |

### 3.3 The ROS2 daemon (if ROS2 happens at all)

Architecture, as pitched and refined:

- **One long-lived privileged container**, started by systemd at boot, host
  networking, bind-mounting the repo. Not `docker run --rm` per script.
- Container owns lidar, IMU, and motors, and exposes a port.
- The `HamBot` client library attaches to that port; students call the same
  methods as always.
- **The camera stays on Raspberry Pi OS, outside the container.** This is
  deliberate and important — see §4.
- Students never type `docker`. Wrap it: `hambot-run demos/foo.py`,
  `hambot-shell`, `hambot-ros2 topic list`, installed into `/usr/local/bin`.
  `deploy/setup_link.sh:42` already establishes exactly this symlink pattern.

This is a well-trodden pattern, not an invention — Player/Stage, Viam, and the
Spot SDK all have this shape. Webots itself supports extern controllers over
TCP, which is why the sim backend later becomes "point the client at a
different port."

Design requirements, in rough priority order:

1. **`step()` is the batch point.** One round trip per tick — flush commands,
   receive a full sensor snapshot. Per-call RPC would mean 8 round trips per
   loop iteration and, worse, readings from different instants.
2. **Watchdog. Non-negotiable.** Today motors stop on Ctrl-C because the SIGINT
   handler shares a process with the motor objects. Once split, a `kill -9`,
   a crash, or a dropped SSH session leaves the robot driving into a wall.
   Stop motors on client disconnect **and** on ~300–500 ms of command silence.
   Done right this is a safety *upgrade*.
3. **Arbitration.** Two students, two scripts, one set of motors.
   `packages/hambot_link/src/hambot_link/locking.py` already solves this for
   the console — promote it into the daemon. Exclusive drive lease with a clear
   rejection message; sensor reads can stay multi-client.
4. **Typed errors.** Connect failure → *"Can't reach the HamBot service. Try
   `sudo systemctl status hambot`."* Hardware fault → `HamBotHardwareError`,
   not a hang or a raw JSON dict.
5. **Protocol version handshake on connect.** Container image and pip library
   will drift across a fleet. Fail loudly at connect, never mysteriously on
   call 47.
6. **Boot race.** Student scripts will start before the container is ready.
   Retry on connect for a few seconds with a friendly message.

Transport: **TCP + JSON-lines on localhost** for v1. Trivial with
`--network host`, debuggable with `nc`, leaves the door open to off-robot
clients. A 360-point scan as JSON is ~4 KB; at 20 Hz that is nothing. Revisit
MessagePack only for high-density scans.

**Open decision — protocol altitude.** Wheel-level (`set_left_motor_speed`,
`get_left_encoder`) or robot-level (`/cmd_vel`, pose)? Today's API is
wheel-level and that is pedagogically better — students should feel differential
drive, not have it hidden. But nav2 and `/cmd_vel` are robot-level. The daemon
likely needs both, **with an explicit rule for who wins when both arrive.**
Settle this before writing the protocol, not after.

---

## 4. Decisions already made — do not re-litigate

**ROS2 is not required by the OpenCR board.** That dependency comes from the
stock TurtleBot3 firmware, not the hardware. The board is an Arduino-compatible
STM32F746.

**Docker is the only sane ROS2-on-Raspberry-Pi-OS path.** There are no official
ROS2 binaries for Pi OS — only Ubuntu. Switching the fleet to Ubuntu means
re-validating `picamera2` and the whole camera stack. Building from source is
6–10 h/build and fragile.

**Keep the camera on the host, outside the container.** `picamera2` in Docker
needs `/dev/dma_heap`, `/run/udev`, and a libcamera version matching the host
exactly; mismatches fail cryptically. This is ~80% of the container risk and it
is entirely avoidable, because the ROS2 side only needs **drive, lidar, and
IMU**. Reuse the existing socket in
`packages/hambot_link/src/hambot_link/camera_stream.py` if ROS2 ever needs
frames.

**OpenCR implies replacing the motors.** The board has no H-bridge and no LEGO
LPF2 port — it drives DYNAMIXELs over a TTL/RS485 bus. LEGO PoweredUp motors
are proprietary to the Build HAT and cannot be driven from it. So "swap the
board" is really "rebuild the drivetrain": new motors at roughly $50–90 each,
plus mounts, hubs, and chassis work. The software is the cheap part.

**OpenCR firmware belongs in a separate repo** (`hambot-opencr-firmware`).
Different language, toolchain, and build system; it is flashed to hardware
rather than pip-installed, and versions against boards, not Python releases.

**Everything Python/Docker stays in this monorepo, on short branches.** Once the
backend seam exists, nearly all remaining work is *additive* — new packages
nothing imports until someone opts in. Additive work does not need isolation. A
long-lived `opencr` branch would diverge for months against a fleet that keeps
shipping fixes; short branches merged often avoid that entirely. The one thing
worth isolating is the `step()` semantics change, since it alters behavior
students depend on: branch it, run all four demos on real hardware, then merge.

**What justifies ROS2, if anything.** At drive+lidar+IMU behind a facade,
ROS2 buys exactly nav2, slam_toolbox, RViz, and TF. If students never look past
`bot.get_range_image()`, the Docker tax buys nothing. The genuine argument is
the **curriculum ladder** this architecture gives for free: the container is a
real ROS2 node graph, so an intro student can drive with
`bot.set_left_motor_speed(30)` while an advanced student, *on the same robot in
the same session*, runs `ros2 topic echo /scan`, opens RViz, records a rosbag,
or launches slam_toolbox. Same hardware, two levels of abstraction, no rebuild
between rungs. Build toward that deliberately or skip ROS2.

---

## 5. Sequencing

Ordered by value per unit of risk. Steps 1–3 need no Docker, no ROS2, no
Ubuntu, and no new hardware, and deliver most of the stated goal.

| # | Work | Rough effort |
|---|---|---|
| 1 | `step()` + freshness/coherence guarantees in `HamBot` | ~1 week |
| 2 | Extract backend protocols; make Build HAT one implementation | ~1 week |
| 3 | Webots backend — sim/real parity for student code | 1–2 weeks |
| 4 | ROS2-in-Docker daemon, **only if 1–3 left a real gap** | several weeks |
| 5 | OpenCR backend, behind the now-stable interface | after the spike |

**Before step 5, do a disposable bench spike.** A bare OpenCR and two motors on
a desk, no chassis. Prove exactly one thing: *can I command velocity and read
encoders over serial, at what rate, and with what latency?* Those numbers set
the tick rate, which sets the daemon protocol. Do not design against hardware
that has not been confirmed to talk. Keep the spike in a scratch directory or
throwaway repo — it is not the product.

---

## 6. Fleet and deploy constraints

This repo is installed on real robots by Ansible. Breaking `main` breaks a
classroom, so:

- **`v0.2.0` (`c5275ac`) is the known-good baseline** and the marker for the
  state before any of this work. Restore any robot by hand with
  `./deploy/install.sh --ref latest all`.
- **The Ansible playbook owns version selection**, not `install.sh`. It lives
  in the sibling `lab-playbooks` repo at `hambots/hambot-installation/`, checks
  out `hambot_repo_version` (now pinned to `v0.2.0`, not `main`), and only then
  runs `install.sh`. A commit cannot reach the robots until it is **tagged**.
- **`install.sh --ref` defaults to `current`** — a bare run touches git not at
  all. This is deliberate: an opinionated default silently overrode Ansible's
  pin and hard-failed the rollout on robots the playbook intentionally leaves
  with local edits. Do not "fix" this back to `latest`.
- **Do not tell anyone to `git pull`.** Once the installer has checked out a
  version the repo sits on a detached HEAD, where `git pull` refuses to run.
  The update path is `./deploy/install.sh --ref <ref> <target>`.
- **Tag before expecting a rollout.** Cut and push a tag, then move
  `hambot_repo_version` in `lab-playbooks`. Beware that a tag created early in a
  work session can go stale as later fixes land — verify what it points at
  before pushing.
- Test rollouts go to `hambot-test` first (`ansible-playbook -l hambot-test ...`,
  or `-e hambot_repo_version=main` to try untagged work).

---

## 7. Definition of done for step 1

A concrete first milestone, so the next session has an unambiguous target:

- [x] `bot.step(dt)` exists, blocks to the period boundary, and returns falsey on
  shutdown. *(`StepContractTests`; **verified on hambot-07** — 50.0 ms median
  period at `dt=0.05`, `step()` returns False after disconnect)*
- [x] Two getters called after one `step()` describe the same instant.
  *(`CoherenceTests`; **verified on hambot-07** — see §9)*
- [x] Stale lidar slots are expired rather than returned as valid readings.
  *(`tests/test_lidar.py`; **verified on hambot-07** — measured distribution in §9)*
- [x] Overrun is detectable by student code, and the policy is documented.
  *(`LoopHealthTests`, `SlipNotSkipTests`, `docs/step.md`; **verified on
  hambot-07** — 0 overruns in 200 cycles with lidar + IMU + camera live)*
- [ ] **All four demos run unmodified on real hardware**: `systems_check.py`,
  `remote_control.py`, `color_shuttle.py`, `cameraGUI.py`. **Still open — needs
  a robot.**
- [ ] Ctrl-C still stops the motors. **Still open — needs a robot**, including
  the new case of Ctrl-C landing during a `step()` wait.

That last pair is the real gate. The existing demos are the compatibility test
suite; if they need edits, the API changed and the change is wrong.


---

## 8. Step 1 — what landed

Branch `step-control-tick`, `robot_systems` 0.3.0. 77 tests pass off-hardware
(`cd packages/robot_systems && python3 -m unittest discover -s tests`). **The
hardware gate in §7 has not been run.** Nothing is tagged, so nothing has
reached a robot.

### New files

| File | What it is |
|---|---|
| `src/robot_systems/ticker.py` | `Ticker` (pacing, overrun accounting) and `Snapshot` (one coherent sensor read). No hardware imports, so the timing semantics are testable off-robot. |
| `docs/step.md` | The tick contract, the slip policy and its cost, the two modes, per-subsystem behaviour, how to choose `dt`. |
| `tests/fake_hardware.py` | Stubs for `buildhat`, `board`, `adafruit_bno055`, `adafruit_rplidar`, plus minimal `numpy`/`cv2`. Also `FakeClock`, which is what makes the pacing tests deterministic. |
| `tests/test_ticker.py`, `tests/test_lidar.py`, `tests/test_robot_step.py` | `unittest`, matching the `hambot_link` style. No pytest, no numpy — neither is installed on a stock robot. |
| `demos/ticked_drive.py` | The one *ticked* demo. The other four are free-running and stay that way; they are the compatibility suite. |

### Changed

- **`robot.py`** — `step(dt)`, `_refresh_snapshot()`, `_ensure_fresh()`,
  `_command()`/`_flush_commands()`/`_direct()`, `overruns` /
  `last_overrun_s` / `worst_overrun_s` / `loop_is_keeping_up()`,
  `get_camera_frame(copy=True)`, `camera_frame_age()`, `snapshot()`.
  Every `set_*_motor_speed` routes through `_command`; every stop and every
  timed/rotation/position run routes through `_direct`.
- **`lidar.py`** — `scan_time` array, `stale_after_s=1.0`,
  `get_current_scan(max_age_s=-1.0)`, `slot_age()`, `sample()`.
- **`imu.py`** — `sample()` returning `(heading, reading_time)` under one lock
  hold, and a monotonic mirror of the update time.
- **`camera.py` / `pi_camera.py` / `oak_camera.py`** — monotonic capture stamp
  inside the existing frame lock, `sample_frame()`, `frame_age()`.
- **`docs/lidar.md`** — the new constructor arg, expiry, and write order.

### Decisions made while implementing

**Overrun policy is slip, not skip.** Per §3.1's recommendation. On a miss the
next boundary is one full period from *now*, so no cycle is dropped and the
loop never free-runs through zero-length waits catching up. The cost —
documented in `docs/step.md` and pinned by a test so nobody later "fixes" it —
is that after a miss, elapsed time is no longer `ticks * dt`.

**`get_heading()` still delegates verbatim when free-running.** Its
`fresh_within` / `blocking` / `wait_timeout` arguments have real behaviour that
a snapshot read cannot reproduce, and §7 makes demo compatibility the hard
constraint. Ticked, the call is pinned to the snapshot and those arguments no
longer apply; `bot.snapshot().heading_age()` is how you judge the reading's
age. This asymmetry is deliberate.

**The 20 Hz position thread stays.** The Build HAT reports position as a
wrapped ±180° angle, so a continuous total needs a sample twice per half
revolution (~0.3 s at max RPM). A student ticking at `dt=0.5` would alias the
wrap and lose whole revolutions if the tick were the only sampler. `step()`
only latches the total that thread maintains.

**`_direct()` cancels pending speeds, and that is a safety property.** Without
the cancel, `stop_motors()` is silently undone by the next flush one tick
later. `StopIsNotUndoneTests` exists specifically to keep that from regressing.

**The camera is off the tick and latched by reference.** It runs at 5 fps, so a
20 Hz loop sees the same frame ~4 times and `camera_frame_age()` says so.
Copying a ~900 KB frame per tick would cost more than every other sensor
combined, so `get_camera_frame()` copies on demand. Keeping the camera off the
tick also avoids baking in the dependency §4 wants avoided, since the camera is
what stays on the Pi OS host if the rest ever moves into a container.

**The command lock is re-entrant, and that is a Ctrl-C fix.** Found while
reviewing the diff, not by a failing test. The SIGINT/SIGTERM handler runs on
the *main* thread, so a signal arriving while that thread is inside
`_command()`, `_direct()` or `_flush_commands()` re-enters the lock via
`stop_motors()`. With a plain `threading.Lock` that deadlocks: Ctrl-C hangs
with the motors still turning. The windows are only a few bytecodes wide, but
Python checks for signals between bytecodes, so they are reachable — and this
is precisely §7's "Ctrl-C still stops the motors" box. `ShutdownReentrancyTests`
pins it; swapping the `RLock` back to a `Lock` hangs that test.

**`reset_encoders()` re-latches the snapshot.** The encoder getters read the
snapshot now, so zeroing only the live attributes would leave a reader seeing
pre-reset totals until the next refresh — a behaviour change from before.

### Still needs a robot

Nothing below has been exercised on hardware. **This is the actual gate; a
green test run does not substitute for any of it.**

1. **All four demos, unmodified**: `systems_check.py`, `remote_control.py`,
   `color_shuttle.py`, `cameraGUI.py`. If any needs an edit, the API changed
   and the change is wrong.
2. **Ctrl-C stops the motors**, including when it lands during a `step()` wait.
   `time.sleep` is interruptible on the main thread and the existing handler
   should still run, but that is reasoning, not evidence. The related
   re-entrancy deadlock is fixed and tested (see the `RLock` note above); what
   is untested is the real signal arriving during a real `step()` wait.
3. **`demos/ticked_drive.py`** — the tick has no on-hardware exercise at all
   yet. Run it on blocks first.
4. **Is `stale_after_s=1.0` right for a real unit?** It reasons from a 5–10 Hz
   spin refreshing visible angles every 100–200 ms. Off-hardware there is no
   way to tell whether a real scan leaves legitimately-visible slots quiet for
   longer; if it does, this blanks readings a demo wants and the default must
   be raised. Watch for obstacles flickering in and out of `scan[i] > 0`.
   Demo tolerance was checked and is fine: `color_shuttle.py` already filters
   on `scan[i] > 0`, and `systems_check.py` only prints four slots.
5. **Overrun counts under real sensor I/O.** A `dt=0.05` loop doing real lidar
   and IMU reads may overrun in ways stubs cannot show.

### Rollout

Per §6, and none of it done yet: do **not** cut the tag until the hardware run
passes. Then tag, move `hambot_repo_version` in the sibling `lab-playbooks`
repo, and roll to `hambot-test` first (`-e hambot_repo_version=main` to try the
untagged branch). The update path is `./deploy/install.sh --ref <ref> <target>`
— installed robots sit on a detached HEAD, so `git pull` does not work.

### Noticed, deliberately not fixed

`docs/lidar.md` documents a `frequency` constructor arg and a
`set_lidar_frequency()` method that do not exist in `lidar.py`. Pre-existing
drift, out of scope for step 1, and not propagated — the new `stale_after_s`
and `get_current_scan(max_age_s=...)` entries are accurate. Worth a separate
cleanup commit.


---

## 9. Hardware validation on hambot-07

Run 2026-10-22 at `0fdab75`, `robot_systems` 0.3.0, installed with
`./deploy/install.sh --ref step-control-tick hambot`. Four of §7's six boxes
are closed. **The two that remain are the demo compatibility suite and Ctrl-C.**

### Pacing — `dt=0.05` is honest

200 cycles with lidar, IMU and camera all live:

| | |
|---|---|
| median period | 50.0 ms |
| min / max | 48.6 / 51.3 ms |
| wall clock | 10.05 s (ideal 10.00 s) |
| overruns | **0 / 200** |

This was the result that could have forced a redesign — if real sensor I/O had
not fitted in a 50 ms budget, the 20 Hz default and most of `docs/step.md`
would have needed rethinking. It fits, with margin.

Snapshot ages at the moment of latching: snapshot 1 ms, heading 50 ms (one full
20 Hz poller period — the expected worst case), scan 3 ms, frame 37 ms. The
per-sensor `*_t` fields are doing exactly the job they exist for.

### Coherence — and a testing trap worth not repeating

**A stationary robot makes every pinning test vacuous.** The first on-hardware
check compared `get_heading()` and `get_encoder_readings()` across a 250 ms
pause and reported `pinned=True` — but the robot was still, so both values were
constants (`90.0` and `[0.0, 0.0]`) and the check would have passed with no
pinning implemented at all. It proved nothing.

The decisive test moves the hardware underneath while *not* calling `step()`:

```
live attribute   -0.0000 -> 8.2554 rad   (left wheel turned by hand)
getter, no step()  -0.0000 -> -0.0000     <- pinned
after one step()            8.2554        <- released, exact
```

That is the coherence guarantee on real hardware. Any future re-test of pinning
must perturb the sensor during the pinned window, or it is measuring nothing.

### Lidar — `stale_after_s=1.0` is correct, now measured rather than reasoned

| | |
|---|---|
| slots ever measured | 155 / 360 |
| median slot age | **138 ms** |
| p90 slot age | 1209 ms |
| max slot age | 11378 ms |
| positive readings, raw | 155 / 360 |
| survive the 1.0 s window | 134 / 360 |
| **blanked by expiry** | **21 / 360** |
| front window (160-200) | 14 / 41 valid, nearest 5538 mm |
| front validity flips / 60 ticks | 12 |

The 138 ms median is the 5-10 Hz spin refreshing visible angles every 100-200 ms
— the exact premise the 1.0 s default was picked from, now confirmed.

**The distribution is bimodal and 1.0 s separates it cleanly.** A tight cluster
near 138 ms (genuinely visible angles) and a long tail to 11.4 s. A slot last
touched 11 s ago is not a slowly-refreshing slot; it is one return off an
oblique surface, or something that moved through and left. Serving that as a
live distance is the §2 bug. Blanking it is the correct behaviour, not a false
positive.

**Only 21 of 360 slots change behaviour at all.** The 205 never-measured slots
already read `-1` before this branch (initialized to `INVALID_READING` and never
written), so there is no regression there — the adafruit driver simply does not
report angles it got no return from. 12 flips across 41 front slots over 3 s is
stable, not flickering.

**Decision: the default stays at 1.0 s.** Raise it only if a demo is observed
losing a reading it needs.

### Camera — behaves as designed

6 distinct frame objects across 20 ticks at 20 Hz, with ages cycling 0.04 ->
0.19 s and resetting. That is one 5 fps frame being seen roughly four times
with `camera_frame_age()` reporting it truthfully, which is the whole reason the
camera is off the tick.

### Noticed, not this branch's problem

`bot.imu.is_calibrated()` returns False on hambot-07, and a stationary heading
reads a constant (`90.0`, i.e. yaw 0 mapped through `(-yaw + 90) % 360`).
Rotating the robot by hand moved it to 140.3125 — a clean 1/16-degree BNO055
LSB — so the sensor *is* fusing and the tick reads it correctly. The
`# TODO: verify it works` on the heading conversion in `imu.py` is still
unresolved, and a demo reporting a heading that looks frozen on a stationary
robot is explained by calibration, not by `step()`.

### Still open — both need the robot on blocks

1. **All four demos, unmodified**: `systems_check.py`, `remote_control.py`,
   `color_shuttle.py`, `cameraGUI.py`. This is the free-running path, which
   `ticked_drive.py` does not exercise at all.
2. **Ctrl-C stops the motors**, including during a `step()` wait. Test with
   `python demos/ticked_drive.py --dt 0.5`, which keeps the robot asleep inside
   the wait ~99% of the time.

`demos/ticked_drive.py` itself ran clean on hambot-07 — all sensors read, no
overruns — so the ticked path has on-hardware exercise now. Note that
`install.sh --ref <branch>` is what actually moved the robot; the Ansible git
task did not (see §10).

---

## 10. Open problem: untagged rollouts do not reach a robot

Getting `0fdab75` onto hambot-07 took three attempts, and **the Ansible path
never worked**. This is unresolved and will block the next untagged rollout.

What was tried, in order:

1. **Lab management portal**, "HamBot repo version" = `step-control-tick`.
   Robot stayed at `8f2a217`. Suspected cause: the portal may validate that
   field against existing release tags and silently fall back to the group pin.
   `step-control-tick` is a *branch*; there is no tag for this work yet.
2. **CLI**, `-e hambot_repo_version=step-control-tick -e hambot_force_reinstall=true`.
   Robot still at `8f2a217`. Not diagnosed — the git task's actual output was
   never captured. `-e` is Ansible's highest precedence, so the group pin should
   not have won; something in the `ansible.builtin.git` task is not checking out
   a branch on a detached-HEAD clone.
3. **`./deploy/install.sh --ref step-control-tick hambot`**, run on the robot.
   Worked first try.

Why the symptom was invisible: `v0.2.3` points at `8f2a217`, which is *exactly*
`main`'s tip. So a fall-back to the group pin finds the repo already at that
commit, the git task reports **ok** rather than **changed**, and the playbook
then skips `install.sh` entirely — that task is gated on
`_hambot_git is changed`. The play looks like a clean success while having done
nothing.

Compounding it: `setup_hambot.sh:21` is a **non-editable** `pip install`, so
even a successful git checkout with a skipped `install.sh` leaves the venv on
the old `robot_systems`. Version and commit must both be checked after any
rollout:

```bash
cd /home/hambot/HamBot && git rev-parse --short HEAD; \
  ./hambot_venv/bin/pip show robot_systems | head -2
```

**To diagnose next time**, capture the git task's output and whether
`origin/<branch>` reached the robot's remotes:

```bash
sudo ansible hambot-07 -m shell -a 'cd /home/hambot/HamBot; git branch -r | grep -i step || echo "branch NOT fetched"'
```

If the branch is absent, the fetch is the problem; if present while HEAD has not
moved, the checkout is. Worth fixing properly — `install.sh --ref` is a fine
manual path but the playbook owning version selection (§6) is the whole point of
the Ansible layer.
