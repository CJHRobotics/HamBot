# HamBot — Backend Abstraction Handoff

Status: **design finalized, no implementation started.** Baseline is `v0.2.0`.

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
| `lidar.py` | `scan_data` slots are never expired. A stale reading from 30 s ago is indistinguishable from a fresh one. |
| `imu.py` | Threaded poller with cached getters — same staleness class of problem, though it does track `_last_update_s`. |
| `robot.py` generally | Free-running background threads; no guarantee that two getters called in sequence describe the same instant. |

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

- `bot.step(dt)` exists, blocks to the period boundary, and returns falsey on
  shutdown.
- Two getters called after one `step()` describe the same instant.
- Stale lidar slots are expired rather than returned as valid readings.
- Overrun is detectable by student code, and the policy is documented.
- **All four demos run unmodified on real hardware**: `systems_check.py`,
  `remote_control.py`, `color_shuttle.py`, `cameraGUI.py`.
- Ctrl-C still stops the motors.

That last pair is the real gate. The existing demos are the compatibility test
suite; if they need edits, the API changed and the change is wrong.
