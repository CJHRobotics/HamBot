# The Control Tick — `bot.step(dt)`

`step()` gives a HamBot program a **tick**: a fixed-rate loop where every
sensor reading describes the same instant and every motor command lands at a
known point in the cycle.

```python
from robot_systems.robot import HamBot

bot = HamBot(lidar_enabled=True)

while bot.step(0.05):                 # 20 Hz
    scan    = bot.get_range_image()   # these two describe
    heading = bot.get_heading()       # the same instant
    bot.set_left_motor_speed(30)
    bot.set_right_motor_speed(30)
```

This is the shape a Webots controller has (`while robot.step(timestep) != -1`),
which is deliberate — the same student file should read the same way in
simulation and on hardware.

---

## The contract

Each `step(dt)` call does three things, **in this order**:

1. **Flush** any motor speeds set since the last tick.
2. **Wait** until the next period boundary.
3. **Sample** every sensor once into a single snapshot.

It returns `True` normally and `False` once the robot has been disconnected,
so `while bot.step(dt):` terminates on shutdown the way Webots' `!= -1` does.

Flushing *before* the wait is the point of that ordering: a speed set during
the previous cycle is applied for the whole of this period rather than only
for whatever is left of it after the wait.

---

## Two modes, chosen by behaviour

There is no constructor flag. A robot starts **free-running** and flips to
**ticked** the first time `step()` is called. The flip is one-way.

| | Free-running (no `step()`) | Ticked (after any `step()`) |
|---|---|---|
| Setters | write straight to the hardware | coalesce, flush on the next tick |
| Getters | refresh the snapshot when it is older than `snapshot_max_age_s` (0.05 s) | pinned to the tick's snapshot |
| Stops and timed runs | immediate | immediate |

**Free-running is the compatibility path.** Every existing demo takes it, and
its behaviour is indistinguishable from the library before `step()` existed.
Intro-week `while True: ... sleep(0.1)` code keeps working unchanged.

**Pinning is the coherence guarantee.** In ticked mode two getters called after
one `step()` return data from the same instant, because they are reading the
same latched snapshot rather than doing fresh I/O. That is the whole reason to
have a tick, so it is not relaxed for convenience.

---

## Overruns: slip, not skip

When the loop body takes longer than `dt`, the boundary has already passed by
the time `step()` is called. There were two options:

* **Skip** — jump to the next boundary that is still in the future. Stays
  locked to wall-clock, but silently drops cycles.
* **Slip** — put the next boundary one full period from *now*. Never drops a
  cycle, but drifts off wall-clock.

**HamBot slips.** On a teaching platform, a loop silently dropping every other
cycle is a bug a student cannot see; a loop running slower than requested is
one they can be told about. Slipping also means the loop never free-runs
through a burst of zero-length waits trying to catch up, which is what makes a
briefly-slow loop turn into a runaway one.

### The cost, stated plainly

**After an overrun, elapsed time is no longer `ticks * dt`.** Code that needs
real elapsed time must read a clock:

```python
import time
start = time.monotonic()
while bot.step(0.05):
    elapsed = time.monotonic() - start     # correct
    # elapsed = bot_ticks * 0.05           # WRONG once anything has overrun
```

### Seeing it happen

```python
while bot.step(0.05):
    if not bot.loop_is_keeping_up():
        print("this cycle was late")

print(bot.overruns)           # how many ticks missed their boundary
print(bot.last_overrun_s)     # how late the most recent one was
print(bot.worst_overrun_s)    # how late the worst one was
```

`loop_is_keeping_up()` describes the **most recent** tick, so it recovers after
a one-off hiccup. `overruns` is cumulative.

A blocking helper such as `run_motors_for_seconds()` blocks by design and
**will** register an overrun inside a ticked loop. That is correct and not a
bug — the loop really did stall.

---

## Per-subsystem behaviour

### Motors

Continuous speeds (`set_left_motor_speed`, `set_right_motor_speed`, and the
per-wheel 4WD setters) coalesce. Only the newest speed per motor is sent, so a
loop body setting a speed three times produces one hardware write.

Everything that cannot wait for a boundary goes straight to the hardware and
**cancels any pending speed for that motor first**:

* every `stop_*` method, including `stop_motors()`
* `run_*_for_seconds`, `run_*_for_rotations`, `run_*_to_position`

The cancel is a safety property, not an optimisation. Without it:

```python
bot.set_left_motor_speed(40)
bot.stop_motors()      # motor stops...
bot.step(0.05)         # ...and the pending 40 is flushed right back onto it
```

A stop must survive the next tick. There is a test pinning exactly this.

### Encoders

The 20 Hz position thread **stays**, and `step()` only latches the total it
maintains. This is not redundancy. The Build HAT reports position as a wrapped
±180° angle, so recovering a continuous total needs a sample at least twice per
half revolution — about every 0.3 s at max RPM. A student ticking at `dt=0.5`
would alias the wrap and silently lose whole revolutions if the tick were the
only sampler.

### IMU

`get_heading()` returns the tick's latched heading in ticked mode. Its
`fresh_within` / `blocking` / `wait_timeout` arguments describe a request the
tick has already made, so they do not apply there; use
`bot.snapshot().heading_age()` to judge the reading's age. Free-running, the
call delegates to the IMU exactly as before, blocking behaviour included.

### Lidar

Slots are now expired. A slot not written within the lidar's staleness window
(`stale_after_s`, default 1.0 s) reads as `-1` rather than as a stale distance,
so "nothing is there" is distinguishable from "nothing has been measured there
recently". Filter on `scan[i] > 0` before using a reading. See
[lidar.md](lidar.md).

### Camera

The camera is **not** on the tick. It runs at 5 fps, so a 20 Hz loop sees the
same frame about four times over; `step()` latches whatever the capture thread
has, and `camera_frame_age()` reports honestly how old that is.

Frames are latched **by reference, never copied**. A 640×480 RGB frame is about
900 KB, and copying one per tick would cost more than reading every other
sensor put together. `get_camera_frame()` copies on demand:

```python
frame = bot.get_camera_frame()             # a copy, safe to draw on
frame = bot.get_camera_frame(copy=False)   # the latched array, read-only
age   = bot.camera_frame_age()             # seconds since capture
```

---

## Choosing `dt`

| `dt` | Rate | Good for |
|---|---|---|
| 0.02 | 50 Hz | tight control loops; faster than the sensors update, so readings repeat |
| **0.05** | **20 Hz** | **the default — matches the IMU and encoder pollers** |
| 0.1 | 10 Hz | loops doing camera work or landmark detection |
| 0.2–0.5 | 2–5 Hz | deliberate, step-by-step behaviours |

Start at 0.05. The sensors underneath update at 20 Hz (IMU, encoders), 5–10 Hz
(lidar spin) and 5 fps (camera), so ticking much faster than 20 Hz mostly
returns you the same readings again.

If `bot.overruns` climbs, the body is too slow for the `dt` you asked for.
Either raise `dt` or do less work per cycle — image processing every cycle is
the usual culprit.

---

## Inspecting a snapshot

`bot.snapshot()` returns the `Snapshot` the getters are reading from. Each
sensor carries its own timestamp, because a reading is almost always older than
the snapshot that latched it:

```python
snap = bot.snapshot()
snap.age()               # seconds since the tick
snap.heading_age()       # seconds since the IMU produced that heading
snap.range_image_age()   # seconds since the freshest lidar slot was written
snap.frame_age()         # seconds since the camera captured that frame
```

A sensor that has never produced a reading reports `inf`, never `0.0`.
