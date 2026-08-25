# hambot_link

Robot-side listener and video source for the
[Vivid HamBot Console](https://github.com/ChanceHamilton59/vivid-hambot-console).

The console's remote-control view opens **one SSH session per driving session**
and streams motor targets to `hambot-link`'s stdin, plus **one SSH session per
viewing session** that reads JPEG frames from `hambot-camera`'s stdout. There is
no daemon and no open port: SSH is both the transport and the authentication,
and the robot's hardware is only held while someone is actually using it.

| Entry point | Direction | Holds |
| --- | --- | --- |
| `hambot-link` | console → robot, on stdin | Build HAT motors |
| `hambot-camera` | robot → console, on stdout | Pi Camera |

The two are independent: you can watch without driving, or drive without
watching, and each takes its own lock.

```
console                                   robot
┌────────────────────┐                    ┌──────────────────────────┐
│ joystick / arrows  │                    │  hambot-link             │
│        ↓           │   ssh (stdin)      │    reader thread → slot  │
│ _set_motor_speeds  │ ─────────────────► │    control loop  → HamBot│
│        ↓           │  20 Hz, one line   │    watchdog      → stop  │
│ seq left right     │                    └──────────────────────────┘
└────────────────────┘
```

## Wire protocol (v1)

One command per line, ASCII, newline-terminated:

```
<seq> <left_rpm> <right_rpm>\n
```

| Field | Meaning |
| --- | --- |
| `seq` | Non-negative integer, strictly increasing within a session. Commands not newer than the last accepted one are discarded. |
| `left_rpm` | Left motor target in RPM. Re-clamped to `±--limit` on arrival. |
| `right_rpm` | Right motor target in RPM. Re-clamped the same way. |

Nothing is sent back. Stdout stays empty; diagnostics go to stderr, which SSH
forwards to the operator's terminal.

Malformed lines are logged and skipped — they never stop the session, because
dropping the link is more dangerous than ignoring one bad line.

### Sender requirements

The console must **send at a steady ~20 Hz, not only when the target changes.**
The listener's watchdog treats silence as a fault, so a sender that goes quiet
while holding the stick will be stopped. Repeating the current target is the
keepalive.

Sequence numbers are what make a delayed line harmless: a stale `(75, 75)`
arriving after a `(0, 0)` is dropped rather than re-accelerating the robot.

## Camera stream (v1)

`hambot-camera` writes one record per frame to stdout:

```
JPEG <byte-count>\n<byte-count bytes of JPEG>
```

The header is ASCII and newline-terminated, so a reader does `readline()` for
the length and then `read()` exactly that many bytes. The length prefix is what
makes this safe — JPEG payloads contain `0x0A` regularly, so a newline-delimited
format would corrupt frames.

```bash
hambot-camera [--resolution WxH] [--fps N] [--quality 1-100] [--no-rotate]
```

Defaults: `640x480`, 15 fps, quality 75, rotated 180° for the HamBot's inverted
camera mount (matching `PiCamera`'s `rotate_180` default).

This does **not** go through `robot_systems.Camera`. That class captures a few
frames a second into numpy arrays for landmark work; the stream uses picamera2's
hardware JPEG encoder writing straight into the pipe, with no per-frame Python.

> **The console must keep stdin open for the life of the session.** Stdin EOF is
> read as a hangup, so `ssh -n robot hambot-camera` (or any redirect from
> `/dev/null`) exits immediately. That check is what stops a stalled camera from
> holding the device forever when no frames are flowing to break the pipe.

## Safety behaviour

| Situation | Result |
| --- | --- |
| No command for `--watchdog` seconds (default 300 ms) | Motors stop. A Wi-Fi drop coasts the robot to a halt instead of holding its last speed. |
| Stdin reaches EOF (operator leaves the view, SSH dies) | Motors stop, `disconnect_robot()` runs, process exits. |
| `SIGINT` / `SIGTERM` | `robot_systems` stops the motors and disconnects before exiting. |
| A second session tries to connect | Refused. An exclusive `flock` means one driver at a time. |
| Speeds beyond `--limit` on the wire | Clamped here. The console's own limit is not something this side can verify. |
| Viewer disconnects | The broken pipe (or stdin EOF) stops recording and releases the camera, so demos can open it again. |

## Usage

```bash
hambot-link [--limit RPM] [--drivetrain 2WD|4WD] [--watchdog SECONDS]
```

Defaults: `--limit 75`, `--drivetrain 2WD`, `--watchdog 0.3`.

Drive it by hand to check a robot without the console:

```bash
printf '1 30 30\n2 30 30\n3 0 0\n' | hambot-link
```

Grab a few seconds of video the same way:

```bash
ssh hambot@hambot-05 hambot-camera > /tmp/frames.bin
```

## Install

```bash
./deploy/setup_link.sh
```

or fold it into a full install with `./deploy/install.sh --link`. The script
installs the package into the shared `hambot_venv` and symlinks both entry
points into `/usr/local/bin`, because `ssh robot hambot-link` runs a non-login
shell where the venv is never activated.

The console's public key must be in the `hambot` user's `~/.ssh/authorized_keys`.

## Tests

```bash
python3 -m unittest discover -s packages/hambot_link/tests -v
```

The protocol logic imports without a Build HAT or camera attached —
`robot_systems`, `picamera2`, and `libcamera` are all imported lazily inside
functions — so the parser, the command slot, the watchdog, the control loop, and
the frame framing all run on a laptop.
