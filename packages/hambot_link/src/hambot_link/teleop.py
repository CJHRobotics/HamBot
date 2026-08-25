"""Robot-side listener for the Vivid HamBot Console's remote-control view.

The console opens one persistent SSH session per driving session and writes
newline-delimited commands to this process's stdin::

    <seq> <left_rpm> <right_rpm>\\n

``seq`` is a non-negative integer that increases with every command; anything
not newer than the last accepted command is discarded, so a delayed packet can
never resurrect a speed the operator has already left behind. The console does
all of the joystick mixing, so the wire carries nothing but two motor targets.

One SSH invocation is one session: this process takes an exclusive lock, builds
a ``HamBot``, drives it, and releases everything when stdin closes. Nothing is
written back to the console -- stdout stays empty and diagnostics go to stderr,
where SSH forwards them to the operator's terminal.

Two rules keep a dropped link from becoming a runaway robot:

* a watchdog stops the motors when no command has arrived recently, so the
  robot coasts to a halt on a Wi-Fi drop instead of holding its last speed, and
* the motor targets are re-clamped here, because the console's own limit is not
  something this side can verify.
"""

from __future__ import annotations

import argparse
import os
import sys
import threading
import time

from hambot_link.locking import CONTROL_LOCK, acquire_lock

PROTOCOL_VERSION = 1
CONTROL_HZ = 20.0
WATCHDOG_SECONDS = 0.3
DEFAULT_SPEED_LIMIT = 75
DRIVE_2WD = "2WD"
DRIVE_4WD = "4WD"


def log(message: str) -> None:
    """Write a diagnostic to stderr; stdout is reserved for saying nothing."""
    print(f"[hambot-link] {message}", file=sys.stderr, flush=True)


def fail(message: str) -> None:
    """Report a session-ending problem.

    Marked so the console can tell a real failure from ordinary chatter
    without pattern-matching on wording.
    """
    log(f"ERROR: {message}")


def clamp(rpm: int, limit: int) -> int:
    return max(-limit, min(limit, rpm))


def parse_command(line: str, limit: int) -> tuple[int, int, int] | None:
    """Parse one ``<seq> <left> <right>`` line into clamped motor targets."""
    parts = line.split()
    if len(parts) != 3:
        return None
    try:
        seq, left, right = (int(part) for part in parts)
    except ValueError:
        return None
    if seq < 0:
        return None
    return seq, clamp(left, limit), clamp(right, limit)


class CommandSlot:
    """The newest accepted command, overwritten in place.

    Commands are coalesced rather than queued: a backlog of stale joystick
    samples is worse than useless, so the control loop only ever sees the most
    recent target.
    """

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._seq = -1
        self._left = 0
        self._right = 0
        self._at = 0.0
        self._closed = False

    def offer(self, seq: int, left: int, right: int) -> bool:
        """Store a command if it is newer than the last one."""
        with self._lock:
            if seq <= self._seq:
                return False
            self._seq, self._left, self._right = seq, left, right
            self._at = time.monotonic()
            return True

    def read(self) -> tuple[int, int, float]:
        with self._lock:
            return self._left, self._right, self._at

    def close(self) -> None:
        with self._lock:
            self._closed = True

    @property
    def closed(self) -> bool:
        with self._lock:
            return self._closed


def read_commands(slot: CommandSlot, limit: int, stream=None) -> None:
    """Feed parsed commands into the slot until the stream ends."""
    stream = sys.stdin if stream is None else stream
    try:
        while True:
            line = stream.readline()
            if not line:
                break
            command = parse_command(line, limit)
            if command is None:
                log(f"ignoring malformed command: {line.strip()!r}")
                continue
            slot.offer(*command)
    finally:
        slot.close()


def next_target(slot: CommandSlot, watchdog: float,
                now: float) -> tuple[tuple[int, int], bool]:
    """Decide one control tick's motor target.

    Returns the ``(left, right)`` target and whether the watchdog has expired.
    A command older than ``watchdog`` yields a stop: on a dropped link the
    robot must coast to a halt rather than hold its last speed.
    """
    left, right, at = slot.read()
    if (now - at) > watchdog:
        return (0, 0), True
    return (left, right), False


def drive(bot, slot: CommandSlot, hz: float = CONTROL_HZ,
          watchdog: float = WATCHDOG_SECONDS) -> None:
    """Apply the newest target at a fixed cadence until the link closes.

    Motor commands are only sent when the target changes, which keeps a held
    joystick from saturating the Build HAT's serial port at ``hz``.
    """
    period = 1.0 / hz
    applied: tuple[int, int] | None = None
    was_stale = False

    while not slot.closed:
        target, stale = next_target(slot, watchdog, time.monotonic())
        if stale and not was_stale and applied not in (None, (0, 0)):
            log(f"watchdog: no command in {watchdog * 1000:.0f} ms, stopping")
        was_stale = stale

        if target != applied:
            bot.set_left_motor_speed(target[0])
            bot.set_right_motor_speed(target[1])
            applied = target
        time.sleep(period)

    log("control link closed, stopping motors")
    bot.stop_motors()


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        prog="hambot-link",
        description="Drive the HamBot from commands on stdin (see module docstring).",
    )
    parser.add_argument("--limit", type=int, default=DEFAULT_SPEED_LIMIT,
                        help=f"Motor speed ceiling in RPM (default: {DEFAULT_SPEED_LIMIT}).")
    parser.add_argument("--drivetrain", default=DRIVE_2WD,
                        choices=[DRIVE_2WD, DRIVE_4WD],
                        help="Drivetrain to initialize (default: 2WD).")
    parser.add_argument("--watchdog", type=float, default=WATCHDOG_SECONDS,
                        help=f"Seconds without a command before stopping (default: {WATCHDOG_SECONDS}).")
    args = parser.parse_args(argv)

    if args.limit < 1:
        parser.error("--limit must be at least 1 RPM")
    if args.watchdog <= 0:
        parser.error("--watchdog must be greater than 0 seconds")

    lock_fd = acquire_lock(CONTROL_LOCK)
    if lock_fd is None:
        fail("another control session is already driving this robot")
        return 1

    log(f"protocol v{PROTOCOL_VERSION}, limit {args.limit} RPM, "
        f"watchdog {args.watchdog * 1000:.0f} ms")
    # Imported here, not at module scope, so the protocol logic above stays
    # importable (and testable) on a machine with no Build HAT attached.
    from robot_systems.robot import HamBot

    try:
        bot = HamBot(drivetrain=args.drivetrain)
    except Exception as exc:
        fail(f"could not attach to the robot: {exc}")
        log("a demo may already be holding the Build HAT")
        os.close(lock_fd)
        return 1

    slot = CommandSlot()
    reader = threading.Thread(target=read_commands, args=(slot, args.limit),
                              name="hambot-link-reader", daemon=True)
    reader.start()
    try:
        drive(bot, slot, watchdog=args.watchdog)
    finally:
        bot.disconnect_robot()
        os.close(lock_fd)
    return 0


if __name__ == "__main__":
    sys.exit(main())
