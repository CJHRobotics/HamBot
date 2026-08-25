"""On-demand video source for the Vivid HamBot Console's remote-control view.

One SSH session per viewer, exactly like the control listener::

    ssh hambot@robot hambot-camera

JPEG frames go to stdout, diagnostics to stderr. When the console closes the
session the pipe breaks, this process exits, and the Pi Camera is released --
so a robot nobody is watching leaves the camera free for demos that construct
``HamBot(camera_enabled=True)``. There is no always-on streaming daemon,
because picamera2 can only open the device once and a permanently held camera
would break every student demo on the robot.

This deliberately does not go through ``robot_systems.Camera``. That class
captures a few frames a second into numpy arrays for landmark work; a video
feed wants picamera2's hardware JPEG encoder writing straight into the pipe,
with no per-frame Python in the path.

Wire format -- one record per frame, repeated until the session ends::

    JPEG <byte-count>\\n<byte-count bytes of JPEG>

The header is ASCII and newline-terminated, so a reader can ``readline()`` for
the length and then ``read()`` exactly that many bytes.
"""

from __future__ import annotations

import argparse
import io
import os
import re
import select
import sys
import threading
import time

from hambot_link.locking import CAMERA_LOCK, acquire_lock

STREAM_VERSION = 1
FRAME_MAGIC = b"JPEG"
DEFAULT_RESOLUTION = (640, 480)
DEFAULT_FPS = 15
DEFAULT_QUALITY = 75
POLL_SECONDS = 0.2


def log(message: str) -> None:
    """Write a diagnostic to stderr; stdout carries frames and nothing else."""
    print(f"[hambot-camera] {message}", file=sys.stderr, flush=True)


def fail(message: str) -> None:
    """Report a session-ending problem.

    Marked so the console can tell a real failure from ordinary chatter
    without pattern-matching on wording.
    """
    log(f"ERROR: {message}")


def parse_resolution(text: str) -> tuple[int, int]:
    """Parse a ``WIDTHxHEIGHT`` argument into a size pair."""
    match = re.fullmatch(r"(\d+)x(\d+)", text.strip().lower())
    if not match:
        raise argparse.ArgumentTypeError(f"expected WIDTHxHEIGHT, got {text!r}")
    width, height = int(match.group(1)), int(match.group(2))
    if width < 1 or height < 1:
        raise argparse.ArgumentTypeError("resolution must be positive")
    return width, height


class FrameWriter(io.BufferedIOBase):
    """Length-prefixes each encoded frame onto a binary stream.

    Deliberately a real IO object: picamera2's ``FileOutput`` type-checks what
    it is handed with ``isinstance(file, io.BufferedIOBase)`` and refuses
    anything else, however complete its ``write``/``flush`` pair looks.
    Subclassing also means ``closed`` and ``close()`` come from the standard
    machinery, which the control loop and the hangup watcher both read.

    picamera2 hands one complete JPEG to :meth:`write` per frame. A viewer that
    walks away closes the pipe, which surfaces here as a broken-pipe error --
    that is the session's end-of-life signal, not an error to report.
    """

    def __init__(self, stream):
        super().__init__()
        self._stream = stream
        self._lock = threading.Lock()
        self.frames = 0

    def writable(self) -> bool:
        return True

    def write(self, frame) -> int:
        with self._lock:
            if self.closed:
                return 0
            try:
                self._stream.write(b"%s %d\n" % (FRAME_MAGIC, len(frame)))
                self._stream.write(frame)
                self._stream.flush()
            except (BrokenPipeError, ValueError, OSError):
                # close() only flushes (a no-op here), so this cannot re-enter.
                self.close()
                return 0
            self.frames += 1
            return len(frame)

    def flush(self) -> None:
        """No-op: :meth:`write` already flushes each complete frame."""


def watch_for_hangup(stream, writer: FrameWriter, poll: float = POLL_SECONDS) -> None:
    """Close the session when the console closes its end of the SSH pipe.

    A broken stdout is the usual signal, but a console that shuts down cleanly
    closes stdin first, so watching both ends the session promptly either way.

    Polled on the raw descriptor rather than blocking in readline(): a thread
    parked in a buffered read still holds stdin's lock when the session ends
    some other way, and the interpreter aborts on that lock at shutdown.
    """
    try:
        fd = stream.fileno()
    except Exception:
        fd = None

    if fd is None:
        # No pollable descriptor: fall back to a plain read.
        try:
            while stream.readline():
                pass
        except OSError:
            pass
        writer.close()
        return

    while not writer.closed:
        try:
            ready, _, _ = select.select([fd], [], [], poll)
        except (OSError, ValueError):
            break
        if not ready:
            continue
        try:
            if not os.read(fd, 4096):
                break  # EOF: the console hung up
        except OSError:
            break
    writer.close()


def open_camera(resolution: tuple[int, int], fps: int, rotate_180: bool):
    """Configure the Pi Camera for streaming and return the picamera2 handle."""
    from libcamera import Transform
    from picamera2 import Picamera2

    picam2 = Picamera2()
    frame_us = int(1_000_000 / fps)
    picam2.configure(picam2.create_video_configuration(
        main={"size": resolution},
        controls={"FrameDurationLimits": (frame_us, frame_us)},
        # The camera is mounted upside down on the HamBot, matching
        # PiCamera's rotate_180 default. Doing it in the transform keeps
        # the rotation out of the per-frame path.
        transform=Transform(hflip=1, vflip=1) if rotate_180 else Transform(),
    ))
    return picam2


def stream(picam2, writer: FrameWriter, quality: int) -> None:
    """Record into the writer until the viewer disconnects."""
    from picamera2.encoders import JpegEncoder
    from picamera2.outputs import FileOutput

    picam2.start_recording(JpegEncoder(q=quality), FileOutput(writer))
    try:
        while not writer.closed:
            time.sleep(POLL_SECONDS)
    finally:
        picam2.stop_recording()


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        prog="hambot-camera",
        description="Stream JPEG frames to stdout for the Vivid HamBot Console.",
    )
    parser.add_argument("--resolution", type=parse_resolution,
                        default=DEFAULT_RESOLUTION, metavar="WxH",
                        help="Capture size (default: %dx%d)." % DEFAULT_RESOLUTION)
    parser.add_argument("--fps", type=int, default=DEFAULT_FPS,
                        help=f"Target frame rate (default: {DEFAULT_FPS}).")
    parser.add_argument("--quality", type=int, default=DEFAULT_QUALITY,
                        help=f"JPEG quality, 1-100 (default: {DEFAULT_QUALITY}).")
    parser.add_argument("--no-rotate", action="store_true",
                        help="Do not rotate 180 degrees for the inverted mount.")
    args = parser.parse_args(argv)

    if not 1 <= args.fps <= 60:
        parser.error("--fps must be between 1 and 60")
    if not 1 <= args.quality <= 100:
        parser.error("--quality must be between 1 and 100")

    lock_fd = acquire_lock(CAMERA_LOCK)
    if lock_fd is None:
        fail("another session is already streaming this camera")
        return 1

    width, height = args.resolution
    log(f"stream v{STREAM_VERSION}, {width}x{height} @ {args.fps} fps, "
        f"quality {args.quality}")

    writer = FrameWriter(sys.stdout.buffer)
    try:
        picam2 = open_camera(args.resolution, args.fps, not args.no_rotate)
    except Exception as exc:
        fail(f"could not open the camera: {exc}")
        log("a demo may already be holding it")
        os.close(lock_fd)
        return 1

    hangup = threading.Thread(target=watch_for_hangup, args=(sys.stdin.buffer, writer),
                              name="hambot-camera-hangup", daemon=True)
    hangup.start()
    try:
        stream(picam2, writer, args.quality)
    except Exception as exc:
        fail(f"streaming failed: {exc}")
        return 1
    finally:
        # Release the hangup thread before the interpreter tears down stdin.
        writer.close()
        hangup.join(timeout=2 * POLL_SECONDS)
        picam2.close()
        os.close(lock_fd)
    log(f"viewer disconnected after {writer.frames} frames, camera released")
    return 0


if __name__ == "__main__":
    sys.exit(main())
