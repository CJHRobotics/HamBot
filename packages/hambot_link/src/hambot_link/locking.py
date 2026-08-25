"""Single-session locks shared by the console-facing entry points.

Each console session -- driving or watching -- is one process holding one
exclusive lock, so a second operator gets a clear refusal instead of two
sessions fighting over the same device.
"""

from __future__ import annotations

import fcntl
import os
import tempfile

CONTROL_LOCK = "/run/lock/hambot-link.lock"
CAMERA_LOCK = "/run/lock/hambot-camera.lock"


def acquire_lock(path: str) -> int | None:
    """Take an exclusive, non-blocking lock, or return None if it is held.

    Falls back to the temp directory when the preferred lock directory is not
    writable, so a session is never blocked by a read-only ``/run/lock``.
    """
    try:
        fd = os.open(path, os.O_CREAT | os.O_RDWR, 0o666)
    except OSError:
        fallback = os.path.join(tempfile.gettempdir(), os.path.basename(path))
        fd = os.open(fallback, os.O_CREAT | os.O_RDWR, 0o666)
    try:
        fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except OSError:
        os.close(fd)
        return None
    return fd
