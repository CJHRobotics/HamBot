"""Stub hardware modules so robot_systems can be imported off a robot.

A stock robot has no pytest and no numpy, and a laptop has no Build HAT, no
BNO055, and no RPLidar. Everything here is installed into ``sys.modules``
before ``robot_systems`` is imported, so the real modules are never needed.

Call ``install()`` at the top of a test module, before importing anything from
``robot_systems``.
"""

import sys
import time
import types


class FakeMotor:
    """A Build HAT Motor that records what it was told to do.

    Records every call in ``calls`` so a test can assert not just the final
    state but the order things happened in — which is how "was the stop undone
    by a later flush?" gets checked.
    """

    def __init__(self, port):
        self.port = port
        self.calls = []
        self.speed = None            # None when stopped
        self.rpm_units = False
        self.position = 0.0

    def set_speed_unit_rpm(self, rpm=True):
        self.rpm_units = bool(rpm)

    def start(self, speed=0):
        self.speed = speed
        self.calls.append(("start", speed))

    def stop(self):
        self.speed = None
        self.calls.append(("stop",))

    def run_for_seconds(self, seconds, speed=0, blocking=True):
        self.calls.append(("run_for_seconds", seconds, speed, blocking))

    def run_for_rotations(self, rotations, speed=0, blocking=True):
        self.calls.append(("run_for_rotations", rotations, speed, blocking))

    def run_to_position(self, position, speed=0, blocking=True):
        self.calls.append(("run_to_position", position, speed, blocking))

    def get_position(self):
        return self.position


class FakeBNO055:
    """A BNO055 that always reports a fixed, fully calibrated orientation."""

    def __init__(self, i2c=None):
        self.i2c = i2c
        # (heading from North, roll, pitch) in degrees, as the real driver.
        self.euler = (30.0, 0.0, 0.0)
        self.calibration_status = (3, 3, 3, 3)


class FakeRPLidarException(Exception):
    pass


class FakeRPLidar:
    """An RPLidar that produces no scans but does not spin the scan thread hot.

    Tests that care about slot expiry write ``scan_data``/``scan_time``
    directly, which is deterministic; letting this yield real scans would make
    those tests depend on thread scheduling.
    """

    def __init__(self, dmesg, port_name, timeout=3):
        self.port_name = port_name
        self.timeout = timeout
        self.motor_running = False
        self.disconnected = False
        self.stopped = False

    def start_motor(self):
        self.motor_running = True

    def iter_scans(self):
        # A generator that idles briefly and ends. Lidar._scan re-enters it
        # while running, so the sleep is what keeps that from busy-looping.
        time.sleep(0.01)
        return
        yield  # pragma: no cover - makes this a generator

    def stop(self):
        self.stopped = True

    def stop_motor(self):
        self.motor_running = False

    def disconnect(self):
        self.disconnected = True


def _fake_numpy():
    """Enough numpy for camera.py to import and construct."""
    np = types.ModuleType("numpy")

    def clip(value, lo, hi):
        return max(lo, min(hi, value))

    np.clip = clip
    np.uint8 = int
    np.zeros = lambda *a, **k: []
    np.array = lambda *a, **k: list(a[0]) if a else []
    return np


def install():
    """Install the stub modules. Idempotent; safe to call from every test."""
    if "buildhat" in sys.modules and getattr(
            sys.modules["buildhat"], "_hambot_fake", False):
        return

    buildhat = types.ModuleType("buildhat")
    buildhat.Motor = FakeMotor
    buildhat._hambot_fake = True

    board = types.ModuleType("board")
    board.I2C = lambda: object()

    bno = types.ModuleType("adafruit_bno055")
    bno.BNO055_I2C = FakeBNO055

    rplidar = types.ModuleType("adafruit_rplidar")
    rplidar.RPLidar = FakeRPLidar
    rplidar.RPLidarException = FakeRPLidarException

    # camera.py imports both at module scope; only numpy is actually called
    # during import and construction, and cv2 only inside find_landmarks.
    cv2 = types.ModuleType("cv2")

    sys.modules.update({
        "buildhat": buildhat,
        "board": board,
        "adafruit_bno055": bno,
        "adafruit_rplidar": rplidar,
        "numpy": _fake_numpy(),
        "cv2": cv2,
    })


class FakeClock:
    """A hand-advanced monotonic clock, plus a sleeper that advances it.

    Pass ``now`` as a Ticker's clock and ``sleep`` as its sleeper and the
    ticker's pacing becomes fully deterministic: ``sleep`` records what it was
    asked for and moves time forward by exactly that much, while ``advance``
    simulates a loop body that took too long.
    """

    def __init__(self, start=1000.0):
        self.t = float(start)
        self.slept = []

    def now(self):
        return self.t

    def sleep(self, seconds):
        self.slept.append(seconds)
        self.t += seconds

    def advance(self, seconds):
        """Move time forward without sleeping — i.e. work the loop body did."""
        self.t += seconds
