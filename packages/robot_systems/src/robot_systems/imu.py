# imu.py
import time
import threading
from typing import Optional, Tuple

import board
import adafruit_bno055


class IMU:
    def __init__(self, poll_hz: float = 20.0, warmup_s: float = 0.5):
        """
        poll_hz: how often to sample the IMU in the background
        warmup_s: how long to wait before serving cached values (gives sensor time to wake)
        """
        self.i2c = board.I2C()
        self.sensor = adafruit_bno055.BNO055_I2C(self.i2c)

        # Cache & sync
        self._heading_deg_from_east: Optional[float] = None
        self._last_update_s: float = 0.0
        self._lock = threading.Lock()
        self._cv = threading.Condition(self._lock)

        # Thread control
        self._stop_evt = threading.Event()
        self._poll_period = 1.0 / max(1e-6, poll_hz)
        self._warmup_until = time.time() + max(0.0, warmup_s)
        self._thread: Optional[threading.Thread] = None

    # ---------- Thread lifecycle ----------
    def start(self):
        """Start the background polling thread (idempotent)."""
        if self._thread and self._thread.is_alive():
            return
        self._stop_evt.clear()
        self._thread = threading.Thread(target=self._poll_loop, name="IMU-Poller", daemon=True)
        self._thread.start()

    def stop(self, join_timeout: float = 1.0):
        """Stop the background polling thread cleanly."""
        self._stop_evt.set()
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=join_timeout)

    # ---------- Public getters ----------
    def get_heading(self,
                    fresh_within: float = 0.5,
                    blocking: bool = False,
                    wait_timeout: float = 0.3) -> Optional[float]:
        """
        Return heading in degrees from East (0–360), using cached value.

        fresh_within: seconds; require data newer than now - fresh_within
        blocking: if True, wait up to wait_timeout for a fresh sample
        wait_timeout: max time to wait for freshness if blocking=True

        Returns None if no valid sample available (or freshness not met when non-blocking).
        """
        deadline = time.time() + max(0.0, wait_timeout)
        must_be_newer_than = time.time() - max(0.0, fresh_within)

        with self._cv:
            while True:
                if self._heading_deg_from_east is not None and self._last_update_s >= must_be_newer_than:
                    return self._heading_deg_from_east
                if not blocking:
                    return None
                remaining = deadline - time.time()
                if remaining <= 0:
                    return None
                self._cv.wait(timeout=remaining)

    def get_heading_cached(self) -> Optional[float]:
        """
        Return the last heading regardless of freshness (may be stale or None).
        """
        with self._lock:
            return self._heading_deg_from_east

    def last_update_age(self) -> float:
        """Seconds since the cache was updated (inf if never)."""
        with self._lock:
            if self._last_update_s == 0.0:
                return float("inf")
            return max(0.0, time.time() - self._last_update_s)

    # ---------- Optional helpers ----------
    def is_calibrated(self) -> bool:
        """True when (sys, gyro, accel, mag) are all 3."""
        status = self.sensor.calibration_status  # may be None or a 4-tuple
        if not status or any(s is None for s in status):
            return False
        return all(s == 3 for s in status)

    # ---------- Internal polling ----------
    def _poll_loop(self):
        next_t = time.time()
        while not self._stop_evt.is_set():
            # read euler; it can be None or contain None values early on
            euler = self.sensor.euler
            if euler is not None and all(v is not None for v in euler):
                # TODO :verify it works
                heading_from_north, _, _ = euler  # BNO055 returns (heading=yaw from North, roll, pitch) in degrees
                # Convert to "degrees from East" to match your convention
                heading_from_east = (-heading_from_north + 90.0) % 360.0

                with self._cv:
                    self._heading_deg_from_east = heading_from_east
                    self._last_update_s = time.time()
                    # Only start signaling once past warmup; avoids handing out very early junk
                    if self._last_update_s >= self._warmup_until:
                        self._cv.notify_all()

            # Sleep to maintain poll rate (skip drift)
            next_t += self._poll_period
            sleep_s = next_t - time.time()
            if sleep_s > 0:
                self._stop_evt.wait(timeout=sleep_s)
            else:
                # If we fell behind, reset schedule
                next_t = time.time()

    def __enter__(self):
        self.start()
        return self

    def __exit__(self, exc_type, exc, tb):
        self.stop()
