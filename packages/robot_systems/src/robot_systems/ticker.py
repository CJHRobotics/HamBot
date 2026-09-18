"""Loop pacing and coherent sensor snapshots for the HamBot control tick.

This module deliberately imports no hardware. The timing rules in ``Ticker``
and the freshness rules in ``Snapshot`` are the semantic content of
``HamBot.step()``, so they need to be testable off-robot with a fake clock
rather than only on a Build HAT.
"""

import time
from dataclasses import dataclass, field
from typing import Any, Callable, Dict, List, Optional, Tuple


class Ticker:
    """Paces a loop to a fixed period and records how badly it is missed.

    Overrun policy is **slip, not skip**. When the loop body runs long, the
    next boundary is placed one full period from *now* rather than from the
    boundary that was missed. No cycle is ever dropped, and the loop never
    free-runs through a burst of zero-length waits trying to catch up.

    The cost of slipping is that wall-clock time is no longer ``ticks *
    period`` once an overrun has happened. Code that needs elapsed time must
    read a clock rather than count ticks.
    """

    def __init__(self,
                 period: float,
                 clock: Callable[[], float] = time.monotonic,
                 sleeper: Callable[[float], Any] = time.sleep):
        """Create a ticker anchored one period from now.

        Args:
            period (float): Seconds between boundaries. Must be positive.
            clock (callable): Returns a monotonically increasing time in
                              seconds. Injectable so tests can drive time.
            sleeper (callable): Blocks for the given number of seconds.
                                Injectable for the same reason.

        Raises:
            ValueError: If period is not positive.
        """
        if period <= 0:
            raise ValueError(f"period must be positive, got {period!r}")

        self._clock = clock
        self._sleeper = sleeper

        self.period = float(period)
        self.ticks = 0
        self.overruns = 0
        self.last_overrun_s = 0.0
        self.worst_overrun_s = 0.0
        self.last_tick_late = False

        self._next = self._clock() + self.period

    def reset(self, period: Optional[float] = None) -> None:
        """Re-anchor the schedule to one period from now.

        Counters are left alone — a student changing ``dt`` mid-run should not
        lose the record of how far behind their loop has been.

        Args:
            period (float | None): New period in seconds, or None to keep the
                                   current one.

        Raises:
            ValueError: If period is given and is not positive.
        """
        if period is not None:
            if period <= 0:
                raise ValueError(f"period must be positive, got {period!r}")
            self.period = float(period)
        self._next = self._clock() + self.period

    def wait(self) -> float:
        """Block until the next period boundary.

        Returns:
            float: Seconds actually slept. 0.0 means the boundary had already
                   arrived or passed, so the caller was not paced at all.
        """
        self.ticks += 1
        remaining = self._next - self._clock()

        if remaining >= 0:
            # On time. Advance the anchor by exactly one period so an
            # on-schedule loop does not accumulate drift.
            if remaining > 0:
                self._sleeper(remaining)
            self._next += self.period
            self.last_tick_late = False
            return remaining

        # Late. Record it, then slip: the next boundary is one full period
        # from now, not from the boundary we just blew through.
        overrun = -remaining
        self.overruns += 1
        self.last_overrun_s = overrun
        if overrun > self.worst_overrun_s:
            self.worst_overrun_s = overrun
        self.last_tick_late = True
        self._next = self._clock() + self.period
        return 0.0

    def is_keeping_up(self) -> bool:
        """bool: True if the most recent tick met its boundary."""
        return not self.last_tick_late


def _age_of(stamp: Optional[float], now: Optional[float]) -> float:
    """Seconds between ``stamp`` and ``now``, or inf if never stamped."""
    if stamp is None:
        return float("inf")
    if now is None:
        now = time.monotonic()
    return max(0.0, now - stamp)


@dataclass
class Snapshot:
    """One coherent read of every sensor, latched at a single instant.

    ``t`` is when the snapshot was taken. Each sensor also carries its own
    ``*_t``, because a reading is almost always older than the snapshot that
    latched it — the IMU poller runs at 20 Hz and the camera at 5 fps, so a
    frame latched at ``t`` may have been captured 200 ms earlier. Reporting
    only ``t`` would overstate freshness.

    A sensor that has never produced a reading has a ``*_t`` of None and
    reports an age of ``inf``, never 0.0.
    """

    t: Optional[float] = None
    heading: Optional[float] = None
    heading_t: Optional[float] = None
    range_image: Optional[List[float]] = None
    range_image_t: Optional[float] = None
    encoders: Dict[str, float] = field(default_factory=dict)
    frame: Any = None
    frame_t: Optional[float] = None

    def age(self, now: Optional[float] = None) -> float:
        """float: Seconds since this snapshot was taken (inf if never)."""
        return _age_of(self.t, now)

    def heading_age(self, now: Optional[float] = None) -> float:
        """float: Seconds since the IMU produced this heading (inf if never)."""
        return _age_of(self.heading_t, now)

    def range_image_age(self, now: Optional[float] = None) -> float:
        """float: Seconds since the freshest lidar slot was written (inf if never).

        The scan is a mosaic of per-slot write times, so this is the age of the
        newest slot in it. Per-slot ages come from ``Lidar.slot_age()``.
        """
        return _age_of(self.range_image_t, now)

    def frame_age(self, now: Optional[float] = None) -> float:
        """float: Seconds since the camera captured this frame (inf if never)."""
        return _age_of(self.frame_t, now)
