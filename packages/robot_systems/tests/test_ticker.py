import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
sys.path.insert(0, str(Path(__file__).resolve().parent))

from fake_hardware import FakeClock

from robot_systems.ticker import Snapshot, Ticker


class TickerPacingTests(unittest.TestCase):
    """The fake clock is what makes any of this assertable off-hardware."""

    def setUp(self):
        self.clock = FakeClock()
        self.ticker = Ticker(0.05, clock=self.clock.now, sleeper=self.clock.sleep)

    def test_rejects_a_non_positive_period(self):
        for bad in (0, -0.01):
            with self.assertRaises(ValueError):
                Ticker(bad)

    def test_waits_the_remainder_of_the_period(self):
        self.clock.advance(0.02)          # loop body took 20 ms
        slept = self.ticker.wait()
        self.assertAlmostEqual(slept, 0.03)
        self.assertEqual(self.ticker.ticks, 1)
        self.assertEqual(self.ticker.overruns, 0)

    def test_an_idle_loop_does_not_drift(self):
        for _ in range(10):
            self.ticker.wait()
        # Ten 50 ms ticks, anchored from construction time, land exactly.
        self.assertAlmostEqual(self.clock.now(), 1000.0 + 0.5)
        self.assertEqual(self.ticker.overruns, 0)

    def test_landing_exactly_on_the_boundary_is_not_an_overrun(self):
        self.clock.advance(0.05)
        slept = self.ticker.wait()
        self.assertEqual(slept, 0.0)
        self.assertEqual(self.ticker.overruns, 0)
        self.assertTrue(self.ticker.is_keeping_up())

    def test_overrun_is_counted_and_measured(self):
        self.clock.advance(0.08)          # 30 ms late
        slept = self.ticker.wait()
        self.assertEqual(slept, 0.0)
        self.assertEqual(self.ticker.overruns, 1)
        self.assertAlmostEqual(self.ticker.last_overrun_s, 0.03)
        self.assertAlmostEqual(self.ticker.worst_overrun_s, 0.03)
        self.assertFalse(self.ticker.is_keeping_up())

    def test_worst_overrun_keeps_the_maximum_and_last_keeps_the_latest(self):
        self.clock.advance(0.25)          # 200 ms late
        self.ticker.wait()
        self.clock.advance(0.06)          # 10 ms late
        self.ticker.wait()
        self.assertEqual(self.ticker.overruns, 2)
        self.assertAlmostEqual(self.ticker.last_overrun_s, 0.01)
        self.assertAlmostEqual(self.ticker.worst_overrun_s, 0.20)

    def test_keeping_up_recovers_after_a_good_tick(self):
        self.clock.advance(0.08)
        self.ticker.wait()
        self.assertFalse(self.ticker.is_keeping_up())
        self.ticker.wait()                # next body is instant
        self.assertTrue(self.ticker.is_keeping_up())

    def test_reset_reanchors_and_can_change_the_period(self):
        self.clock.advance(0.04)
        self.ticker.reset(period=0.10)
        self.assertEqual(self.ticker.period, 0.10)
        slept = self.ticker.wait()
        self.assertAlmostEqual(slept, 0.10)   # a full new period from the reset

    def test_reset_keeps_the_overrun_history(self):
        self.clock.advance(0.09)
        self.ticker.wait()
        self.ticker.reset(period=0.2)
        self.assertEqual(self.ticker.overruns, 1)
        self.assertAlmostEqual(self.ticker.worst_overrun_s, 0.04)

    def test_reset_rejects_a_non_positive_period(self):
        with self.assertRaises(ValueError):
            self.ticker.reset(period=0)


class SlipNotSkipTests(unittest.TestCase):
    """The overrun policy is the semantic content of step(); pin it."""

    def setUp(self):
        self.clock = FakeClock()
        self.ticker = Ticker(0.05, clock=self.clock.now, sleeper=self.clock.sleep)

    def test_a_late_tick_slips_a_full_period_from_now(self):
        self.clock.advance(0.12)          # body overran by 70 ms
        self.ticker.wait()
        at_overrun = self.clock.now()
        # Next boundary is one period from *now*, not from the missed one, so
        # a subsequent instant body still waits a full period.
        slept = self.ticker.wait()
        self.assertAlmostEqual(slept, 0.05)
        self.assertAlmostEqual(self.clock.now(), at_overrun + 0.05)

    def test_slipping_never_free_runs_to_catch_up(self):
        # One very long body. Skipping would burn several zero-length waits
        # racing to the schedule; slipping must not.
        self.clock.advance(0.5)           # ten periods late
        self.ticker.wait()
        self.assertEqual(self.ticker.overruns, 1)
        for _ in range(3):
            self.assertAlmostEqual(self.ticker.wait(), 0.05)
        self.assertEqual(self.ticker.overruns, 1)

    def test_no_cycle_is_ever_dropped(self):
        for _ in range(5):
            self.clock.advance(0.2)       # every body overruns badly
            self.ticker.wait()
        self.assertEqual(self.ticker.ticks, 5)
        self.assertEqual(self.ticker.overruns, 5)

    def test_elapsed_time_is_not_ticks_times_period_after_a_miss(self):
        # This is the documented cost of slipping, so assert it rather than
        # let someone later "fix" it.
        start = self.clock.now()
        self.clock.advance(0.3)
        self.ticker.wait()
        self.ticker.wait()
        self.assertEqual(self.ticker.ticks, 2)
        elapsed = self.clock.now() - start
        self.assertGreater(elapsed, 2 * 0.05)
        self.assertAlmostEqual(elapsed, 0.35)


class SnapshotTests(unittest.TestCase):

    def test_an_unread_sensor_reports_inf_not_zero(self):
        snap = Snapshot()
        self.assertEqual(snap.age(), float("inf"))
        self.assertEqual(snap.heading_age(), float("inf"))
        self.assertEqual(snap.range_image_age(), float("inf"))
        self.assertEqual(snap.frame_age(), float("inf"))

    def test_ages_are_measured_from_each_sensors_own_stamp(self):
        # A frame is older than the snapshot that latched it, which is exactly
        # why the per-sensor stamps exist.
        snap = Snapshot(t=100.0, heading=90.0, heading_t=99.95,
                        frame=object(), frame_t=99.8)
        self.assertAlmostEqual(snap.age(now=100.0), 0.0)
        self.assertAlmostEqual(snap.heading_age(now=100.0), 0.05)
        self.assertAlmostEqual(snap.frame_age(now=100.0), 0.20)

    def test_age_never_goes_negative(self):
        snap = Snapshot(t=100.0)
        self.assertEqual(snap.age(now=99.0), 0.0)

    def test_encoders_default_to_an_empty_dict_per_instance(self):
        a, b = Snapshot(), Snapshot()
        a.encoders["left"] = 1.0
        self.assertEqual(b.encoders, {})


if __name__ == "__main__":
    unittest.main()
