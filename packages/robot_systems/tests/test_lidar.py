import sys
import time
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
sys.path.insert(0, str(Path(__file__).resolve().parent))

import fake_hardware

fake_hardware.install()

from robot_systems.lidar import Lidar


class LidarSlotExpiryTests(unittest.TestCase):
    """Stale slots must read as INVALID_READING, not as a valid distance.

    Slots are written directly rather than fed through the scan thread: expiry
    is a property of the timestamps, and driving it through a thread would make
    these assertions depend on scheduling.
    """

    def setUp(self):
        self.lidar = Lidar(port_name="/dev/null")
        self.addCleanup(self.lidar.stop_lidar)

    def _write(self, degree, distance, age_s):
        """Write a slot as if it had been measured age_s seconds ago."""
        self.lidar.scan_data[degree] = distance
        self.lidar.scan_time[degree] = time.monotonic() - age_s

    def test_a_never_written_slot_is_invalid(self):
        scan = self.lidar.get_current_scan()
        self.assertEqual(scan[0], Lidar.INVALID_READING)
        self.assertEqual(self.lidar.slot_age(0), float("inf"))

    def test_a_fresh_slot_survives(self):
        self._write(90, 1234, age_s=0.0)
        self.assertEqual(self.lidar.get_current_scan()[90], 1234)

    def test_a_stale_slot_is_expired(self):
        # The exact failure the redesign is for: a reading from 30 s ago was
        # indistinguishable from a fresh one.
        self._write(180, 500, age_s=30.0)
        self.assertEqual(self.lidar.get_current_scan()[180],
                         Lidar.INVALID_READING)

    def test_expiry_does_not_touch_the_stored_array(self):
        self._write(180, 500, age_s=30.0)
        self.lidar.get_current_scan()
        self.assertEqual(self.lidar.scan_data[180], 500)

    def test_fresh_and_stale_slots_are_distinguished_in_one_scan(self):
        self._write(10, 800, age_s=0.05)
        self._write(20, 900, age_s=5.0)
        scan = self.lidar.get_current_scan()
        self.assertEqual(scan[10], 800)
        self.assertEqual(scan[20], Lidar.INVALID_READING)

    def test_the_window_is_the_constructor_default(self):
        lidar = Lidar(port_name="/dev/null", stale_after_s=10.0)
        self.addCleanup(lidar.stop_lidar)
        lidar.scan_data[45] = 700
        lidar.scan_time[45] = time.monotonic() - 5.0
        self.assertEqual(lidar.get_current_scan()[45], 700)   # inside 10 s
        self.assertEqual(self.lidar.stale_after_s, 1.0)       # default unchanged

    def test_an_explicit_window_overrides_the_default(self):
        self._write(30, 600, age_s=2.0)
        self.assertEqual(self.lidar.get_current_scan()[30],
                         Lidar.INVALID_READING)
        self.assertEqual(self.lidar.get_current_scan(max_age_s=5.0)[30], 600)

    def test_none_returns_the_raw_array_without_expiry(self):
        self._write(30, 600, age_s=99.0)
        self.assertEqual(self.lidar.get_current_scan(max_age_s=None)[30], 600)

    def test_a_scan_is_always_360_slots(self):
        self.assertEqual(len(self.lidar.get_current_scan()), 360)
        self.assertEqual(len(self.lidar.get_current_scan(max_age_s=None)), 360)

    def test_slot_age_reports_seconds_since_the_write(self):
        self._write(200, 400, age_s=0.5)
        self.assertAlmostEqual(self.lidar.slot_age(200), 0.5, places=1)

    def test_slot_age_wraps_the_angle(self):
        self._write(5, 400, age_s=0.0)
        self.assertLess(self.lidar.slot_age(365), 1.0)


class LidarSampleTests(unittest.TestCase):
    """sample() pairs a scan with a reading time for the tick to latch."""

    def setUp(self):
        self.lidar = Lidar(port_name="/dev/null")
        self.addCleanup(self.lidar.stop_lidar)

    def test_sample_reports_no_time_before_any_slot_is_written(self):
        scan, t = self.lidar.sample()
        self.assertEqual(len(scan), 360)
        self.assertIsNone(t)

    def test_sample_reports_the_freshest_slots_time(self):
        now = time.monotonic()
        self.lidar.scan_data[10] = 100
        self.lidar.scan_time[10] = now - 0.9
        self.lidar.scan_data[20] = 200
        self.lidar.scan_time[20] = now - 0.1
        _, t = self.lidar.sample()
        self.assertAlmostEqual(t, now - 0.1, places=3)

    def test_sample_applies_the_same_expiry_as_get_current_scan(self):
        self.lidar.scan_data[10] = 100
        self.lidar.scan_time[10] = time.monotonic() - 30.0
        scan, _ = self.lidar.sample()
        self.assertEqual(scan[10], Lidar.INVALID_READING)


class LidarWriteOrderTests(unittest.TestCase):
    """The distance is written before its timestamp, on purpose."""

    def test_source_writes_distance_before_timestamp(self):
        # A reader interleaving between the two sees a NEW distance with an OLD
        # timestamp and discards it as stale — losing a good reading, never
        # serving a stale one. Reversing the order inverts that, so the order
        # is asserted here rather than left to a comment.
        source = Path(__file__).resolve().parents[1] / \
            "src" / "robot_systems" / "lidar.py"
        body = source.read_text()
        scan_body = body.split("def _scan")[1].split("def get_current_scan")[0]
        self.assertLess(scan_body.index("self.scan_data[slot]"),
                        scan_body.index("self.scan_time[slot]"))


if __name__ == "__main__":
    unittest.main()
