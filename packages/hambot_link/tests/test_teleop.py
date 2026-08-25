import io
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from hambot_link import teleop


class FakeBot:
    """Records motor calls so the control loop can be exercised off-hardware."""

    def __init__(self):
        self.calls = []
        self.left = None
        self.right = None

    def set_left_motor_speed(self, speed):
        self.left = speed
        self.calls.append(("left", speed))

    def set_right_motor_speed(self, speed):
        self.right = speed
        self.calls.append(("right", speed))

    def stop_motors(self):
        self.left = self.right = 0
        self.calls.append(("stop", 0))

    @property
    def targets(self):
        """The (left, right) pairs actually pushed to the motors, in order."""
        pairs = []
        for index in range(0, len(self.calls) - 1, 2):
            name, value = self.calls[index]
            if name == "left":
                pairs.append((value, self.calls[index + 1][1]))
        return pairs


class ParseCommandTests(unittest.TestCase):
    def test_parses_a_well_formed_command(self):
        self.assertEqual((3, 40, -40), teleop.parse_command("3 40 -40\n", 75))

    def test_clamps_beyond_the_limit_because_the_wire_is_not_trusted(self):
        self.assertEqual((1, 75, -75), teleop.parse_command("1 4000 -4000", 75))
        self.assertEqual((1, 20, -20), teleop.parse_command("1 4000 -4000", 20))

    def test_rejects_malformed_input(self):
        for line in ("", "\n", "1 2", "1 2 3 4", "a b c", "1 2 x", "1.5 2 3", "-1 0 0"):
            with self.subTest(line=line):
                self.assertIsNone(teleop.parse_command(line, 75))


class CommandSlotTests(unittest.TestCase):
    def test_keeps_only_the_newest_command(self):
        slot = teleop.CommandSlot()
        self.assertTrue(slot.offer(1, 10, 10))
        self.assertTrue(slot.offer(2, 20, 20))
        self.assertEqual((20, 20), slot.read()[:2])

    def test_drops_stale_and_replayed_sequence_numbers(self):
        slot = teleop.CommandSlot()
        slot.offer(5, 75, 75)
        self.assertFalse(slot.offer(4, -75, -75))
        self.assertFalse(slot.offer(5, -75, -75))
        self.assertEqual((75, 75), slot.read()[:2])

    def test_starts_stopped_and_stale(self):
        slot = teleop.CommandSlot()
        left, right, at = slot.read()
        self.assertEqual((0, 0), (left, right))
        self.assertEqual(0.0, at)


class ReadCommandsTests(unittest.TestCase):
    def test_reads_a_stream_and_closes_the_slot_at_eof(self):
        slot = teleop.CommandSlot()
        teleop.read_commands(slot, 75, io.StringIO("1 10 10\n2 20 20\n"))
        self.assertEqual((20, 20), slot.read()[:2])
        self.assertTrue(slot.closed)

    def test_survives_malformed_lines(self):
        slot = teleop.CommandSlot()
        teleop.read_commands(slot, 75, io.StringIO("garbage\n1 10 10\n\n"))
        self.assertEqual((10, 10), slot.read()[:2])

    def test_closes_the_slot_even_when_the_stream_raises(self):
        class Exploding(io.StringIO):
            def readline(self):
                raise OSError("connection reset")

        slot = teleop.CommandSlot()
        with self.assertRaises(OSError):
            teleop.read_commands(slot, 75, Exploding())
        self.assertTrue(slot.closed)


class NextTargetTests(unittest.TestCase):
    def test_passes_a_fresh_command_through(self):
        slot = teleop.CommandSlot()
        slot.offer(1, 40, -40)
        _left, _right, at = slot.read()
        self.assertEqual(((40, -40), False), teleop.next_target(slot, 0.3, at))

    def test_expires_a_command_older_than_the_watchdog(self):
        slot = teleop.CommandSlot()
        slot.offer(1, 75, 75)
        _left, _right, at = slot.read()
        self.assertEqual(((0, 0), True), teleop.next_target(slot, 0.3, at + 0.31))

    def test_a_never_used_slot_is_already_expired(self):
        slot = teleop.CommandSlot()
        self.assertEqual(((0, 0), True), teleop.next_target(slot, 0.3, 100.0))


class ClosingSlot(teleop.CommandSlot):
    """A slot that reports open for a fixed number of control ticks."""

    def __init__(self, ticks):
        super().__init__()
        self._remaining = ticks

    @property
    def closed(self):
        if self._remaining <= 0:
            return True
        self._remaining -= 1
        return False


class DriveTests(unittest.TestCase):
    def test_applies_the_newest_target(self):
        slot = ClosingSlot(1)
        slot.offer(1, 40, -40)
        bot = FakeBot()
        teleop.drive(bot, slot, hz=1000, watchdog=60)
        self.assertEqual([(40, -40)], bot.targets)

    def test_watchdog_stops_when_commands_go_quiet(self):
        slot = ClosingSlot(1)
        slot.offer(1, 75, 75)
        bot = FakeBot()
        # A watchdog this short has already expired by the first tick.
        teleop.drive(bot, slot, hz=1000, watchdog=1e-9)
        self.assertEqual([(0, 0)], bot.targets)

    def test_stops_the_motors_when_the_link_closes(self):
        slot = teleop.CommandSlot()
        slot.offer(1, 75, 75)
        slot.close()
        bot = FakeBot()
        teleop.drive(bot, slot, hz=1000)
        self.assertEqual(("stop", 0), bot.calls[-1])

    def test_does_not_resend_an_unchanged_target(self):
        slot = ClosingSlot(5)
        slot.offer(1, 30, 30)
        bot = FakeBot()
        teleop.drive(bot, slot, hz=1000, watchdog=60)
        # Five ticks, one unchanging target: the Build HAT is written once.
        self.assertEqual([(30, 30)], bot.targets)


class ClampTests(unittest.TestCase):
    def test_clamps_symmetrically(self):
        self.assertEqual(75, teleop.clamp(1000, 75))
        self.assertEqual(-75, teleop.clamp(-1000, 75))
        self.assertEqual(0, teleop.clamp(0, 75))


if __name__ == "__main__":
    unittest.main()
