import sys
import time
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
sys.path.insert(0, str(Path(__file__).resolve().parent))

import fake_hardware

fake_hardware.install()

from robot_systems.robot import HamBot


class HamBotTestCase(unittest.TestCase):
    """Builds a HamBot on stubs. Lidar and camera stay off unless a test wants them."""

    def make_bot(self, **kwargs):
        bot = HamBot(lidar_enabled=False, camera_enabled=False, **kwargs)
        self.addCleanup(bot.disconnect_robot)
        return bot

    def setUp(self):
        self.bot = self.make_bot()


class FreeRunningCompatibilityTests(HamBotTestCase):
    """The four demos never call step(). This path must not have changed.

    If any of these fail, student code that worked before step() existed now
    behaves differently, which is the one outcome the branch is not allowed to
    have.
    """

    def test_a_fresh_robot_is_not_ticked(self):
        self.assertFalse(self.bot._ticked)

    def test_setters_reach_the_hardware_immediately(self):
        self.bot.set_left_motor_speed(30)
        self.bot.set_right_motor_speed(40)
        # No step() has been called, so nothing may be sitting in a buffer.
        self.assertEqual(self.bot.left_motor.speed, -30)   # left is inverted
        self.assertEqual(self.bot.right_motor.speed, 40)
        self.assertEqual(self.bot._pending_speeds, {})

    def test_speeds_are_still_clamped_to_max_rpm(self):
        self.bot.set_right_motor_speed(500)
        self.assertEqual(self.bot.right_motor.speed, self.bot.MAX_RPM)

    def test_stop_reaches_the_hardware_immediately(self):
        self.bot.set_left_motor_speed(30)
        self.bot.stop_motors()
        self.assertIsNone(self.bot.left_motor.speed)
        self.assertIsNone(self.bot.right_motor.speed)

    def test_getters_are_not_pinned(self):
        # Free-running, a getter must follow the hardware rather than a
        # snapshot taken at some earlier instant.
        self.bot.left_motor_radians = 1.0
        self.assertAlmostEqual(self.bot.get_left_encoder_reading(), 1.0)
        time.sleep(self.bot.snapshot_max_age_s * 1.5)
        self.bot.left_motor_radians = 2.0
        self.assertAlmostEqual(self.bot.get_left_encoder_reading(), 2.0)

    def test_heading_still_delegates_with_its_blocking_arguments(self):
        seen = {}

        def spy(fresh_within, blocking, wait_timeout):
            seen.update(fresh_within=fresh_within, blocking=blocking,
                        wait_timeout=wait_timeout)
            return 123.0

        self.bot.imu.get_heading = spy
        value = self.bot.get_heading(fresh_within=0.25, blocking=True,
                                     wait_timeout=0.1)
        self.assertEqual(value, 123.0)
        self.assertEqual(seen, dict(fresh_within=0.25, blocking=True,
                                    wait_timeout=0.1))

    def test_blocking_helpers_go_straight_to_the_hardware(self):
        self.bot.run_motors_for_seconds(0.1, left_speed=20, right_speed=20)
        kinds = [c[0] for c in self.bot.right_motor.calls]
        self.assertIn("run_for_seconds", kinds)


class CoherenceTests(HamBotTestCase):
    """Two getters after one step() must describe the same instant."""

    def test_encoder_getters_agree_within_a_tick(self):
        self.bot.step(0.01)
        first = self.bot.get_left_encoder_reading()
        # The position thread keeps moving underneath; the snapshot must not.
        self.bot.left_motor_radians += 5.0
        time.sleep(0.05)
        self.assertEqual(self.bot.get_left_encoder_reading(), first)
        self.assertEqual(self.bot.get_encoder_readings()[0], first)

    def test_a_new_step_picks_up_the_new_value(self):
        self.bot.step(0.01)
        self.bot.left_motor_radians = 7.0
        self.bot.step(0.01)
        self.assertAlmostEqual(self.bot.get_left_encoder_reading(), 7.0)

    def test_heading_is_pinned_to_the_tick(self):
        self.bot.imu.sample = lambda: (11.0, time.monotonic())
        self.bot.step(0.01)
        self.bot.imu.sample = lambda: (99.0, time.monotonic())
        self.assertEqual(self.bot.get_heading(), 11.0)
        self.bot.step(0.01)
        self.assertEqual(self.bot.get_heading(), 99.0)

    def test_the_snapshot_carries_per_sensor_ages(self):
        self.bot.imu.sample = lambda: (11.0, time.monotonic() - 0.2)
        self.bot.step(0.01)
        snap = self.bot.snapshot()
        self.assertLess(snap.age(), 0.2)
        self.assertGreater(snap.heading_age(), 0.15)

    def test_reset_encoders_is_visible_immediately(self):
        self.bot.left_motor_radians = 3.0
        self.bot.step(0.01)
        self.bot.reset_encoders()
        self.assertEqual(self.bot.get_left_encoder_reading(), 0.0)


class CommandCoalescingTests(HamBotTestCase):

    def test_a_setter_does_not_reach_hardware_until_the_flush(self):
        self.bot.step(0.01)
        self.bot.left_motor.calls.clear()
        self.bot.set_left_motor_speed(30)
        self.assertEqual(self.bot.left_motor.calls, [])
        self.bot.step(0.01)
        self.assertEqual(self.bot.left_motor.calls, [("start", -30)])

    def test_only_the_newest_speed_per_motor_is_sent(self):
        self.bot.step(0.01)
        self.bot.left_motor.calls.clear()
        for speed in (10, 20, 30):
            self.bot.set_left_motor_speed(speed)
        self.bot.step(0.01)
        self.assertEqual(self.bot.left_motor.calls, [("start", -30)])

    def test_the_flush_happens_before_the_wait(self):
        # Flushing first means a command is acted on for the whole period
        # rather than whatever is left of it. Timing the wait is flaky, so
        # assert the effect: after step() returns, the speed is already live.
        self.bot.step(0.01)
        self.bot.set_right_motor_speed(25)
        started = []
        real_start = self.bot.right_motor.start

        def watch(speed=0):
            started.append(time.monotonic())
            real_start(speed=speed)

        self.bot.right_motor.start = watch
        begin = time.monotonic()
        self.bot.step(0.2)
        self.assertTrue(started, "pending speed was never flushed")
        # The write landed near the start of the cycle, not after the wait.
        self.assertLess(started[0] - begin, 0.1)

    def test_commands_survive_across_several_idle_ticks(self):
        self.bot.step(0.01)
        self.bot.set_right_motor_speed(15)
        self.bot.step(0.01)
        self.assertEqual(self.bot.right_motor.speed, 15)
        self.bot.step(0.01)
        self.assertEqual(self.bot.right_motor.speed, 15)


class StopIsNotUndoneTests(HamBotTestCase):
    """The safety property: a stop must survive the next flush.

    Without the cancel in _direct(), a pending speed set earlier in the cycle
    is flushed one tick after the stop and the robot drives on.
    """

    def test_stop_motors_survives_the_next_tick(self):
        self.bot.step(0.01)
        self.bot.set_left_motor_speed(40)
        self.bot.set_right_motor_speed(40)
        self.bot.stop_motors()
        self.bot.step(0.01)
        self.assertIsNone(self.bot.left_motor.speed)
        self.assertIsNone(self.bot.right_motor.speed)

    def test_stop_motors_clears_the_pending_buffer(self):
        self.bot.step(0.01)
        self.bot.set_left_motor_speed(40)
        self.assertTrue(self.bot._pending_speeds)
        self.bot.stop_motors()
        self.assertEqual(self.bot._pending_speeds, {})

    def test_no_start_is_issued_after_the_stop(self):
        self.bot.step(0.01)
        self.bot.set_left_motor_speed(40)
        self.bot.stop_motors()
        self.bot.left_motor.calls.clear()
        self.bot.step(0.01)
        self.bot.step(0.01)
        self.assertEqual(self.bot.left_motor.calls, [])

    def test_a_single_side_stop_only_cancels_that_side(self):
        self.bot.step(0.01)
        self.bot.set_left_motor_speed(40)
        self.bot.set_right_motor_speed(40)
        self.bot.stop_left_motor()
        self.bot.step(0.01)
        self.assertIsNone(self.bot.left_motor.speed)
        self.assertEqual(self.bot.right_motor.speed, 40)

    def test_a_blocking_run_leaves_no_stale_pending_speed(self):
        # run_motors_for_seconds blocks by design and will register an overrun
        # in a ticked loop — that is correct. What must not happen is a speed
        # set beforehand being flushed on top of it a tick later.
        self.bot.step(0.01)
        self.bot.set_left_motor_speed(40)
        self.bot.set_right_motor_speed(40)
        self.bot.run_motors_for_seconds(0.01, left_speed=10, right_speed=10)
        self.assertEqual(self.bot._pending_speeds, {})
        self.bot.left_motor.calls.clear()
        self.bot.right_motor.calls.clear()
        self.bot.step(0.01)
        self.assertEqual(self.bot.left_motor.calls, [])
        self.assertEqual(self.bot.right_motor.calls, [])

    def test_a_speed_set_after_the_stop_still_takes_effect(self):
        # The cancel must not latch: stopping is not a permanent veto.
        self.bot.step(0.01)
        self.bot.stop_motors()
        self.bot.set_left_motor_speed(20)
        self.bot.step(0.01)
        self.assertEqual(self.bot.left_motor.speed, -20)


class ShutdownReentrancyTests(HamBotTestCase):
    """Ctrl-C must stop the motors even mid-command.

    The SIGINT handler runs on the main thread, so it re-enters the command
    lock through stop_motors(). With a plain Lock that deadlocks and Ctrl-C
    hangs with the motors still turning.
    """

    def test_stopping_while_holding_the_command_lock_does_not_deadlock(self):
        self.bot.step(0.01)
        self.bot.set_left_motor_speed(40)
        # Stand in for a signal arriving mid-_command()/_flush_commands().
        with self.bot._command_lock:
            self.bot.stop_motors()
        self.assertIsNone(self.bot.left_motor.speed)
        self.assertEqual(self.bot._pending_speeds, {})

    def test_disconnecting_while_holding_the_command_lock_does_not_deadlock(self):
        self.bot.step(0.01)
        self.bot.set_right_motor_speed(40)
        with self.bot._command_lock:
            self.bot.disconnect_robot()
        self.assertIsNone(self.bot.right_motor.speed)
        self.assertFalse(self.bot.step(0.01))

    def test_the_signal_handler_path_stops_the_motors(self):
        self.bot.step(0.01)
        self.bot.set_left_motor_speed(40)
        self.bot.set_right_motor_speed(40)
        with self.assertRaises(SystemExit):
            self.bot.shutdown(2, None)      # SIGINT
        self.assertIsNone(self.bot.left_motor.speed)
        self.assertIsNone(self.bot.right_motor.speed)
        self.assertEqual(self.bot._pending_speeds, {})


class StepContractTests(HamBotTestCase):

    def test_step_returns_true_while_connected(self):
        self.assertTrue(self.bot.step(0.01))

    def test_step_returns_false_once_disconnected(self):
        self.bot.step(0.01)
        self.bot.disconnect_robot()
        self.assertFalse(self.bot.step(0.01))

    def test_a_while_step_loop_terminates_on_disconnect(self):
        ticks = 0
        while self.bot.step(0.01):
            ticks += 1
            if ticks == 3:
                self.bot.disconnect_robot()
        self.assertEqual(ticks, 3)

    def test_step_is_one_way(self):
        self.bot.step(0.01)
        self.assertTrue(self.bot._ticked)
        self.bot.set_left_motor_speed(10)
        # Still coalescing; there is no route back to free-running.
        self.assertTrue(self.bot._pending_speeds)

    def test_step_paces_to_the_period(self):
        self.bot.step(0.05)
        begin = time.monotonic()
        self.bot.step(0.05)
        self.assertGreater(time.monotonic() - begin, 0.02)

    def test_changing_dt_reanchors_rather_than_stacking_periods(self):
        self.bot.step(0.2)
        self.bot.step(0.01)
        self.assertEqual(self.bot._ticker.period, 0.01)

    def test_disconnect_is_still_idempotent(self):
        self.bot.disconnect_robot()
        self.bot.disconnect_robot()


class LoopHealthTests(HamBotTestCase):

    def test_a_fresh_robot_reports_no_overruns(self):
        self.assertEqual(self.bot.overruns, 0)
        self.assertEqual(self.bot.last_overrun_s, 0.0)
        self.assertEqual(self.bot.worst_overrun_s, 0.0)
        self.assertTrue(self.bot.loop_is_keeping_up())

    def test_a_slow_loop_body_is_reported(self):
        self.bot.step(0.02)
        time.sleep(0.08)               # body far longer than dt
        self.bot.step(0.02)
        self.assertGreaterEqual(self.bot.overruns, 1)
        self.assertGreater(self.bot.worst_overrun_s, 0.0)
        self.assertFalse(self.bot.loop_is_keeping_up())

    def test_keeping_up_recovers(self):
        self.bot.step(0.02)
        time.sleep(0.08)
        self.bot.step(0.02)
        self.assertFalse(self.bot.loop_is_keeping_up())
        self.bot.step(0.02)
        self.assertTrue(self.bot.loop_is_keeping_up())


class DisabledPeripheralTests(HamBotTestCase):

    def test_range_image_reports_disabled_lidar(self):
        self.assertEqual(self.bot.get_range_image(), -1)

    def test_camera_frame_reports_disabled_camera(self):
        self.assertIsNone(self.bot.get_camera_frame())
        self.assertEqual(self.bot.camera_frame_age(), float("inf"))

    def test_stepping_with_no_peripherals_still_works(self):
        self.assertTrue(self.bot.step(0.01))
        snap = self.bot.snapshot()
        self.assertIsNone(snap.range_image)
        self.assertEqual(snap.frame_age(), float("inf"))


class FakeFrame:
    """Stands in for a numpy frame; counts copies so the tick can be checked."""

    def __init__(self, tag):
        self.tag = tag
        self.copies = 0

    def copy(self):
        self.copies += 1
        clone = FakeFrame(self.tag)
        return clone


class FakeCamera:
    def __init__(self):
        self.frame = FakeFrame("a")
        self.t = time.monotonic()

    def sample_frame(self):
        return self.frame, self.t

    def stop_camera(self):
        pass


class CameraLatchingTests(HamBotTestCase):
    """The camera is latched by reference and deliberately not on the tick."""

    def setUp(self):
        super().setUp()
        self.cam = FakeCamera()
        self.bot.camera = self.cam

    def test_the_tick_does_not_copy_the_frame(self):
        # A 640x480 RGB frame is ~900 KB; copying one per tick would cost more
        # than reading every other sensor put together.
        for _ in range(5):
            self.bot.step(0.01)
        self.assertEqual(self.cam.frame.copies, 0)

    def test_get_camera_frame_copies_on_demand(self):
        self.bot.step(0.01)
        self.bot.get_camera_frame()
        self.assertEqual(self.cam.frame.copies, 1)

    def test_copy_false_hands_back_the_latched_array(self):
        self.bot.step(0.01)
        self.assertIs(self.bot.get_camera_frame(copy=False), self.cam.frame)

    def test_a_20hz_loop_sees_the_same_5fps_frame_repeatedly(self):
        # The camera runs at 5 fps, so this is expected — and the age must say
        # so rather than pretending each tick brought a new frame.
        self.bot.step(0.01)
        first = self.bot.get_camera_frame(copy=False)
        self.bot.step(0.01)
        self.assertIs(self.bot.get_camera_frame(copy=False), first)

    def test_frame_age_is_measured_from_capture_not_from_the_tick(self):
        self.cam.t = time.monotonic() - 0.18
        self.bot.step(0.01)
        self.assertGreater(self.bot.camera_frame_age(), 0.15)
        self.assertLess(self.bot.snapshot().age(), 0.1)

    def test_an_uncaptured_frame_reports_inf_not_zero(self):
        self.cam.frame = None
        self.cam.t = None
        self.bot.step(0.01)
        self.assertIsNone(self.bot.get_camera_frame())
        self.assertEqual(self.bot.camera_frame_age(), float("inf"))


class FourWheelDriveTests(unittest.TestCase):
    """4WD routes through the same _command/_direct plumbing."""

    def setUp(self):
        self.bot = HamBot(drivetrain=HamBot.DRIVE_4WD,
                          lidar_enabled=False, camera_enabled=False)
        self.addCleanup(self.bot.disconnect_robot)

    def test_a_side_setter_coalesces_both_of_its_motors(self):
        self.bot.step(0.01)
        self.bot.set_left_motor_speed(30)
        self.assertEqual(len(self.bot._pending_speeds), 2)
        self.bot.step(0.01)
        self.assertEqual(self.bot.front_left_motor.speed, -30)
        self.assertEqual(self.bot.rear_left_motor.speed, -30)

    def test_stop_motors_survives_the_next_tick(self):
        self.bot.step(0.01)
        self.bot.set_left_motor_speed(30)
        self.bot.set_right_motor_speed(30)
        self.bot.stop_motors()
        self.bot.step(0.01)
        for name in ("front_left", "rear_left", "front_right", "rear_right"):
            self.assertIsNone(getattr(self.bot, f"{name}_motor").speed, name)

    def test_encoder_readings_come_from_the_snapshot(self):
        self.bot.front_left_motor_radians = 1.5
        self.bot.step(0.01)
        self.assertAlmostEqual(self.bot.get_front_left_encoder_reading(), 1.5)
        self.assertAlmostEqual(self.bot.get_encoder_readings()[0], 1.5)


if __name__ == "__main__":
    unittest.main()
