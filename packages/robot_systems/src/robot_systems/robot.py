import math, time, threading, sys, signal
from buildhat import Motor
from robot_systems.imu import IMU
from robot_systems.lidar import Lidar
from robot_systems.camera import Camera
from robot_systems.ticker import Snapshot, Ticker

class HamBot:
    DRIVE_2WD = '2WD'
    DRIVE_4WD = '4WD'

    def __init__(self, drivetrain='2WD', lidar_enabled=False, camera_enabled=False, camera_type=None):
        """Initialize HamBot with the specified drivetrain and optional peripherals.

        Args:
            drivetrain (str): Drivetrain type. Use HamBot.DRIVE_2WD or HamBot.DRIVE_4WD.
                              Defaults to '2WD'.
            lidar_enabled (bool): Whether to initialize the Lidar sensor. Defaults to True.
            camera_enabled (bool): Whether to initialize the camera. Defaults to True.

        Raises:
            ValueError: If drivetrain is not DRIVE_2WD or DRIVE_4WD.
        """
        if drivetrain not in (self.DRIVE_2WD, self.DRIVE_4WD):
            raise ValueError(f"Invalid drivetrain '{drivetrain}'. Must be '{self.DRIVE_2WD}' or '{self.DRIVE_4WD}'.")

        self.drivetrain = drivetrain
        self.MAX_RPM = 100

        # --- control tick state ---------------------------------------------
        # _ticked starts False and flips one-way on the first step() call.
        # Which mode we are in is decided by behaviour, not by a constructor
        # flag, so a script that never calls step() behaves exactly as it did
        # before step() existed.
        self._ticked = False
        self._ticker = None
        self._snapshot = Snapshot()
        # Matches the 20 Hz IMU and encoder pollers: a free-running getter that
        # refreshes when the snapshot passes this age is no staler than the
        # live attributes it replaced.
        self.snapshot_max_age_s = 0.05
        self._pending_speeds = {}
        # Re-entrant on purpose. The SIGINT/SIGTERM handler runs on the main
        # thread, so a signal arriving while that thread is inside _command(),
        # _direct() or _flush_commands() would re-enter this lock via
        # stop_motors(). A plain Lock deadlocks there — Ctrl-C would hang with
        # the motors still turning, which is the one failure this class must
        # not have. The windows are only a few bytecodes long, but Python
        # checks for signals between bytecodes, so they are reachable.
        self._command_lock = threading.RLock()

        self.imu = IMU(poll_hz=20.0)
        self.imu.start()

        if self.drivetrain == self.DRIVE_2WD:
            self.left_motor = Motor('B')
            self.left_motor.set_speed_unit_rpm(rpm=True)
            self.right_motor = Motor('A')
            self.right_motor.set_speed_unit_rpm(rpm=True)

            self.left_motor_radians = 0.0
            self.last_left_position = self.left_motor.get_position()
            self.right_motor_radians = 0.0
            self.last_right_position = self.right_motor.get_position()
        else:
            self.front_left_motor = Motor('C')
            self.front_left_motor.set_speed_unit_rpm(rpm=True)
            self.rear_left_motor = Motor('D')
            self.rear_left_motor.set_speed_unit_rpm(rpm=True)
            self.front_right_motor = Motor('B')
            self.front_right_motor.set_speed_unit_rpm(rpm=True)
            self.rear_right_motor = Motor('A')
            self.rear_right_motor.set_speed_unit_rpm(rpm=True)

            self.front_left_motor_radians = 0.0
            self.last_front_left_position = self.front_left_motor.get_position()
            self.rear_left_motor_radians = 0.0
            self.last_rear_left_position = self.rear_left_motor.get_position()
            self.front_right_motor_radians = 0.0
            self.last_front_right_position = self.front_right_motor.get_position()
            self.rear_right_motor_radians = 0.0
            self.last_rear_right_position = self.rear_right_motor.get_position()

        if lidar_enabled:
            self.lidar = Lidar()
        else:
            self.lidar = None

        if camera_enabled:
            cam_type = camera_type if camera_type is not None else Camera.CAM_PICAM
            try:
                self.camera = Camera.create(cam_type)
            except Exception as e:
                print(f"Camera initialization failed ({e}). Operating without camera.")
                self.camera = None
        else:
            self.camera = None

        self.stop_thread = False
        self.disconnected = False
        self.position_thread = threading.Thread(target=self.update_motor_positions)
        self.position_thread.start()

        # SIGTERM matters as much as SIGINT: systemd and `kill` use it, and with
        # no handler the process dies while the motors are still turning.
        # signal.signal() only works on the main thread, so a HamBot built on a
        # worker thread simply goes without handlers.
        for sig in (signal.SIGINT, signal.SIGTERM):
            try:
                signal.signal(sig, self.shutdown)
            except ValueError:
                pass

    # -------------------------------------------------------------------------
    # Control tick
    # -------------------------------------------------------------------------

    def step(self, dt=0.05):
        """Advance one control cycle: flush commands, pace to the boundary, sample.

        Shaped after a Webots controller loop, so the same student code reads
        the same way in simulation and on hardware::

            while bot.step(0.05):
                heading = bot.get_heading()       # both describe
                scan    = bot.get_range_image()   # the same instant

        The order is flush, wait, sample. Flushing *before* the wait means a
        speed set during the previous cycle is acted on for this whole period
        rather than only for whatever is left of it after the wait.

        Calling step() at all switches the robot into ticked mode for the rest
        of its life: setters coalesce until the next flush, and getters are
        pinned to the snapshot this call latched. The pinning is the coherence
        guarantee, so it is not relaxed for convenience.

        Overruns slip rather than skip — see ``robot_systems.ticker.Ticker``
        and ``docs/step.md``. No cycle is dropped, but after an overrun
        elapsed time is no longer ``ticks * dt``.

        Args:
            dt (float): Seconds per cycle. Defaults to 0.05 (20 Hz). Changing
                        it between calls re-anchors the schedule.

        Returns:
            bool: True normally, False once the robot has been disconnected,
                  mirroring Webots' ``while robot.step(t) != -1``.
        """
        if self.disconnected:
            return False

        self._ticked = True
        if self._ticker is None:
            self._ticker = Ticker(dt)
        elif dt != self._ticker.period:
            self._ticker.reset(period=dt)

        self._flush_commands()
        self._ticker.wait()

        # Ctrl-C during the wait runs the signal handler, which disconnects.
        # Re-check rather than touching hardware that has already been shut down.
        if self.disconnected:
            return False

        self._refresh_snapshot()
        return True

    def _refresh_snapshot(self):
        """Read every sensor once into a single coherent Snapshot."""
        now = time.monotonic()

        heading = heading_t = None
        if self.imu is not None:
            heading, heading_t = self.imu.sample()

        scan = scan_t = None
        if self.lidar is not None:
            scan, scan_t = self.lidar.sample()

        # By reference, never copied — see Camera.sample_frame().
        frame = frame_t = None
        if self.camera is not None:
            frame, frame_t = self.camera.sample_frame()

        self._snapshot = Snapshot(
            t=now,
            heading=heading, heading_t=heading_t,
            range_image=scan, range_image_t=scan_t,
            encoders=self._read_encoder_totals(),
            frame=frame, frame_t=frame_t,
        )

    def _ensure_fresh(self):
        """Refresh the snapshot if free-running and it has gone stale.

        In ticked mode this does nothing: getters stay pinned to the snapshot
        that step() latched, which is what makes two getters agree.
        """
        if self._ticked:
            return
        if self._snapshot.age() > self.snapshot_max_age_s:
            self._refresh_snapshot()

    def _read_encoder_totals(self):
        """Return the accumulated encoder radians the position thread maintains.

        Args:
            None

        Returns:
            dict: Motor name to accumulated radians.
        """
        if self.drivetrain == self.DRIVE_2WD:
            return {
                'left': self.left_motor_radians,
                'right': self.right_motor_radians,
            }
        return {
            'front_left': self.front_left_motor_radians,
            'rear_left': self.rear_left_motor_radians,
            'front_right': self.front_right_motor_radians,
            'rear_right': self.rear_right_motor_radians,
        }

    # -------------------------------------------------------------------------
    # Command coalescing
    # -------------------------------------------------------------------------

    def _command(self, motor, speed):
        """Apply a continuous speed now, or hold it for the next flush.

        Args:
            motor (Motor): The motor to drive.
            speed (float): Speed in RPM, already sign-corrected and clamped.
        """
        if not self._ticked:
            motor.start(speed=speed)
            return
        with self._command_lock:
            self._pending_speeds[motor] = speed

    def _flush_commands(self):
        """Push the newest pending speed to each motor that has one.

        Only the newest speed per motor is sent: a loop body that sets a speed
        three times in one cycle should produce one hardware write, not three.
        """
        with self._command_lock:
            pending = self._pending_speeds
            self._pending_speeds = {}
        for motor, speed in pending.items():
            motor.start(speed=speed)

    def _direct(self, motor):
        """Cancel any pending speed for a motor and return it for immediate use.

        Stops and timed/rotation/position runs cannot wait for a boundary, so
        they go straight to the hardware. The cancel is the safety-critical
        half: without it, ``stop_motors()`` would be silently undone by the
        next flush one tick later, and the robot would drive on after the
        student told it to stop.

        Args:
            motor (Motor): The motor about to be commanded directly.

        Returns:
            Motor: The same motor, so callers read as
                   ``self._direct(self.left_motor).stop()``.
        """
        with self._command_lock:
            self._pending_speeds.pop(motor, None)
        return motor

    # -------------------------------------------------------------------------
    # Loop health
    # -------------------------------------------------------------------------

    @property
    def overruns(self):
        """int: Number of ticks that missed their boundary."""
        return self._ticker.overruns if self._ticker is not None else 0

    @property
    def last_overrun_s(self):
        """float: How late the most recent overrun was, in seconds."""
        return self._ticker.last_overrun_s if self._ticker is not None else 0.0

    @property
    def worst_overrun_s(self):
        """float: How late the worst overrun so far was, in seconds."""
        return self._ticker.worst_overrun_s if self._ticker is not None else 0.0

    def loop_is_keeping_up(self):
        """Whether the most recent tick met its deadline.

        Returns:
            bool: True if the last step() made its boundary, or if step() has
                  not been called yet.
        """
        if self._ticker is None:
            return True
        return self._ticker.is_keeping_up()

    # -------------------------------------------------------------------------
    # Sensors
    # -------------------------------------------------------------------------

    def get_range_image(self):
        """Return the current 360° lidar scan as a list of distance measurements.

        Returns:
            list: 360 distance values (mm) indexed by degree, where 180° is the
                  front of the robot and 0° is the rear. Slots not measured
                  within the lidar's staleness window read as
                  Lidar.INVALID_READING (-1), so an angle nothing has bounced
                  off recently is distinguishable from one measured as empty.
                  Returns -1 if the lidar is not enabled.
        """
        if self.lidar is None:
            print("Lidar is not enabled.")
            return -1
        if self._ticked:
            self._ensure_fresh()
            return self._snapshot.range_image
        return self.lidar.get_current_scan()

    def get_heading(self, fresh_within=0.5, blocking=False, wait_timeout=0.3):
        """Return the robot's current heading from the IMU.

        Args:
            fresh_within (float): Maximum age in seconds for an acceptable reading.
                                  Defaults to 0.5.
            blocking (bool): If True, wait for a fresh reading. Defaults to False.
            wait_timeout (float): Maximum time in seconds to wait when blocking.
                                  Defaults to 0.3.

        Returns:
            float: Heading in degrees relative to East (0°–360°).
        """
        if self._ticked:
            # Pinned to the tick, so it agrees with every other getter called
            # after the same step(). The freshness arguments describe a
            # request the tick has already made, so they do not apply here;
            # use snapshot().heading_age() to judge the reading's age.
            self._ensure_fresh()
            return self._snapshot.heading
        # Free-running: delegate verbatim, including the blocking wait, so
        # this path is indistinguishable from the pre-step() library.
        return self.imu.get_heading(fresh_within=fresh_within,
                                    blocking=blocking,
                                    wait_timeout=wait_timeout)

    def get_camera_frame(self, copy=True):
        """Return the latest camera frame.

        The camera is deliberately *not* driven by the tick. It runs at 5 fps,
        so a 20 Hz loop sees the same frame about four times over; step() only
        latches whatever the capture thread has, and camera_frame_age() reports
        honestly how old that is.

        Args:
            copy (bool): If True return a copy, which is what student code
                         wants if it intends to draw on the frame. If False
                         return the latched array itself — cheaper, but the
                         capture thread will replace it, so treat it as
                         read-only. Defaults to True.

        Returns:
            numpy.ndarray | None: (H, W, 3) uint8 RGB array, or None if the
                                  camera is disabled or has captured nothing.
        """
        if self.camera is None:
            print("Camera is not enabled.")
            return None
        self._ensure_fresh()
        frame = self._snapshot.frame
        if frame is None:
            return None
        return frame.copy() if copy else frame

    def camera_frame_age(self):
        """Seconds since the latest camera frame was captured.

        Returns:
            float: Age in seconds, or inf if the camera is disabled or has
                   captured nothing. At 5 fps this is normally 0-200 ms and a
                   20 Hz loop will see the same frame repeatedly.
        """
        if self.camera is None:
            return float("inf")
        self._ensure_fresh()
        return self._snapshot.frame_age()

    def snapshot(self):
        """Return the Snapshot the getters are currently reading from.

        Useful for per-sensor ages — ``bot.snapshot().heading_age()`` — which
        the individual getters do not expose.

        Returns:
            Snapshot: The latched snapshot. In ticked mode this is the one
                      step() took; free-running, it is refreshed if stale.
        """
        self._ensure_fresh()
        return self._snapshot

    # -------------------------------------------------------------------------
    # Encoder tracking (internal)
    # -------------------------------------------------------------------------

    def _read_motor_delta(self, motor, last_position, invert=False):
        """Read a motor's position and return the displacement since last_position.

        Handles the 360°→0° wrap-around of the BuildHat position sensor.

        Args:
            motor (Motor): The BuildHat Motor instance to read.
            last_position (float): The position recorded on the previous update.
            invert (bool): If True, negate the delta to correct for mounting polarity.
                           Defaults to False.

        Returns:
            tuple: (current_position, delta_radians) where current_position is the
                   raw reading to store for the next call and delta_radians is the
                   signed angular displacement in radians.
        """
        current = motor.get_position()
        delta = current - last_position
        if delta > 180:
            delta -= 360
        elif delta < -180:
            delta += 360
        return current, -math.radians(delta) if invert else math.radians(delta)

    def update_motor_positions(self):
        """Continuously update accumulated encoder radians for all drive motors.

        Runs in a background thread at 20 Hz. Polls each motor's position,
        computes the delta since the last reading, and accumulates it into the
        corresponding *_motor_radians attribute.
        """
        while not self.stop_thread:
            if self.drivetrain == self.DRIVE_2WD:
                self.last_left_position, delta = self._read_motor_delta(
                    self.left_motor, self.last_left_position, invert=True)
                self.left_motor_radians += delta

                self.last_right_position, delta = self._read_motor_delta(
                    self.right_motor, self.last_right_position)
                self.right_motor_radians += delta
            else:
                self.last_front_left_position, delta = self._read_motor_delta(
                    self.front_left_motor, self.last_front_left_position, invert=True)
                self.front_left_motor_radians += delta

                self.last_rear_left_position, delta = self._read_motor_delta(
                    self.rear_left_motor, self.last_rear_left_position, invert=True)
                self.rear_left_motor_radians += delta

                self.last_front_right_position, delta = self._read_motor_delta(
                    self.front_right_motor, self.last_front_right_position)
                self.front_right_motor_radians += delta

                self.last_rear_right_position, delta = self._read_motor_delta(
                    self.rear_right_motor, self.last_rear_right_position)
                self.rear_right_motor_radians += delta

            time.sleep(0.05)

    def _encoders(self):
        """Return the encoder totals the getters should report.

        Ticked, this is the dict step() latched, so two encoder reads after one
        step() describe the same instant. Free-running, the snapshot is
        refreshed when stale, which at snapshot_max_age_s == 0.05 is no staler
        than the 20 Hz position thread the getters used to read directly.

        Returns:
            dict: Motor name to accumulated radians.
        """
        self._ensure_fresh()
        return self._snapshot.encoders

    def reset_encoders(self):
        """Reset all encoder accumulators to zero and re-snapshot current positions.

        For 2WD resets left and right. For 4WD resets all four wheels
        individually. The reset is visible to the encoder getters immediately,
        without waiting for the next step().
        """
        if self.drivetrain == self.DRIVE_2WD:
            self.left_motor_radians = 0.0
            self.right_motor_radians = 0.0
            self.last_left_position = self.left_motor.get_position()
            self.last_right_position = self.right_motor.get_position()
        else:
            self.front_left_motor_radians = 0.0
            self.rear_left_motor_radians = 0.0
            self.front_right_motor_radians = 0.0
            self.rear_right_motor_radians = 0.0
            self.last_front_left_position = self.front_left_motor.get_position()
            self.last_rear_left_position = self.rear_left_motor.get_position()
            self.last_front_right_position = self.front_right_motor.get_position()
            self.last_rear_right_position = self.rear_right_motor.get_position()

        # The getters read the snapshot, so re-latch the totals here or a read
        # before the next refresh would still report the pre-reset values.
        self._snapshot.encoders = self._read_encoder_totals()

    # -------------------------------------------------------------------------
    # Speed validation
    # -------------------------------------------------------------------------

    def check_speed(self, input_speed):
        """Clamp a speed value to the valid range [-MAX_RPM, MAX_RPM].

        Args:
            input_speed (float): Requested speed in RPM.

        Returns:
            float: The clamped speed value.
        """
        if -self.MAX_RPM <= input_speed <= self.MAX_RPM:
            return input_speed
        elif input_speed < -self.MAX_RPM:
            print(f"Speed must be between -{self.MAX_RPM} and {self.MAX_RPM} revolutions per minute.")
            return -self.MAX_RPM
        elif input_speed > self.MAX_RPM:
            print(f"Speed must be between -{self.MAX_RPM} and {self.MAX_RPM} revolutions per minute.")
            return self.MAX_RPM

    # -------------------------------------------------------------------------
    # Left-side motor control (2WD and 4WD)
    # -------------------------------------------------------------------------

    def set_left_motor_speed(self, speed_rpm):
        """Set the left side to run continuously at the given speed.

        For 4WD, both front-left and rear-left motors are set simultaneously.

        Args:
            speed_rpm (float): Desired speed in RPM. Positive = forward,
                               negative = reverse.
        """
        speed_rpm *= -1
        speed_rpm = self.check_speed(speed_rpm)
        if self.drivetrain == self.DRIVE_2WD:
            self._command(self.left_motor, speed_rpm)
        else:
            self._command(self.front_left_motor, speed_rpm)
            self._command(self.rear_left_motor, speed_rpm)

    def run_left_motor_for_seconds(self, seconds, speed=75, blocking=True):
        """Run the left side for a fixed duration.

        For 4WD, front-left runs non-blocking and rear-left respects the
        blocking parameter so both motors run in tandem.

        Args:
            seconds (float): Duration to run in seconds.
            speed (float): Speed in RPM. Defaults to 75.
            blocking (bool): If True, the call blocks until the motors finish.
                             Defaults to True.
        """
        speed *= -1
        speed = self.check_speed(speed)
        if self.drivetrain == self.DRIVE_2WD:
            self._direct(self.left_motor).run_for_seconds(seconds, speed=speed, blocking=blocking)
        else:
            self._direct(self.front_left_motor).run_for_seconds(seconds, speed=speed, blocking=False)
            self._direct(self.rear_left_motor).run_for_seconds(seconds, speed=speed, blocking=blocking)

    def run_left_motor_for_rotations(self, rotations, speed=75, blocking=True):
        """Run the left side for a fixed number of rotations.

        For 4WD, front-left runs non-blocking and rear-left respects the
        blocking parameter so both motors run in tandem.

        Args:
            rotations (float): Number of rotations to perform.
            speed (float): Speed in RPM. Defaults to 75.
            blocking (bool): If True, the call blocks until the motors finish.
                             Defaults to True.
        """
        speed *= -1
        speed = self.check_speed(speed)
        if self.drivetrain == self.DRIVE_2WD:
            self._direct(self.left_motor).run_for_rotations(rotations, speed=speed, blocking=blocking)
        else:
            self._direct(self.front_left_motor).run_for_rotations(rotations, speed=speed, blocking=False)
            self._direct(self.rear_left_motor).run_for_rotations(rotations, speed=speed, blocking=blocking)

    def run_left_motor_to_position(self, position, speed=100, blocking=True):
        """Move the left side to an absolute position.

        For 4WD, front-left runs non-blocking and rear-left respects the
        blocking parameter so both motors run in tandem.

        Args:
            position (float): Target position in degrees.
            speed (float): Speed in RPM. Defaults to 100.
            blocking (bool): If True, the call blocks until the motors finish.
                             Defaults to True.
        """
        speed *= -1
        speed = self.check_speed(speed)
        if self.drivetrain == self.DRIVE_2WD:
            self._direct(self.left_motor).run_to_position(position, speed=speed, blocking=blocking)
        else:
            self._direct(self.front_left_motor).run_to_position(position, speed=speed, blocking=False)
            self._direct(self.rear_left_motor).run_to_position(position, speed=speed, blocking=blocking)

    def stop_left_motor(self):
        """Stop the left side motor(s).

        For 4WD, stops both front-left and rear-left motors.
        """
        if self.drivetrain == self.DRIVE_2WD:
            self._direct(self.left_motor).stop()
        else:
            self._direct(self.front_left_motor).stop()
            self._direct(self.rear_left_motor).stop()

    # -------------------------------------------------------------------------
    # Right-side motor control (2WD and 4WD)
    # -------------------------------------------------------------------------

    def set_right_motor_speed(self, speed_rpm):
        """Set the right side to run continuously at the given speed.

        For 4WD, both front-right and rear-right motors are set simultaneously.

        Args:
            speed_rpm (float): Desired speed in RPM. Positive = forward,
                               negative = reverse.
        """
        speed_rpm = self.check_speed(speed_rpm)
        if self.drivetrain == self.DRIVE_2WD:
            self._command(self.right_motor, speed_rpm)
        else:
            self._command(self.front_right_motor, speed_rpm)
            self._command(self.rear_right_motor, speed_rpm)

    def run_right_motor_for_seconds(self, seconds, speed=75, blocking=True):
        """Run the right side for a fixed duration.

        For 4WD, front-right runs non-blocking and rear-right respects the
        blocking parameter so both motors run in tandem.

        Args:
            seconds (float): Duration to run in seconds.
            speed (float): Speed in RPM. Defaults to 75.
            blocking (bool): If True, the call blocks until the motors finish.
                             Defaults to True.
        """
        speed = self.check_speed(speed)
        if self.drivetrain == self.DRIVE_2WD:
            self._direct(self.right_motor).run_for_seconds(seconds, speed=speed, blocking=blocking)
        else:
            self._direct(self.front_right_motor).run_for_seconds(seconds, speed=speed, blocking=False)
            self._direct(self.rear_right_motor).run_for_seconds(seconds, speed=speed, blocking=blocking)

    def run_right_motor_for_rotations(self, rotations, speed=75, blocking=True):
        """Run the right side for a fixed number of rotations.

        For 4WD, front-right runs non-blocking and rear-right respects the
        blocking parameter so both motors run in tandem.

        Args:
            rotations (float): Number of rotations to perform.
            speed (float): Speed in RPM. Defaults to 75.
            blocking (bool): If True, the call blocks until the motors finish.
                             Defaults to True.
        """
        speed = self.check_speed(speed)
        if self.drivetrain == self.DRIVE_2WD:
            self._direct(self.right_motor).run_for_rotations(rotations, speed=speed, blocking=blocking)
        else:
            self._direct(self.front_right_motor).run_for_rotations(rotations, speed=speed, blocking=False)
            self._direct(self.rear_right_motor).run_for_rotations(rotations, speed=speed, blocking=blocking)

    def run_right_motor_to_position(self, position, speed=75, blocking=True):
        """Move the right side to an absolute position.

        For 4WD, front-right runs non-blocking and rear-right respects the
        blocking parameter so both motors run in tandem.

        Args:
            position (float): Target position in degrees.
            speed (float): Speed in RPM. Defaults to 75.
            blocking (bool): If True, the call blocks until the motors finish.
                             Defaults to True.
        """
        speed = self.check_speed(speed)
        if self.drivetrain == self.DRIVE_2WD:
            self._direct(self.right_motor).run_to_position(position, speed=speed, blocking=blocking)
        else:
            self._direct(self.front_right_motor).run_to_position(position, speed=speed, blocking=False)
            self._direct(self.rear_right_motor).run_to_position(position, speed=speed, blocking=blocking)

    def stop_right_motor(self):
        """Stop the right side motor(s).

        For 4WD, stops both front-right and rear-right motors.
        """
        if self.drivetrain == self.DRIVE_2WD:
            self._direct(self.right_motor).stop()
        else:
            self._direct(self.front_right_motor).stop()
            self._direct(self.rear_right_motor).stop()

    # -------------------------------------------------------------------------
    # Individual wheel control (4WD only)
    # -------------------------------------------------------------------------

    def set_front_left_motor_speed(self, speed_rpm):
        """Set the front-left motor to run continuously at the given speed (4WD only).

        Args:
            speed_rpm (float): Desired speed in RPM. Positive = forward,
                               negative = reverse.
        """
        if self.drivetrain != self.DRIVE_4WD:
            print("set_front_left_motor_speed is only available on 4WD.")
            return
        speed_rpm *= -1
        speed_rpm = self.check_speed(speed_rpm)
        self._command(self.front_left_motor, speed_rpm)

    def run_front_left_motor_for_seconds(self, seconds, speed=75, blocking=True):
        """Run the front-left motor for a fixed duration (4WD only).

        Args:
            seconds (float): Duration to run in seconds.
            speed (float): Speed in RPM. Defaults to 75.
            blocking (bool): If True, the call blocks until the motor finishes.
                             Defaults to True.
        """
        if self.drivetrain != self.DRIVE_4WD:
            print("run_front_left_motor_for_seconds is only available on 4WD.")
            return
        speed *= -1
        speed = self.check_speed(speed)
        self._direct(self.front_left_motor).run_for_seconds(seconds, speed=speed, blocking=blocking)

    def run_front_left_motor_for_rotations(self, rotations, speed=75, blocking=True):
        """Run the front-left motor for a fixed number of rotations (4WD only).

        Args:
            rotations (float): Number of rotations to perform.
            speed (float): Speed in RPM. Defaults to 75.
            blocking (bool): If True, the call blocks until the motor finishes.
                             Defaults to True.
        """
        if self.drivetrain != self.DRIVE_4WD:
            print("run_front_left_motor_for_rotations is only available on 4WD.")
            return
        speed *= -1
        speed = self.check_speed(speed)
        self._direct(self.front_left_motor).run_for_rotations(rotations, speed=speed, blocking=blocking)

    def run_front_left_motor_to_position(self, position, speed=100, blocking=True):
        """Move the front-left motor to an absolute position (4WD only).

        Args:
            position (float): Target position in degrees.
            speed (float): Speed in RPM. Defaults to 100.
            blocking (bool): If True, the call blocks until the motor finishes.
                             Defaults to True.
        """
        if self.drivetrain != self.DRIVE_4WD:
            print("run_front_left_motor_to_position is only available on 4WD.")
            return
        speed *= -1
        speed = self.check_speed(speed)
        self._direct(self.front_left_motor).run_to_position(position, speed=speed, blocking=blocking)

    def stop_front_left_motor(self):
        """Stop the front-left motor (4WD only)."""
        if self.drivetrain != self.DRIVE_4WD:
            print("stop_front_left_motor is only available on 4WD.")
            return
        self._direct(self.front_left_motor).stop()

    def set_rear_left_motor_speed(self, speed_rpm):
        """Set the rear-left motor to run continuously at the given speed (4WD only).

        Args:
            speed_rpm (float): Desired speed in RPM. Positive = forward,
                               negative = reverse.
        """
        if self.drivetrain != self.DRIVE_4WD:
            print("set_rear_left_motor_speed is only available on 4WD.")
            return
        speed_rpm *= -1
        speed_rpm = self.check_speed(speed_rpm)
        self._command(self.rear_left_motor, speed_rpm)

    def run_rear_left_motor_for_seconds(self, seconds, speed=75, blocking=True):
        """Run the rear-left motor for a fixed duration (4WD only).

        Args:
            seconds (float): Duration to run in seconds.
            speed (float): Speed in RPM. Defaults to 75.
            blocking (bool): If True, the call blocks until the motor finishes.
                             Defaults to True.
        """
        if self.drivetrain != self.DRIVE_4WD:
            print("run_rear_left_motor_for_seconds is only available on 4WD.")
            return
        speed *= -1
        speed = self.check_speed(speed)
        self._direct(self.rear_left_motor).run_for_seconds(seconds, speed=speed, blocking=blocking)

    def run_rear_left_motor_for_rotations(self, rotations, speed=75, blocking=True):
        """Run the rear-left motor for a fixed number of rotations (4WD only).

        Args:
            rotations (float): Number of rotations to perform.
            speed (float): Speed in RPM. Defaults to 75.
            blocking (bool): If True, the call blocks until the motor finishes.
                             Defaults to True.
        """
        if self.drivetrain != self.DRIVE_4WD:
            print("run_rear_left_motor_for_rotations is only available on 4WD.")
            return
        speed *= -1
        speed = self.check_speed(speed)
        self._direct(self.rear_left_motor).run_for_rotations(rotations, speed=speed, blocking=blocking)

    def run_rear_left_motor_to_position(self, position, speed=100, blocking=True):
        """Move the rear-left motor to an absolute position (4WD only).

        Args:
            position (float): Target position in degrees.
            speed (float): Speed in RPM. Defaults to 100.
            blocking (bool): If True, the call blocks until the motor finishes.
                             Defaults to True.
        """
        if self.drivetrain != self.DRIVE_4WD:
            print("run_rear_left_motor_to_position is only available on 4WD.")
            return
        speed *= -1
        speed = self.check_speed(speed)
        self._direct(self.rear_left_motor).run_to_position(position, speed=speed, blocking=blocking)

    def stop_rear_left_motor(self):
        """Stop the rear-left motor (4WD only)."""
        if self.drivetrain != self.DRIVE_4WD:
            print("stop_rear_left_motor is only available on 4WD.")
            return
        self._direct(self.rear_left_motor).stop()

    def set_front_right_motor_speed(self, speed_rpm):
        """Set the front-right motor to run continuously at the given speed (4WD only).

        Args:
            speed_rpm (float): Desired speed in RPM. Positive = forward,
                               negative = reverse.
        """
        if self.drivetrain != self.DRIVE_4WD:
            print("set_front_right_motor_speed is only available on 4WD.")
            return
        speed_rpm = self.check_speed(speed_rpm)
        self._command(self.front_right_motor, speed_rpm)

    def run_front_right_motor_for_seconds(self, seconds, speed=75, blocking=True):
        """Run the front-right motor for a fixed duration (4WD only).

        Args:
            seconds (float): Duration to run in seconds.
            speed (float): Speed in RPM. Defaults to 75.
            blocking (bool): If True, the call blocks until the motor finishes.
                             Defaults to True.
        """
        if self.drivetrain != self.DRIVE_4WD:
            print("run_front_right_motor_for_seconds is only available on 4WD.")
            return
        speed = self.check_speed(speed)
        self._direct(self.front_right_motor).run_for_seconds(seconds, speed=speed, blocking=blocking)

    def run_front_right_motor_for_rotations(self, rotations, speed=75, blocking=True):
        """Run the front-right motor for a fixed number of rotations (4WD only).

        Args:
            rotations (float): Number of rotations to perform.
            speed (float): Speed in RPM. Defaults to 75.
            blocking (bool): If True, the call blocks until the motor finishes.
                             Defaults to True.
        """
        if self.drivetrain != self.DRIVE_4WD:
            print("run_front_right_motor_for_rotations is only available on 4WD.")
            return
        speed = self.check_speed(speed)
        self._direct(self.front_right_motor).run_for_rotations(rotations, speed=speed, blocking=blocking)

    def run_front_right_motor_to_position(self, position, speed=100, blocking=True):
        """Move the front-right motor to an absolute position (4WD only).

        Args:
            position (float): Target position in degrees.
            speed (float): Speed in RPM. Defaults to 100.
            blocking (bool): If True, the call blocks until the motor finishes.
                             Defaults to True.
        """
        if self.drivetrain != self.DRIVE_4WD:
            print("run_front_right_motor_to_position is only available on 4WD.")
            return
        speed = self.check_speed(speed)
        self._direct(self.front_right_motor).run_to_position(position, speed=speed, blocking=blocking)

    def stop_front_right_motor(self):
        """Stop the front-right motor (4WD only)."""
        if self.drivetrain != self.DRIVE_4WD:
            print("stop_front_right_motor is only available on 4WD.")
            return
        self._direct(self.front_right_motor).stop()

    def set_rear_right_motor_speed(self, speed_rpm):
        """Set the rear-right motor to run continuously at the given speed (4WD only).

        Args:
            speed_rpm (float): Desired speed in RPM. Positive = forward,
                               negative = reverse.
        """
        if self.drivetrain != self.DRIVE_4WD:
            print("set_rear_right_motor_speed is only available on 4WD.")
            return
        speed_rpm = self.check_speed(speed_rpm)
        self._command(self.rear_right_motor, speed_rpm)

    def run_rear_right_motor_for_seconds(self, seconds, speed=75, blocking=True):
        """Run the rear-right motor for a fixed duration (4WD only).

        Args:
            seconds (float): Duration to run in seconds.
            speed (float): Speed in RPM. Defaults to 75.
            blocking (bool): If True, the call blocks until the motor finishes.
                             Defaults to True.
        """
        if self.drivetrain != self.DRIVE_4WD:
            print("run_rear_right_motor_for_seconds is only available on 4WD.")
            return
        speed = self.check_speed(speed)
        self._direct(self.rear_right_motor).run_for_seconds(seconds, speed=speed, blocking=blocking)

    def run_rear_right_motor_for_rotations(self, rotations, speed=75, blocking=True):
        """Run the rear-right motor for a fixed number of rotations (4WD only).

        Args:
            rotations (float): Number of rotations to perform.
            speed (float): Speed in RPM. Defaults to 75.
            blocking (bool): If True, the call blocks until the motor finishes.
                             Defaults to True.
        """
        if self.drivetrain != self.DRIVE_4WD:
            print("run_rear_right_motor_for_rotations is only available on 4WD.")
            return
        speed = self.check_speed(speed)
        self._direct(self.rear_right_motor).run_for_rotations(rotations, speed=speed, blocking=blocking)

    def run_rear_right_motor_to_position(self, position, speed=100, blocking=True):
        """Move the rear-right motor to an absolute position (4WD only).

        Args:
            position (float): Target position in degrees.
            speed (float): Speed in RPM. Defaults to 100.
            blocking (bool): If True, the call blocks until the motor finishes.
                             Defaults to True.
        """
        if self.drivetrain != self.DRIVE_4WD:
            print("run_rear_right_motor_to_position is only available on 4WD.")
            return
        speed = self.check_speed(speed)
        self._direct(self.rear_right_motor).run_to_position(position, speed=speed, blocking=blocking)

    def stop_rear_right_motor(self):
        """Stop the rear-right motor (4WD only)."""
        if self.drivetrain != self.DRIVE_4WD:
            print("stop_rear_right_motor is only available on 4WD.")
            return
        self._direct(self.rear_right_motor).stop()

    # -------------------------------------------------------------------------
    # Grouped axle stop helpers (4WD only)
    # -------------------------------------------------------------------------

    def stop_front_motors(self):
        """Stop both front motors simultaneously (4WD only)."""
        if self.drivetrain != self.DRIVE_4WD:
            print("stop_front_motors is only available on 4WD.")
            return
        self._direct(self.front_left_motor).stop()
        self._direct(self.front_right_motor).stop()

    def stop_rear_motors(self):
        """Stop both rear motors simultaneously (4WD only)."""
        if self.drivetrain != self.DRIVE_4WD:
            print("stop_rear_motors is only available on 4WD.")
            return
        self._direct(self.rear_left_motor).stop()
        self._direct(self.rear_right_motor).stop()

    # -------------------------------------------------------------------------
    # Coordinated drive commands (2WD and 4WD)
    # -------------------------------------------------------------------------

    def run_motors_for_rotations(self, rotations, left_speed=50, right_speed=50):
        """Run both sides for a fixed number of rotations simultaneously.

        The slower side blocks the call; the faster side runs non-blocking so
        both sides start together and the function returns when both are done.

        Args:
            rotations (float): Number of rotations for each side.
            left_speed (float): Left side speed in RPM. Defaults to 50.
            right_speed (float): Right side speed in RPM. Defaults to 50.
        """
        left_speed = self.check_speed(left_speed)
        right_speed = self.check_speed(right_speed)

        abs_left_speed = abs(left_speed)
        abs_right_speed = abs(right_speed)

        if abs_right_speed >= abs_left_speed:
            self.run_right_motor_for_rotations(rotations, speed=right_speed, blocking=False)
            self.run_left_motor_for_rotations(rotations, speed=left_speed, blocking=True)
        else:
            self.run_left_motor_for_rotations(rotations, speed=left_speed, blocking=False)
            self.run_right_motor_for_rotations(rotations, speed=right_speed, blocking=True)

    def run_motors_for_seconds(self, seconds, left_speed=50, right_speed=50):
        """Run both sides for a fixed duration simultaneously.

        Left side runs non-blocking; right side blocks so the call returns
        when both sides have finished.

        Args:
            seconds (float): Duration to run in seconds.
            left_speed (float): Left side speed in RPM. Defaults to 50.
            right_speed (float): Right side speed in RPM. Defaults to 50.
        """
        left_speed = self.check_speed(left_speed)
        right_speed = self.check_speed(right_speed)
        self.run_left_motor_for_seconds(seconds, speed=left_speed, blocking=False)
        self.run_right_motor_for_seconds(seconds, speed=right_speed, blocking=True)

    # -------------------------------------------------------------------------
    # Encoder readers
    # -------------------------------------------------------------------------

    def get_encoder_readings(self):
        """Return accumulated encoder readings for all drive motors.

        Returns:
            list: For 2WD, [left_radians, right_radians].
                  For 4WD, [FL_radians, RL_radians, FR_radians, RR_radians].
        """
        if self.drivetrain == self.DRIVE_2WD:
            e = self._encoders()
            return [e['left'], e['right']]
        else:
            e = self._encoders()
            return [e['front_left'], e['rear_left'],
                    e['front_right'], e['rear_right']]

    def get_left_encoder_reading(self):
        """Return accumulated encoder reading(s) for the left side.

        Returns:
            float: Accumulated radians for 2WD.
            list: [front_left_radians, rear_left_radians] for 4WD.
        """
        if self.drivetrain == self.DRIVE_2WD:
            return self._encoders()['left']
        else:
            e = self._encoders()
            return [e['front_left'], e['rear_left']]

    def get_right_encoder_reading(self):
        """Return accumulated encoder reading(s) for the right side.

        Returns:
            float: Accumulated radians for 2WD.
            list: [front_right_radians, rear_right_radians] for 4WD.
        """
        if self.drivetrain == self.DRIVE_2WD:
            return self._encoders()['right']
        else:
            e = self._encoders()
            return [e['front_right'], e['rear_right']]

    def get_front_left_encoder_reading(self):
        """Return the accumulated encoder reading for the front-left motor (4WD only).

        Returns:
            float: Accumulated radians, or None if not in 4WD mode.
        """
        if self.drivetrain != self.DRIVE_4WD:
            print("get_front_left_encoder_reading is only available on 4WD.")
            return None
        return self._encoders()['front_left']

    def get_rear_left_encoder_reading(self):
        """Return the accumulated encoder reading for the rear-left motor (4WD only).

        Returns:
            float: Accumulated radians, or None if not in 4WD mode.
        """
        if self.drivetrain != self.DRIVE_4WD:
            print("get_rear_left_encoder_reading is only available on 4WD.")
            return None
        return self._encoders()['rear_left']

    def get_front_right_encoder_reading(self):
        """Return the accumulated encoder reading for the front-right motor (4WD only).

        Returns:
            float: Accumulated radians, or None if not in 4WD mode.
        """
        if self.drivetrain != self.DRIVE_4WD:
            print("get_front_right_encoder_reading is only available on 4WD.")
            return None
        return self._encoders()['front_right']

    def get_rear_right_encoder_reading(self):
        """Return the accumulated encoder reading for the rear-right motor (4WD only).

        Returns:
            float: Accumulated radians, or None if not in 4WD mode.
        """
        if self.drivetrain != self.DRIVE_4WD:
            print("get_rear_right_encoder_reading is only available on 4WD.")
            return None
        return self._encoders()['rear_right']

    # -------------------------------------------------------------------------
    # Shutdown
    # -------------------------------------------------------------------------

    def stop_motors(self):
        """Stop all drive motors immediately.

        For 2WD, stops left and right motors.
        For 4WD, stops all four motors.
        """
        if self.drivetrain == self.DRIVE_2WD:
            self._direct(self.left_motor).stop()
            self._direct(self.right_motor).stop()
        else:
            self._direct(self.front_left_motor).stop()
            self._direct(self.rear_left_motor).stop()
            self._direct(self.front_right_motor).stop()
            self._direct(self.rear_right_motor).stop()

    def disconnect_robot(self):
        """Safely shut down all robot subsystems.

        Stops all motors, halts the lidar and camera if enabled, terminates
        the encoder tracking thread, and stops the IMU. Safe to call more than
        once; repeat calls return immediately.
        """
        if self.disconnected:
            return
        self.disconnected = True
        # Drop pending speeds before stopping, so nothing can be flushed back
        # onto the hardware afterwards. step() also refuses to run once
        # disconnected, which is what makes `while bot.step(dt):` terminate.
        with self._command_lock:
            self._pending_speeds.clear()
        self.stop_motors()
        if self.lidar is not None:
            self.lidar.stop_lidar()
        if self.camera is not None:
            self.camera.stop_camera()
        self.stop_thread = True
        self.position_thread.join()
        self.imu.stop()

    def shutdown(self, signum, frame):
        """Handle SIGINT/SIGTERM by gracefully disconnecting and exiting.

        Args:
            signum (int): Signal number received.
            frame: Current stack frame (unused).
        """
        print("Shutdown signal received. Stopping motors...")
        self.disconnect_robot()
        sys.exit(0)
