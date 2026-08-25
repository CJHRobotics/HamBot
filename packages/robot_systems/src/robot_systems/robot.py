import math, time, threading, sys, signal
from buildhat import Motor
from robot_systems.imu import IMU
from robot_systems.lidar import Lidar
from robot_systems.camera import Camera

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
    # Sensors
    # -------------------------------------------------------------------------

    def get_range_image(self):
        """Return the current 360° lidar scan as a list of distance measurements.

        Returns:
            list: 360 distance values (mm) indexed by degree, where 180° is the
                  front of the robot and 0° is the rear. Returns -1 if the lidar
                  is not enabled.
        """
        if self.lidar is not None:
            return self.lidar.get_current_scan()
        else:
            print("Lidar is not enabled.")
            return -1

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
        return self.imu.get_heading(fresh_within=fresh_within,
                                    blocking=blocking,
                                    wait_timeout=wait_timeout)

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

    def reset_encoders(self):
        """Reset all encoder accumulators to zero and re-snapshot current positions.

        For 2WD resets left and right. For 4WD resets all four wheels individually.
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
            self.left_motor.start(speed=speed_rpm)
        else:
            self.front_left_motor.start(speed=speed_rpm)
            self.rear_left_motor.start(speed=speed_rpm)

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
            self.left_motor.run_for_seconds(seconds, speed=speed, blocking=blocking)
        else:
            self.front_left_motor.run_for_seconds(seconds, speed=speed, blocking=False)
            self.rear_left_motor.run_for_seconds(seconds, speed=speed, blocking=blocking)

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
            self.left_motor.run_for_rotations(rotations, speed=speed, blocking=blocking)
        else:
            self.front_left_motor.run_for_rotations(rotations, speed=speed, blocking=False)
            self.rear_left_motor.run_for_rotations(rotations, speed=speed, blocking=blocking)

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
            self.left_motor.run_to_position(position, speed=speed, blocking=blocking)
        else:
            self.front_left_motor.run_to_position(position, speed=speed, blocking=False)
            self.rear_left_motor.run_to_position(position, speed=speed, blocking=blocking)

    def stop_left_motor(self):
        """Stop the left side motor(s).

        For 4WD, stops both front-left and rear-left motors.
        """
        if self.drivetrain == self.DRIVE_2WD:
            self.left_motor.stop()
        else:
            self.front_left_motor.stop()
            self.rear_left_motor.stop()

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
            self.right_motor.start(speed=speed_rpm)
        else:
            self.front_right_motor.start(speed=speed_rpm)
            self.rear_right_motor.start(speed=speed_rpm)

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
            self.right_motor.run_for_seconds(seconds, speed=speed, blocking=blocking)
        else:
            self.front_right_motor.run_for_seconds(seconds, speed=speed, blocking=False)
            self.rear_right_motor.run_for_seconds(seconds, speed=speed, blocking=blocking)

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
            self.right_motor.run_for_rotations(rotations, speed=speed, blocking=blocking)
        else:
            self.front_right_motor.run_for_rotations(rotations, speed=speed, blocking=False)
            self.rear_right_motor.run_for_rotations(rotations, speed=speed, blocking=blocking)

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
            self.right_motor.run_to_position(position, speed=speed, blocking=blocking)
        else:
            self.front_right_motor.run_to_position(position, speed=speed, blocking=False)
            self.rear_right_motor.run_to_position(position, speed=speed, blocking=blocking)

    def stop_right_motor(self):
        """Stop the right side motor(s).

        For 4WD, stops both front-right and rear-right motors.
        """
        if self.drivetrain == self.DRIVE_2WD:
            self.right_motor.stop()
        else:
            self.front_right_motor.stop()
            self.rear_right_motor.stop()

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
        self.front_left_motor.start(speed=speed_rpm)

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
        self.front_left_motor.run_for_seconds(seconds, speed=speed, blocking=blocking)

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
        self.front_left_motor.run_for_rotations(rotations, speed=speed, blocking=blocking)

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
        self.front_left_motor.run_to_position(position, speed=speed, blocking=blocking)

    def stop_front_left_motor(self):
        """Stop the front-left motor (4WD only)."""
        if self.drivetrain != self.DRIVE_4WD:
            print("stop_front_left_motor is only available on 4WD.")
            return
        self.front_left_motor.stop()

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
        self.rear_left_motor.start(speed=speed_rpm)

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
        self.rear_left_motor.run_for_seconds(seconds, speed=speed, blocking=blocking)

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
        self.rear_left_motor.run_for_rotations(rotations, speed=speed, blocking=blocking)

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
        self.rear_left_motor.run_to_position(position, speed=speed, blocking=blocking)

    def stop_rear_left_motor(self):
        """Stop the rear-left motor (4WD only)."""
        if self.drivetrain != self.DRIVE_4WD:
            print("stop_rear_left_motor is only available on 4WD.")
            return
        self.rear_left_motor.stop()

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
        self.front_right_motor.start(speed=speed_rpm)

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
        self.front_right_motor.run_for_seconds(seconds, speed=speed, blocking=blocking)

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
        self.front_right_motor.run_for_rotations(rotations, speed=speed, blocking=blocking)

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
        self.front_right_motor.run_to_position(position, speed=speed, blocking=blocking)

    def stop_front_right_motor(self):
        """Stop the front-right motor (4WD only)."""
        if self.drivetrain != self.DRIVE_4WD:
            print("stop_front_right_motor is only available on 4WD.")
            return
        self.front_right_motor.stop()

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
        self.rear_right_motor.start(speed=speed_rpm)

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
        self.rear_right_motor.run_for_seconds(seconds, speed=speed, blocking=blocking)

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
        self.rear_right_motor.run_for_rotations(rotations, speed=speed, blocking=blocking)

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
        self.rear_right_motor.run_to_position(position, speed=speed, blocking=blocking)

    def stop_rear_right_motor(self):
        """Stop the rear-right motor (4WD only)."""
        if self.drivetrain != self.DRIVE_4WD:
            print("stop_rear_right_motor is only available on 4WD.")
            return
        self.rear_right_motor.stop()

    # -------------------------------------------------------------------------
    # Grouped axle stop helpers (4WD only)
    # -------------------------------------------------------------------------

    def stop_front_motors(self):
        """Stop both front motors simultaneously (4WD only)."""
        if self.drivetrain != self.DRIVE_4WD:
            print("stop_front_motors is only available on 4WD.")
            return
        self.front_left_motor.stop()
        self.front_right_motor.stop()

    def stop_rear_motors(self):
        """Stop both rear motors simultaneously (4WD only)."""
        if self.drivetrain != self.DRIVE_4WD:
            print("stop_rear_motors is only available on 4WD.")
            return
        self.rear_left_motor.stop()
        self.rear_right_motor.stop()

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
            return [self.left_motor_radians, self.right_motor_radians]
        else:
            return [self.front_left_motor_radians, self.rear_left_motor_radians,
                    self.front_right_motor_radians, self.rear_right_motor_radians]

    def get_left_encoder_reading(self):
        """Return accumulated encoder reading(s) for the left side.

        Returns:
            float: Accumulated radians for 2WD.
            list: [front_left_radians, rear_left_radians] for 4WD.
        """
        if self.drivetrain == self.DRIVE_2WD:
            return self.left_motor_radians
        else:
            return [self.front_left_motor_radians, self.rear_left_motor_radians]

    def get_right_encoder_reading(self):
        """Return accumulated encoder reading(s) for the right side.

        Returns:
            float: Accumulated radians for 2WD.
            list: [front_right_radians, rear_right_radians] for 4WD.
        """
        if self.drivetrain == self.DRIVE_2WD:
            return self.right_motor_radians
        else:
            return [self.front_right_motor_radians, self.rear_right_motor_radians]

    def get_front_left_encoder_reading(self):
        """Return the accumulated encoder reading for the front-left motor (4WD only).

        Returns:
            float: Accumulated radians, or None if not in 4WD mode.
        """
        if self.drivetrain != self.DRIVE_4WD:
            print("get_front_left_encoder_reading is only available on 4WD.")
            return None
        return self.front_left_motor_radians

    def get_rear_left_encoder_reading(self):
        """Return the accumulated encoder reading for the rear-left motor (4WD only).

        Returns:
            float: Accumulated radians, or None if not in 4WD mode.
        """
        if self.drivetrain != self.DRIVE_4WD:
            print("get_rear_left_encoder_reading is only available on 4WD.")
            return None
        return self.rear_left_motor_radians

    def get_front_right_encoder_reading(self):
        """Return the accumulated encoder reading for the front-right motor (4WD only).

        Returns:
            float: Accumulated radians, or None if not in 4WD mode.
        """
        if self.drivetrain != self.DRIVE_4WD:
            print("get_front_right_encoder_reading is only available on 4WD.")
            return None
        return self.front_right_motor_radians

    def get_rear_right_encoder_reading(self):
        """Return the accumulated encoder reading for the rear-right motor (4WD only).

        Returns:
            float: Accumulated radians, or None if not in 4WD mode.
        """
        if self.drivetrain != self.DRIVE_4WD:
            print("get_rear_right_encoder_reading is only available on 4WD.")
            return None
        return self.rear_right_motor_radians

    # -------------------------------------------------------------------------
    # Shutdown
    # -------------------------------------------------------------------------

    def stop_motors(self):
        """Stop all drive motors immediately.

        For 2WD, stops left and right motors.
        For 4WD, stops all four motors.
        """
        if self.drivetrain == self.DRIVE_2WD:
            self.left_motor.stop()
            self.right_motor.stop()
        else:
            self.front_left_motor.stop()
            self.rear_left_motor.stop()
            self.front_right_motor.stop()
            self.rear_right_motor.stop()

    def disconnect_robot(self):
        """Safely shut down all robot subsystems.

        Stops all motors, halts the lidar and camera if enabled, terminates
        the encoder tracking thread, and stops the IMU. Safe to call more than
        once; repeat calls return immediately.
        """
        if self.disconnected:
            return
        self.disconnected = True
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
