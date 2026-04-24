import math, time, threading, sys, signal
from buildhat import Motor
from robot_systems.imu import IMU
from robot_systems.lidar import Lidar
from robot_systems.camera import Camera

class HamBot:
    DRIVE_2WD = '2WD'
    DRIVE_4WD = '4WD'

    def __init__(self, drivetrain='2WD', lidar_enabled=True, camera_enabled=True):
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
            self.camera = Camera()
        else:
            self.camera = None

        self.stop_thread = False
        self.position_thread = threading.Thread(target=self.update_motor_positions)
        self.position_thread.start()

        signal.signal(signal.SIGINT, self.shutdown)

    def get_range_image(self):
        """
        Retrieve the current range image from the Lidar.

        Returns:
            list: A list of 360 distance measurements corresponding to each degree,
                  where 0° is towards the back of the Lidar, 90° is to the left,
                  180° is to the front, and 270° is to the right.
                  Returns -1 if the Lidar is not enabled.

        This function checks if the Lidar is enabled and then retrieves the latest
        scan data, which represents the distance measurements at each degree of rotation.
        If the Lidar is not enabled, it prints an error message and returns -1.
        """
        if self.lidar is not None:
            return self.lidar.get_current_scan()
        else:
            print("Lidar is not enabled.")
            return -1

    def get_heading(self, fresh_within=0.5, blocking=False, wait_timeout=0.3):
        return self.imu.get_heading(fresh_within=fresh_within,
                                    blocking=blocking,
                                    wait_timeout=wait_timeout)

    def _read_motor_delta(self, motor, last_position, invert=False):
        current = motor.get_position()
        delta = current - last_position
        if delta > 180:
            delta -= 360
        elif delta < -180:
            delta += 360
        return current, -math.radians(delta) if invert else math.radians(delta)

    def update_motor_positions(self):
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

    def check_speed(self, input_speed):
        if -self.MAX_RPM <= input_speed <= self.MAX_RPM:
            return input_speed
        elif input_speed < -self.MAX_RPM:
            print(f"Speed must be between -{self.MAX_RPM} and {self.MAX_RPM} revolutions per minute.")
            return -self.MAX_RPM
        elif input_speed > self.MAX_RPM:
            print(f"Speed must be between -{self.MAX_RPM} and {self.MAX_RPM} revolutions per minute.")
            return self.MAX_RPM

    def set_left_motor_speed(self, speed_rpm):
        speed_rpm *= -1
        speed_rpm = self.check_speed(speed_rpm)
        if self.drivetrain == self.DRIVE_2WD:
            self.left_motor.start(speed=speed_rpm)
        else:
            self.front_left_motor.start(speed=speed_rpm)
            self.rear_left_motor.start(speed=speed_rpm)

    def run_left_motor_for_seconds(self, seconds, speed=75, blocking=True):
        speed *= -1
        speed = self.check_speed(speed)
        if self.drivetrain == self.DRIVE_2WD:
            self.left_motor.run_for_seconds(seconds, speed=speed, blocking=blocking)
        else:
            self.front_left_motor.run_for_seconds(seconds, speed=speed, blocking=False)
            self.rear_left_motor.run_for_seconds(seconds, speed=speed, blocking=blocking)

    def run_left_motor_for_rotations(self, rotations, speed=75, blocking=True):
        speed *= -1
        speed = self.check_speed(speed)
        if self.drivetrain == self.DRIVE_2WD:
            self.left_motor.run_for_rotations(rotations, speed=speed, blocking=blocking)
        else:
            self.front_left_motor.run_for_rotations(rotations, speed=speed, blocking=False)
            self.rear_left_motor.run_for_rotations(rotations, speed=speed, blocking=blocking)

    def run_left_motor_to_position(self, position, speed=100, blocking=True):
        speed *= -1
        speed = self.check_speed(speed)
        if self.drivetrain == self.DRIVE_2WD:
            self.left_motor.run_to_position(position, speed=speed, blocking=blocking)
        else:
            self.front_left_motor.run_to_position(position, speed=speed, blocking=False)
            self.rear_left_motor.run_to_position(position, speed=speed, blocking=blocking)

    def stop_left_motor(self):
        if self.drivetrain == self.DRIVE_2WD:
            self.left_motor.stop()
        else:
            self.front_left_motor.stop()
            self.rear_left_motor.stop()

    def set_right_motor_speed(self, speed_rpm):
        speed_rpm = self.check_speed(speed_rpm)
        if self.drivetrain == self.DRIVE_2WD:
            self.right_motor.start(speed=speed_rpm)
        else:
            self.front_right_motor.start(speed=speed_rpm)
            self.rear_right_motor.start(speed=speed_rpm)

    def run_right_motor_for_seconds(self, seconds, speed=75, blocking=True):
        speed = self.check_speed(speed)
        if self.drivetrain == self.DRIVE_2WD:
            self.right_motor.run_for_seconds(seconds, speed=speed, blocking=blocking)
        else:
            self.front_right_motor.run_for_seconds(seconds, speed=speed, blocking=False)
            self.rear_right_motor.run_for_seconds(seconds, speed=speed, blocking=blocking)

    def run_right_motor_for_rotations(self, rotations, speed=75, blocking=True):
        speed = self.check_speed(speed)
        if self.drivetrain == self.DRIVE_2WD:
            self.right_motor.run_for_rotations(rotations, speed=speed, blocking=blocking)
        else:
            self.front_right_motor.run_for_rotations(rotations, speed=speed, blocking=False)
            self.rear_right_motor.run_for_rotations(rotations, speed=speed, blocking=blocking)

    def run_right_motor_to_position(self, position, speed=75, blocking=True):
        speed = self.check_speed(speed)
        if self.drivetrain == self.DRIVE_2WD:
            self.right_motor.run_to_position(position, speed=speed, blocking=blocking)
        else:
            self.front_right_motor.run_to_position(position, speed=speed, blocking=False)
            self.rear_right_motor.run_to_position(position, speed=speed, blocking=blocking)

    def stop_right_motor(self):
        if self.drivetrain == self.DRIVE_2WD:
            self.right_motor.stop()
        else:
            self.front_right_motor.stop()
            self.rear_right_motor.stop()

    # --- Front Left (Port C) ---

    def set_front_left_motor_speed(self, speed_rpm):
        if self.drivetrain != self.DRIVE_4WD:
            print("set_front_left_motor_speed is only available on 4WD.")
            return
        speed_rpm *= -1
        speed_rpm = self.check_speed(speed_rpm)
        self.front_left_motor.start(speed=speed_rpm)

    def run_front_left_motor_for_seconds(self, seconds, speed=75, blocking=True):
        if self.drivetrain != self.DRIVE_4WD:
            print("run_front_left_motor_for_seconds is only available on 4WD.")
            return
        speed *= -1
        speed = self.check_speed(speed)
        self.front_left_motor.run_for_seconds(seconds, speed=speed, blocking=blocking)

    def run_front_left_motor_for_rotations(self, rotations, speed=75, blocking=True):
        if self.drivetrain != self.DRIVE_4WD:
            print("run_front_left_motor_for_rotations is only available on 4WD.")
            return
        speed *= -1
        speed = self.check_speed(speed)
        self.front_left_motor.run_for_rotations(rotations, speed=speed, blocking=blocking)

    def run_front_left_motor_to_position(self, position, speed=100, blocking=True):
        if self.drivetrain != self.DRIVE_4WD:
            print("run_front_left_motor_to_position is only available on 4WD.")
            return
        speed *= -1
        speed = self.check_speed(speed)
        self.front_left_motor.run_to_position(position, speed=speed, blocking=blocking)

    def stop_front_left_motor(self):
        if self.drivetrain != self.DRIVE_4WD:
            print("stop_front_left_motor is only available on 4WD.")
            return
        self.front_left_motor.stop()

    # --- Rear Left (Port D) ---

    def set_rear_left_motor_speed(self, speed_rpm):
        if self.drivetrain != self.DRIVE_4WD:
            print("set_rear_left_motor_speed is only available on 4WD.")
            return
        speed_rpm *= -1
        speed_rpm = self.check_speed(speed_rpm)
        self.rear_left_motor.start(speed=speed_rpm)

    def run_rear_left_motor_for_seconds(self, seconds, speed=75, blocking=True):
        if self.drivetrain != self.DRIVE_4WD:
            print("run_rear_left_motor_for_seconds is only available on 4WD.")
            return
        speed *= -1
        speed = self.check_speed(speed)
        self.rear_left_motor.run_for_seconds(seconds, speed=speed, blocking=blocking)

    def run_rear_left_motor_for_rotations(self, rotations, speed=75, blocking=True):
        if self.drivetrain != self.DRIVE_4WD:
            print("run_rear_left_motor_for_rotations is only available on 4WD.")
            return
        speed *= -1
        speed = self.check_speed(speed)
        self.rear_left_motor.run_for_rotations(rotations, speed=speed, blocking=blocking)

    def run_rear_left_motor_to_position(self, position, speed=100, blocking=True):
        if self.drivetrain != self.DRIVE_4WD:
            print("run_rear_left_motor_to_position is only available on 4WD.")
            return
        speed *= -1
        speed = self.check_speed(speed)
        self.rear_left_motor.run_to_position(position, speed=speed, blocking=blocking)

    def stop_rear_left_motor(self):
        if self.drivetrain != self.DRIVE_4WD:
            print("stop_rear_left_motor is only available on 4WD.")
            return
        self.rear_left_motor.stop()

    # --- Front Right (Port B) ---

    def set_front_right_motor_speed(self, speed_rpm):
        if self.drivetrain != self.DRIVE_4WD:
            print("set_front_right_motor_speed is only available on 4WD.")
            return
        speed_rpm = self.check_speed(speed_rpm)
        self.front_right_motor.start(speed=speed_rpm)

    def run_front_right_motor_for_seconds(self, seconds, speed=75, blocking=True):
        if self.drivetrain != self.DRIVE_4WD:
            print("run_front_right_motor_for_seconds is only available on 4WD.")
            return
        speed = self.check_speed(speed)
        self.front_right_motor.run_for_seconds(seconds, speed=speed, blocking=blocking)

    def run_front_right_motor_for_rotations(self, rotations, speed=75, blocking=True):
        if self.drivetrain != self.DRIVE_4WD:
            print("run_front_right_motor_for_rotations is only available on 4WD.")
            return
        speed = self.check_speed(speed)
        self.front_right_motor.run_for_rotations(rotations, speed=speed, blocking=blocking)

    def run_front_right_motor_to_position(self, position, speed=100, blocking=True):
        if self.drivetrain != self.DRIVE_4WD:
            print("run_front_right_motor_to_position is only available on 4WD.")
            return
        speed = self.check_speed(speed)
        self.front_right_motor.run_to_position(position, speed=speed, blocking=blocking)

    def stop_front_right_motor(self):
        if self.drivetrain != self.DRIVE_4WD:
            print("stop_front_right_motor is only available on 4WD.")
            return
        self.front_right_motor.stop()

    # --- Rear Right (Port A) ---

    def set_rear_right_motor_speed(self, speed_rpm):
        if self.drivetrain != self.DRIVE_4WD:
            print("set_rear_right_motor_speed is only available on 4WD.")
            return
        speed_rpm = self.check_speed(speed_rpm)
        self.rear_right_motor.start(speed=speed_rpm)

    def run_rear_right_motor_for_seconds(self, seconds, speed=75, blocking=True):
        if self.drivetrain != self.DRIVE_4WD:
            print("run_rear_right_motor_for_seconds is only available on 4WD.")
            return
        speed = self.check_speed(speed)
        self.rear_right_motor.run_for_seconds(seconds, speed=speed, blocking=blocking)

    def run_rear_right_motor_for_rotations(self, rotations, speed=75, blocking=True):
        if self.drivetrain != self.DRIVE_4WD:
            print("run_rear_right_motor_for_rotations is only available on 4WD.")
            return
        speed = self.check_speed(speed)
        self.rear_right_motor.run_for_rotations(rotations, speed=speed, blocking=blocking)

    def run_rear_right_motor_to_position(self, position, speed=100, blocking=True):
        if self.drivetrain != self.DRIVE_4WD:
            print("run_rear_right_motor_to_position is only available on 4WD.")
            return
        speed = self.check_speed(speed)
        self.rear_right_motor.run_to_position(position, speed=speed, blocking=blocking)

    def stop_rear_right_motor(self):
        if self.drivetrain != self.DRIVE_4WD:
            print("stop_rear_right_motor is only available on 4WD.")
            return
        self.rear_right_motor.stop()

    # --- Grouped stop helpers (4WD only) ---

    def stop_front_motors(self):
        if self.drivetrain != self.DRIVE_4WD:
            print("stop_front_motors is only available on 4WD.")
            return
        self.front_left_motor.stop()
        self.front_right_motor.stop()

    def stop_rear_motors(self):
        if self.drivetrain != self.DRIVE_4WD:
            print("stop_rear_motors is only available on 4WD.")
            return
        self.rear_left_motor.stop()
        self.rear_right_motor.stop()

    def run_motors_for_rotations(self, rotations, left_speed=50, right_speed=50):
        """
        Run both motors for a specified number of rotations.

        Args:
            rotations (float): The number of rotations for each motor to perform.
            left_speed (float): The speed for the left motor in RPM (default is 50).
            right_speed (float): The speed for the right motor in RPM (default is 50).

        This method runs both motors for the given number of rotations, with the left motor running
        asynchronously and the right motor running synchronously to ensure accurate movement.
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
        """
        Run both motors for a specified number of seconds.

        Args:
            seconds (float): The duration in seconds to run both motors.
            left_speed (float): The speed for the left motor in RPM (default is 50).
            right_speed (float): The speed for the right motor in RPM (default is 50).

        This method runs both motors for the given duration, with the left motor running
        asynchronously and the right motor running synchronously to ensure consistent movement.
        """
        left_speed = self.check_speed(left_speed)
        right_speed = self.check_speed(right_speed)
        self.run_left_motor_for_seconds(seconds, speed=left_speed, blocking=False)
        self.run_right_motor_for_seconds(seconds, speed=right_speed, blocking=True)

    def get_encoder_readings(self):
        if self.drivetrain == self.DRIVE_2WD:
            return [self.left_motor_radians, self.right_motor_radians]
        else:
            return [self.front_left_motor_radians, self.rear_left_motor_radians,
                    self.front_right_motor_radians, self.rear_right_motor_radians]

    def get_left_encoder_reading(self):
        if self.drivetrain == self.DRIVE_2WD:
            return self.left_motor_radians
        else:
            return [self.front_left_motor_radians, self.rear_left_motor_radians]

    def get_right_encoder_reading(self):
        if self.drivetrain == self.DRIVE_2WD:
            return self.right_motor_radians
        else:
            return [self.front_right_motor_radians, self.rear_right_motor_radians]

    # 4WD individual encoder getters
    def get_front_left_encoder_reading(self):
        if self.drivetrain != self.DRIVE_4WD:
            print("get_front_left_encoder_reading is only available on 4WD.")
            return None
        return self.front_left_motor_radians

    def get_rear_left_encoder_reading(self):
        if self.drivetrain != self.DRIVE_4WD:
            print("get_rear_left_encoder_reading is only available on 4WD.")
            return None
        return self.rear_left_motor_radians

    def get_front_right_encoder_reading(self):
        if self.drivetrain != self.DRIVE_4WD:
            print("get_front_right_encoder_reading is only available on 4WD.")
            return None
        return self.front_right_motor_radians

    def get_rear_right_encoder_reading(self):
        if self.drivetrain != self.DRIVE_4WD:
            print("get_rear_right_encoder_reading is only available on 4WD.")
            return None
        return self.rear_right_motor_radians

    def stop_motors(self):
        if self.drivetrain == self.DRIVE_2WD:
            self.left_motor.stop()
            self.right_motor.stop()
        else:
            self.front_left_motor.stop()
            self.rear_left_motor.stop()
            self.front_right_motor.stop()
            self.rear_right_motor.stop()

    def disconnect_robot(self):
        self.stop_motors()
        if self.lidar is not None:
            self.lidar.stop_lidar()
        if self.camera is not None:
            self.camera.stop_camera()
        self.stop_thread = True
        self.position_thread.join()
        self.imu.stop()


    def shutdown(self, signum, frame):
        """
        Gracefully shutdown HamBot.
        """
        print("Shutdown signal received. Stopping motors...")
        self.disconnect_robot()
        sys.exit(0)