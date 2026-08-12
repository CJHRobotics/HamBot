import argparse
from sshkeyboard import listen_keyboard
from robot_systems.robot import HamBot
from robot_systems.camera import Camera

parser = argparse.ArgumentParser()
parser.add_argument('--drivetrain', choices=['2WD', '4WD'], default='2WD')
args = parser.parse_args()

robot = HamBot(
    drivetrain=args.drivetrain,
    lidar_enabled=True,
    camera_enabled=True,
    camera_type=Camera.CAM_PICAM
)

if robot.camera is not None:
    robot.camera.set_target_colors((0, 83, 155), tolerance=0.10)


def press(key):
    if key == "up":
        print("Robot Moving Forward")
        robot.set_left_motor_speed(50)
        robot.set_right_motor_speed(50)
    elif key == "down":
        print("Robot Moving Backwards")
        robot.set_left_motor_speed(-50)
        robot.set_right_motor_speed(-50)
    elif key == "left":
        print("Robot Rotating Left")
        robot.set_left_motor_speed(-50)
        robot.set_right_motor_speed(50)
    elif key == "right":
        print("Robot Rotating Right")
        robot.set_left_motor_speed(50)
        robot.set_right_motor_speed(-50)
    elif key == "c":
        print("--- Camera Test ---")
        if robot.camera is None:
            print("Camera not available.")
            return
        frame = robot.camera.get_frame()
        if frame is None:
            print("No frame captured yet.")
        else:
            print(f"Frame OK: shape={frame.shape}, dtype={frame.dtype}")
    elif key == "l":
        print("--- Landmark Detection Test ---")
        if robot.camera is None:
            print("Camera not available.")
            return
        landmarks = robot.camera.find_landmarks()
        if not landmarks:
            print("No landmarks detected.")
        else:
            for i, lm in enumerate(landmarks):
                print(f"  [{i}] {lm}")
    elif key == "q":
        print("Exiting...")
        robot.stop_motors()
        robot.disconnect_robot()
        exit()


def release(key):
    heading = robot.get_heading()
    print(f"Robot heading: {heading}")
    range_image = robot.get_range_image()
    print(f"Range image: {range_image[0], range_image[90], range_image[180], range_image[270]}")
    if key == "up":
        print("Up arrow released.")
        robot.stop_motors()
    elif key == "down":
        print("Down arrow released.")
        robot.stop_motors()
    elif key == "left":
        print("Left arrow released.")
        robot.stop_motors()
    elif key == "right":
        print("Right arrow released.")
        robot.stop_motors()


# Start listening for key presses and releases
listen_keyboard(on_press=press, on_release=release)
