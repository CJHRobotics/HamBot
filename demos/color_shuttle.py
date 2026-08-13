"""Shuttle between two colored cylinders (blue <-> pink).

Behavior:
    1. Search for the current target color by rotating in place.
    2. When the target landmark is visible, drive forward while steering
       to keep it centered in the frame.
    3. When the front lidar reads closer than STOP_MM, stop, back off a
       hair, switch to the other color, and repeat.

Tuning knobs live at the top of the file. Colors especially will need
to be measured against your actual cylinders under your lighting.

Run with --show to pop up a live camera feed with the detected bounding
box drawn on top (requires a display / X forwarding).
"""

import argparse
import time

from robot_systems.robot import HamBot
from robot_systems.camera import Camera


# --- Target colors (OpenCV HSV: H 0-179, S 0-255, V 0-255) ---------------
# Measure these on-robot with your cylinders in the actual lighting using
# demos/cameraGUI.py (click a pixel — it prints the HSV values).
# Only H is used for matching; S and V here are informational.
BLUE_HSV = (110, 200, 150)
PINK_HSV = (170, 180, 200)
HUE_TOLERANCE  = 15   # ± hue units per target (OpenCV H ranges 0-179)
MIN_SATURATION = 80   # reject washed-out pixels
MIN_VALUE      = 60   # reject shadows / near-black

# --- Motion ---------------------------------------------------------------
CRUISE_RPM = 40          # forward speed while approaching a target
SEARCH_RPM = 20          # in-place rotation speed while searching
# Camera is mounted upside-down, so a target on the LEFT of the physical
# scene appears on the RIGHT of the (rotate_180-corrected) frame's horizontal
# axis inverted relative to steering. Negate STEER_KP to invert the response.
# Flip the sign again if you re-mount the camera right-side-up.
STEER_KP   = -0.06       # RPM per pixel of horizontal error

# --- Lidar ----------------------------------------------------------------
# Index 180 is straight ahead (per robot_systems.lidar); sample a small
# window around it and take the closest valid reading.
FRONT_WINDOW = range(170, 191)
STOP_MM      = 300       # stop when the nearest front reading is <= this

# --- Camera ---------------------------------------------------------------
FRAME_WIDTH  = 640       # PiCamera default; matches HamBot's PiCamera init
MIN_LM_AREA  = 800       # pixels; ignore small blobs of matching color

# --- Timing ---------------------------------------------------------------
LOOP_HZ           = 10
SEARCH_TIMEOUT_S  = 20    # give up on a target after this long searching
POST_STOP_PAUSE_S = 1.0   # rest a moment before hunting the next color

WINDOW_NAME = "HamBot camera"


def nearest_front_mm(scan):
    """Return the closest valid front-facing lidar reading in mm, or None."""
    valid = [scan[i] for i in FRONT_WINDOW if scan[i] > 0]
    return min(valid) if valid else None


def hsv_to_bgr(hsv):
    """Convert a single (H, S, V) tuple (OpenCV ranges) to a BGR tuple for drawing."""
    import cv2, numpy as np
    px = np.uint8([[list(hsv)]])
    b, g, r = cv2.cvtColor(px, cv2.COLOR_HSV2BGR)[0, 0]
    return int(b), int(g), int(r)


def render_view(bot, landmarks, target_name, target_hsv, front_mm):
    """Draw the current frame with bounding boxes and status. Returns True
    unless the user pressed 'q' in the OpenCV window."""
    import cv2  # local import so headless runs don't require GUI cv2

    frame_rgb = bot.camera.get_frame(copy=False)
    if frame_rgb is None:
        return True

    view = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)
    box_bgr = hsv_to_bgr(target_hsv)

    for i, lm in enumerate(landmarks):
        x0 = int(lm.x - lm.width // 2)
        y0 = int(lm.y - lm.height // 2)
        x1 = int(lm.x + lm.width // 2)
        y1 = int(lm.y + lm.height // 2)
        thickness = 3 if i == 0 else 1
        cv2.rectangle(view, (x0, y0), (x1, y1), box_bgr, thickness)
        cv2.circle(view, (int(lm.x), int(lm.y)), 4, box_bgr, -1)

    label = f"target: {target_name}  landmarks: {len(landmarks)}"
    if front_mm is not None:
        label += f"  front: {front_mm} mm"
    cv2.putText(view, label, (8, 22), cv2.FONT_HERSHEY_SIMPLEX,
                0.6, (255, 255, 255), 2, cv2.LINE_AA)

    # Camera is mounted upside-down; rotate only the display so the operator
    # sees the world right-side-up. Detection/steering still runs on the
    # un-rotated frame (see the STEER_KP sign note above).
    view = cv2.rotate(view, cv2.ROTATE_180)

    cv2.imshow(WINDOW_NAME, view)
    return cv2.waitKey(1) & 0xFF != ord("q")


def approach_target(bot, target_name, target_hsv, show=False):
    """Rotate to find target_hsv, then drive to it. Returns True on arrival."""
    bot.camera.set_target_colors(target_hsv,
                                 hue_tolerance=HUE_TOLERANCE,
                                 min_saturation=MIN_SATURATION,
                                 min_value=MIN_VALUE)
    frame_center_x = FRAME_WIDTH // 2
    period = 1.0 / LOOP_HZ

    searching_since = time.time()
    saw_target = False

    while True:
        landmarks = bot.camera.find_landmarks(min_area=MIN_LM_AREA)
        scan = bot.get_range_image()
        front = nearest_front_mm(scan) if scan != -1 else None

        if show and not render_view(bot, landmarks, target_name, target_hsv, front):
            bot.stop_motors()
            print("  quit requested from viewer")
            return False

        if not landmarks:
            if time.time() - searching_since > SEARCH_TIMEOUT_S:
                bot.stop_motors()
                msg = "lost target" if saw_target else "never spotted target"
                print(f"  {msg}, giving up on this leg")
                return False
            # rotate in place (CCW: left back, right forward)
            bot.set_left_motor_speed(-SEARCH_RPM)
            bot.set_right_motor_speed(SEARCH_RPM)
            time.sleep(period)
            continue

        saw_target = True
        searching_since = time.time()
        lm = landmarks[0]  # largest matching blob

        if front is not None and front <= STOP_MM:
            bot.stop_motors()
            print(f"  arrived ({front} mm)")
            return True

        # P-controller: steer to bring landmark to center
        error = lm.x - frame_center_x
        correction = STEER_KP * error
        left  = CRUISE_RPM + correction  # target-right -> left speeds up -> turn right
        right = CRUISE_RPM - correction
        bot.set_left_motor_speed(left)
        bot.set_right_motor_speed(right)

        time.sleep(period)


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--show", action="store_true",
                        help="open a live camera window with detected bounding boxes")
    args = parser.parse_args()

    if args.show:
        import cv2
        cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_AUTOSIZE)

    bot = HamBot(drivetrain=HamBot.DRIVE_2WD,
                 lidar_enabled=True,
                 camera_enabled=True,
                 camera_type=Camera.CAM_PICAM)

    # Let sensors warm up
    time.sleep(2.0)

    targets = [("blue", BLUE_HSV), ("pink", PINK_HSV)]
    idx = 0

    try:
        while True:
            name, hsv = targets[idx]
            print(f"heading to {name} cylinder...")
            approach_target(bot, name, hsv, show=args.show)

            # Small backup so we don't clip the cylinder on the next rotate
            bot.run_motors_for_seconds(0.4, left_speed=-25, right_speed=-25)
            time.sleep(POST_STOP_PAUSE_S)

            idx = 1 - idx  # toggle target
    finally:
        bot.disconnect_robot()
        if args.show:
            import cv2
            cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
