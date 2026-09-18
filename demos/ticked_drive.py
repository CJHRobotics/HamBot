#!/usr/bin/env python3
"""Drive forward under a fixed control tick, stopping for obstacles.

The other four demos are free-running: they loop as fast as Python lets them
and read each sensor whenever they happen to need it. This one uses
``bot.step(dt)``, which is the shape a Webots controller has::

    while bot.step(DT):
        ...

Two things that loop gives you:

* **Coherence.** ``get_heading()`` and ``get_range_image()`` called after the
  same ``step()`` describe the same instant, because ``step()`` latched them
  together. Free-running, the heading could be from 40 ms after the scan.
* **Timing you can see.** The loop runs at a known rate, and when the body
  takes longer than ``DT`` the robot says so instead of silently running slow.

Run it with the robot on blocks the first time — it drives.

    python3 demos/ticked_drive.py
    python3 demos/ticked_drive.py --dt 0.1 --speed 25
"""

import argparse

from robot_systems.robot import HamBot

DT = 0.05                  # 20 Hz, matching the IMU and encoder pollers
CRUISE_RPM = 30
STOP_MM = 400              # back off if anything is this close in front
CLEAR_MM = 600             # start again once the way is this clear

# 180° is straight ahead; see docs/lidar.md.
FRONT_WINDOW = range(160, 201)


def nearest_front_mm(scan):
    """Closest valid front-facing reading in mm, or None if nothing is seen.

    Expired and invalid slots come back as -1, so they are filtered out here
    the same way demos/color_shuttle.py does it. A slot nothing has bounced off
    recently is not the same as a slot measured as empty, and treating -1 as a
    distance would drive the robot into whatever is actually there.
    """
    valid = [scan[i] for i in FRONT_WINDOW if scan[i] > 0]
    return min(valid) if valid else None


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--dt", type=float, default=DT,
                        help=f"seconds per control cycle (default {DT})")
    parser.add_argument("--speed", type=float, default=CRUISE_RPM,
                        help=f"cruise speed in RPM (default {CRUISE_RPM})")
    parser.add_argument("--ticks", type=int, default=0,
                        help="stop after this many cycles (0 = run until Ctrl-C)")
    args = parser.parse_args()

    bot = HamBot(lidar_enabled=True, camera_enabled=False)

    ticks = 0
    driving = False
    try:
        while bot.step(args.dt):
            ticks += 1

            # Both reads describe the instant this tick latched.
            scan = bot.get_range_image()
            heading = bot.get_heading()
            front = nearest_front_mm(scan) if scan != -1 else None

            if front is not None and front < STOP_MM:
                if driving:
                    bot.stop_motors()
                    driving = False
            elif front is None or front > CLEAR_MM:
                if not driving:
                    driving = True
                bot.set_left_motor_speed(args.speed)
                bot.set_right_motor_speed(args.speed)

            if ticks % 20 == 0:
                front_txt = f"{front:.0f} mm" if front is not None else "clear"
                head_txt = f"{heading:.0f}°" if heading is not None else "--"
                late = "" if bot.loop_is_keeping_up() else "  LATE"
                print(f"t={ticks:5d}  front={front_txt:>9}  heading={head_txt:>5}"
                      f"  overruns={bot.overruns}{late}")

            if args.ticks and ticks >= args.ticks:
                break
    except KeyboardInterrupt:
        # The signal handler in HamBot already stops the motors; this only
        # keeps the traceback off the screen.
        pass
    finally:
        bot.stop_motors()
        print(f"\n{ticks} cycles, {bot.overruns} overrun(s), "
              f"worst {bot.worst_overrun_s * 1000:.0f} ms late.")
        if bot.overruns:
            print("A loop body took longer than dt. Raise --dt, or do less "
                  "work per cycle. Cycles are never dropped, so the loop just "
                  "runs slower than you asked for.")
        bot.disconnect_robot()


if __name__ == "__main__":
    main()
