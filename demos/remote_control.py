"""Drive the HamBot with the keyboard over SSH.

Arrow keys steer, space stops, q quits. Works over SSH because sshkeyboard
reads stdin directly and does not require a graphical session.

Controls:
    Up / Down     forward / reverse
    Left / Right  turn in place
    Up + Left     arc forward-left  (combine held keys)
    space         stop
    +  /  -       increase / decrease cruise speed
    q             quit
"""

from sshkeyboard import listen_keyboard, stop_listening
from robot_systems.robot import HamBot

CRUISE_RPM_START = 40
CRUISE_RPM_MIN = 10
CRUISE_RPM_MAX = 75
CRUISE_RPM_STEP = 5
TURN_RATIO = 0.6  # inside-wheel speed as a fraction of cruise when arcing


class Teleop:
    def __init__(self):
        self.bot = HamBot(drivetrain=HamBot.DRIVE_2WD)
        self.cruise = CRUISE_RPM_START
        self.held = set()
        self._print_help()

    def _print_help(self):
        print(
            f"\n  HamBot remote — cruise={self.cruise} rpm"
            "\n  arrows=drive  space=stop  +/-=speed  q=quit\n"
        )

    def _apply(self):
        """Recompute wheel speeds from currently-held keys."""
        fwd = ("up" in self.held) - ("down" in self.held)      # -1, 0, +1
        turn = ("right" in self.held) - ("left" in self.held)  # -1, 0, +1

        if fwd == 0 and turn == 0:
            self.bot.stop_motors()
            return

        if fwd == 0:
            # Spin in place
            left = self.cruise * turn
            right = -self.cruise * turn
        else:
            base = self.cruise * fwd
            if turn == 0:
                left = right = base
            elif turn > 0:  # arc right → slow right wheel
                left = base
                right = base * TURN_RATIO
            else:           # arc left → slow left wheel
                left = base * TURN_RATIO
                right = base

        self.bot.set_left_motor_speed(left)
        self.bot.set_right_motor_speed(right)

    def on_press(self, key):
        if key == "q":
            stop_listening()
            return
        if key == "space":
            self.held.clear()
            self.bot.stop_motors()
            return
        if key == "+":
            self.cruise = min(CRUISE_RPM_MAX, self.cruise + CRUISE_RPM_STEP)
            print(f"  cruise = {self.cruise} rpm")
            self._apply()
            return
        if key == "-":
            self.cruise = max(CRUISE_RPM_MIN, self.cruise - CRUISE_RPM_STEP)
            print(f"  cruise = {self.cruise} rpm")
            self._apply()
            return
        if key in ("up", "down", "left", "right"):
            self.held.add(key)
            self._apply()

    def on_release(self, key):
        if key in self.held:
            self.held.discard(key)
            self._apply()

    def run(self):
        try:
            listen_keyboard(
                on_press=self.on_press,
                on_release=self.on_release,
                delay_second_char=0.05,
                sequential=True,
            )
        finally:
            self.bot.disconnect_robot()


if __name__ == "__main__":
    Teleop().run()
