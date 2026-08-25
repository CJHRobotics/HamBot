"""Drive the HamBot with the keyboard over SSH.

Arrow keys steer, space stops, q quits. Works over SSH because sshkeyboard
reads stdin directly and does not require a graphical session.

Controls:
    Up / Down     forward / reverse
    Left / Right  spin in place
    space         stop
    +  /  -       increase / decrease cruise speed
    q             quit

One key drives at a time. A terminal sends no key-release events, so
sshkeyboard infers them from gaps in the terminal's auto-repeat and tracks a
single key: pressing a new one releases the previous. Arrow keys therefore
cannot be combined.
"""

from sshkeyboard import listen_keyboard, stop_listening
from robot_systems.robot import HamBot

CRUISE_RPM_START = 40
CRUISE_RPM_MIN = 10
CRUISE_RPM_MAX = 75
CRUISE_RPM_STEP = 5

# Left at sshkeyboard's defaults deliberately. A terminal emits a held key
# once, waits about half a second, and only then starts repeating it.
# RELEASE_AFTER_FIRST_CHAR has to outlast that gap; shortening it makes
# sshkeyboard synthesize a release before the first repeat arrives, which
# stutters the motors on every single key press.
RELEASE_AFTER_FIRST_CHAR = 0.75
RELEASE_AFTER_REPEAT = 0.05

# Wheel directions per key, as (left, right) multiples of the cruise speed.
DIRECTIONS = {
    "up": (1, 1),
    "down": (-1, -1),
    "left": (-1, 1),
    "right": (1, -1),
}


class Teleop:
    def __init__(self):
        self.bot = HamBot(drivetrain=HamBot.DRIVE_2WD)
        self.cruise = CRUISE_RPM_START
        self.driving = None   # key currently driving, or None
        self.applied = None   # last (left, right) written to the motors
        self._print_help()

    def _print_help(self):
        print(
            f"\n  HamBot remote — cruise={self.cruise} rpm"
            "\n  arrows=drive  space=stop  +/-=speed  q=quit\n"
        )

    def _apply(self, left, right):
        """Write wheel speeds, but only when they actually change.

        Anything that re-sends an unchanged speed floods the Build HAT's
        serial port, and the motors fall behind the keyboard.
        """
        if (left, right) == self.applied:
            return
        self.applied = (left, right)
        if left == 0 and right == 0:
            self.bot.stop_motors()
        else:
            self.bot.set_left_motor_speed(left)
            self.bot.set_right_motor_speed(right)

    def _drive(self, key):
        left, right = DIRECTIONS[key]
        self._apply(left * self.cruise, right * self.cruise)

    def stop(self):
        self.driving = None
        self._apply(0, 0)

    def on_press(self, key):
        if key == "q":
            stop_listening()
            return
        if key == "space":
            self.stop()
            return
        if key in ("+", "-"):
            step = CRUISE_RPM_STEP if key == "+" else -CRUISE_RPM_STEP
            self.cruise = max(CRUISE_RPM_MIN, min(CRUISE_RPM_MAX, self.cruise + step))
            print(f"  cruise = {self.cruise} rpm")
            if self.driving is not None:
                self._drive(self.driving)
            return
        if key in DIRECTIONS:
            self.driving = key
            self._drive(key)

    def on_release(self, key):
        # Pressing a new key releases the old one, so a release for a key that
        # is no longer driving has already been superseded.
        if key == self.driving:
            self.stop()

    def run(self):
        try:
            listen_keyboard(
                on_press=self.on_press,
                on_release=self.on_release,
                delay_second_char=RELEASE_AFTER_FIRST_CHAR,
                delay_other_chars=RELEASE_AFTER_REPEAT,
                sequential=True,
            )
        finally:
            self.bot.disconnect_robot()


if __name__ == "__main__":
    Teleop().run()
