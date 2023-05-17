#!/usr/bin/python3

# TODO: Publish to topic instead of writing to servos

"""This should publish to the JOINT_CONTROLLER_TOPIC so that the servo_interface will move the servos to the desired position.
Currently, however, we will just set positions to 0 directly so that we can assemble the robot without it moving around.
Ideally we would make this a GUI like the Champ UI so we can adjust each servo individually.
"""

from adafruit_servokit import ServoKit
import curses
from curses import wrapper
from yaml import load, dump


# from servo_interface import PWM_MAP, JOINT_NAMES, MAX_ANGLE

MAX_ANGLE = 180  # degrees

JOINT_NAMES = [
    "front_left_shoulder",
    "front_left_leg",
    "front_left_foot",
    # "front_left_toe",
    "front_right_shoulder",
    "front_right_leg",
    "front_right_foot",
    # "front_right_toe",
    "rear_left_shoulder",
    "rear_left_leg",
    "rear_left_foot",
    # "rear_left_toe",
    "rear_right_shoulder",
    "rear_right_leg",
    "rear_right_foot",
    # "rear_right_toe",
]


PWM_MAP = {
    "front_left_foot": {"angle": 152, "multiplier": 1.0},
    "front_left_leg": {"angle": 35, "multiplier": 1.0},
    "front_left_shoulder": {"angle": 102, "multiplier": 1.0},
    "front_right_foot": {"angle": 40, "multiplier": -1.0},
    "front_right_leg": {"angle": 155, "multiplier": -1.0},
    "front_right_shoulder": {"angle": 98, "multiplier": 1.0},
    "rear_left_foot": {"angle": 155, "multiplier": 1.0},
    "rear_left_leg": {"angle": 53, "multiplier": 1.0},
    "rear_left_shoulder": {"angle": 102, "multiplier": -1.0},
    "rear_right_foot": {"angle": 45, "multiplier": -1.0},
    "rear_right_leg": {"angle": 155, "multiplier": -1.0},
    "rear_right_shoulder": {"angle": 98, "multiplier": -1.0},
}


def degrees_to_percent(degrees):
    return degrees / 180.0


def degrees_to_bars(cols, degrees):
    bar_count = int(degrees_to_percent(degrees) * float(cols))
    return "[" + "#" * bar_count + " " * (cols - bar_count) + "]"


def normalize_angle(degrees):
    return max(0, min(MAX_ANGLE, degrees))


class ServoCalibration:
    def __init__(self):
        super().__init__()
        print("Initilizing I2C...")
        self.pwm = ServoKit(channels=16)
        print("I2C initialized.")
        self.selected = 0

    def init_gui(self, stdscr):
        curses.noecho()
        curses.cbreak()
        stdscr.keypad(True)

    def draw_servos(self, stdscr):
        rows, cols = stdscr.getmaxyx()

        stdscr.addstr(
            0, 0, "Servo Joint             Angle".ljust(cols), curses.A_REVERSE
        )

        for i, joint_name in enumerate(JOINT_NAMES):
            row = i + 2
            joint = PWM_MAP[joint_name]
            angle = joint["angle"]
            multiplier = joint["multiplier"]
            selected = i == self.selected
            color = curses.A_REVERSE if selected else curses.A_NORMAL
            d = dict(i=i, joint_name=joint_name, angle=angle)
            stdscr.addstr(row, 0, "{i:<2}: {joint_name:<20} {angle:<10}".format(**d), color)
            # TODO: Figure out if we can just append to the row instead of calculating the length of the string
            stdscr.addstr(row, 35, degrees_to_bars(cols - 40, angle), color)
            self.pwm.servo[i].angle = normalize_angle(angle)
            

    def draw_menu(self, stdscr):
        stdscr.addstr(15, 0, "UP   / DOWN   to select servo ", curses.A_REVERSE)
        stdscr.addstr(16, 0, "LEFT / RIGHT  to adjust angle ", curses.A_REVERSE)
        stdscr.addstr(17, 0, "LEFT / RIGHT  to adjust angle ", curses.A_REVERSE)

    def draw_gui(self, stdscr):
        stdscr.clear()
        while True:
            # stdscr.clear()
            self.draw_servos(stdscr)
            self.draw_menu(stdscr)
            stdscr.refresh()
            joint_name = JOINT_NAMES[self.selected]

            # TODO: Toggle direction
            c = stdscr.getch()
            if c == ord("q"):
                return
            elif c == curses.KEY_UP:
                self.selected -= 1
            elif c == curses.KEY_DOWN:
                self.selected += 1
            elif c == curses.KEY_LEFT:
                PWM_MAP[joint_name]["angle"] -= 1
            elif c == curses.KEY_RIGHT:
                PWM_MAP[joint_name]["angle"] -= 1
            elif c == 115:
                # s = Save
                # stdscr.addstr(31, 0, "Key: {}".format(c))
                dump(PWM_MAP, open("pwm_map.yaml", "w"))
                # print(dump(PWM_MAP))
                return
            else:
                # time.sleep(5)
                stdscr.addstr(18, 0, "Key: {}".format(c))
            self.selected = max(0, min(len(JOINT_NAMES) - 1, self.selected))

    def deinit_gui(self, stdscr):
        curses.nocbreak()
        stdscr.keypad(False)
        curses.echo()
        curses.endwin()


# def calibrate_servos():

#     print("Setting all servos to middle...")

#     for i in range(16):
#         pwm.servo[i].angle = 90


def main(stdscr):
    # calibrate_servos()
    sc = ServoCalibration()
    sc.init_gui(stdscr)
    sc.draw_gui(stdscr)
    print("Done.")
    sc.deinit_gui(stdscr)


if __name__ == "__main__":
    wrapper(main)
