#!/usr/bin/python3

"""Default post is all legs pointing straight down, with the body level.
Servos should be close to 90 deegrees pulse to reduce the amount of offset required"""

import rospy
import math
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import curses
from curses import wrapper
from yaml import dump
try:
    from yaml import CDumper as Dumper
except ImportError:
    from yaml import  Dumper

JOINT_CONTROLLER_TOPIC = "joint_group_position_controller/command"
OUT_QUEUE_SIZE = 1

"""We don't want to start the curses GUI until the servo_interface node has started"""
class SubscriberListener(rospy.SubscribeListener):
    def __init__(self, servo_calibration):
        super(SubscriberListener, self).__init__()
        self.servo_calibration = servo_calibration

    def peer_subscribe(self, topic_name, topic_publish, peer_publish):
        self.servo_calibration.subscriber_started = True
        # rospy.loginfo("Subscriber connected")

    def peer_unsubscribe(self, topic_name, num_peers):
        rospy.loginfo("Subscriber disconnected")


class ServoCalibration:
    def __init__(self):
        super().__init__()
        self.subscriber_started = False

        rospy.init_node("servo_calibration")
        
        self.selected = 0
        self.servo_calibration = rospy.get_param("servo_calibration")
        self.servo_calibration_original = rospy.get_param("servo_calibration")
        self.servo_calibration_file = rospy.get_param("servo_calibration_file")
        self.hardware_connected = rospy.get_param("hardware_connected")
        self.joint_names = rospy.get_param("/hardware_interface/joints")
        self.max_angle = rospy.get_param("servo_max_angle")
        self.servo_states_topic = rospy.Publisher(
            JOINT_CONTROLLER_TOPIC, JointTrajectory,
            queue_size=OUT_QUEUE_SIZE, 
            subscriber_listener=SubscriberListener(self),
            latch=True
        )

        rospy.SubscribeListener()

    def wait_for_subscriber(self):
        rospy.loginfo("Waiting for subscriber...")
        while not self.subscriber_started:
            rospy.sleep(.1)
            if rospy.is_shutdown():
                rospy.loginfo("Exiting...")
                exit()
        # Wait for logs to output
        rospy.sleep(.5)

    def init_gui(self, stdscr):
        curses.noecho()
        curses.cbreak()
        stdscr.keypad(True)

    def draw_servos(self, stdscr):
        rows, cols = stdscr.getmaxyx()

        stdscr.addstr(
            0, 0, "Servo Joint             Angle".ljust(cols), curses.A_REVERSE
        )

        for i, joint_name in enumerate(self.joint_names):
            row = i + 2
            joint = self.servo_calibration[joint_name]
            angle = joint["angle"]
            multiplier = joint["multiplier"]
            selected = i == self.selected
            color = curses.A_REVERSE if selected else curses.A_NORMAL
            d = dict(i=i, joint_name=joint_name, angle=angle)
            stdscr.addstr(row, 0, "{i:<2}: {joint_name:<20} {angle:<10}".format(**d), color)
            # TODO: Figure out if we can just append to the row instead of calculating the length of the string
            stdscr.addstr(row, 35, self.degrees_to_bars(cols - 40, angle), color)
        
        self.publish_joint_trajectories()
            
    def publish_joint_trajectories(self):
        # if len(positions) != len(self.joint_names):
        #     rospy.logerr("Number of positions does not match number of joints")
        #     return
        positions = []
        for joint_name in self.joint_names:
            joint = self.servo_calibration[joint_name]
            angle = joint["angle"]
            multiplier = joint["multiplier"]
            # Calulate the offset compared to the initial calibration
            if multiplier > 0:
                angle_diff = angle - self.servo_calibration_original[joint_name]["angle"]
            else:
                angle_diff = self.servo_calibration_original[joint_name]["angle"] - angle
            positions.append(math.radians(angle_diff))

        vel_cmd = JointTrajectory()
        vel_cmd.header.stamp = rospy.Time.now()
        # vel_cmd.name = self.joint_names
        vel_cmd.joint_names = self.joint_names
        jtp = JointTrajectoryPoint()
        jtp.positions = positions
        vel_cmd.points.append(jtp)
        self.servo_states_topic.publish(vel_cmd)

    def draw_menu(self, stdscr):
        stdscr.addstr(15, 0, "UP   / DOWN   to select servo ", curses.A_REVERSE)
        stdscr.addstr(16, 0, "LEFT / RIGHT  to adjust angle ", curses.A_REVERSE)
        # stdscr.addstr(17, 0, "LEFT / RIGHT  to adjust angle ", curses.A_REVERSE)

    def draw_gui(self, stdscr):
        stdscr.clear()
        while not rospy.is_shutdown():
            # stdscr.clear()
            self.draw_servos(stdscr)
            self.draw_menu(stdscr)
            stdscr.refresh()
            joint_name = self.joint_names[self.selected]

            # TODO: Toggle direction
            c = stdscr.getch()
            if c == ord("q"):
                return
            elif c == curses.KEY_UP:
                self.selected -= 1
            elif c == curses.KEY_DOWN:
                self.selected += 1
            elif c == curses.KEY_LEFT:
                self.servo_calibration[joint_name]["angle"] -= 1
            elif c == curses.KEY_RIGHT:
                self.servo_calibration[joint_name]["angle"] += 1
            elif c == 115:
                # s = Save
                dump(self.servo_calibration, open(self.servo_calibration_file, "w"), Dumper=Dumper)
                rospy.signal_shutdown("Saved servo calibration")
                return
            else:
                # time.sleep(5)
                stdscr.addstr(18, 0, "Key: {}".format(c))
            self.selected = max(0, min(len(self.joint_names) - 1, self.selected))

    def deinit_gui(self, stdscr):
        curses.nocbreak()
        stdscr.keypad(False)
        curses.echo()
        curses.endwin()

    def degrees_to_percent(self, degrees):
        return degrees / 180.0


    def degrees_to_bars(self, cols, degrees):
        bar_count = int(self.degrees_to_percent(degrees) * float(cols))
        return "[" + "#" * bar_count + " " * (cols - bar_count) + "]"


    def normalize_angle(self, degrees):
        return max(0, min(self.max_angle, degrees))


# def calibrate_servos():

#     print("Setting all servos to middle...")

#     for i in range(16):
#         pwm.servo[i].angle = 90


def main(stdscr):    
    sc.init_gui(stdscr)
    sc.draw_gui(stdscr)
    print("Done.")
    sc.deinit_gui(stdscr)


if __name__ == "__main__":
    sc = ServoCalibration()
    sc.wait_for_subscriber()
    wrapper(main)
