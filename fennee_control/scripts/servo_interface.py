#!/usr/bin/python3

"""Subscribes to the joint group position controller and converts the joint
positions to joint angles. The joint angles are then used to control the
servos.

We will reduce the frequency of the joint group position controller to 10Hz 
to avoid overflowing the serial buffer to the microcontroller.

http://wiki.ros.org/rospy/Overview/Time
"""

PUBLISH_FREQUENCY = 200  # Hz
SERVO_STATES_TOPIC = "servo_duty_cycles"
JOINT_STATES_TOPIC = "joint_states"
JOINT_CONTROLLER_TOPIC = "joint_group_position_controller/command"
OUT_QUEUE_SIZE = 10
IN_QUEUE_SIZE = 1
PCA_PWM_FREQUENCY = 50 # Hz for analog servos
PCA_ADDRESS = 0x40

import rospy
# import numpy as np
import math
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from std_msgs.msg import UInt16MultiArray
from adafruit_servokit import ServoKit
import time

# When multiplier is positive, the leg moves backwards as the angle increases

# JOINT_NAMES = [
#     "front_left_shoulder",    # 0
#     "front_left_leg",         # 1
#     "front_left_foot",        # 2
#     "front_right_shoulder",   # 3
#     "front_right_leg",        # 4
#     "front_right_foot",       # 5
#     "rear_left_shoulder",     # 6
#     "rear_left_leg",          # 7
#     "rear_left_foot",         # 8
#     "rear_right_shoulder",    # 9
#     "rear_right_leg",         # 10
#     "rear_right_foot",        # 11
# ]

class ServoInterface:
    def __init__(self):
        super().__init__()

        self.servo_calibration = rospy.get_param("servo_calibration")
        self.servo_calibration_file = rospy.get_param("servo_calibration_file")
        self.hardware_connected = rospy.get_param("hardware_connected")
        self.joint_names = rospy.get_param("/hardware_interface/joints")
        self.max_angle = rospy.get_param("servo_max_angle")

        # PCA9685 I2C PWM controller
        if self.hardware_connected:
            self.pwm = ServoKit(channels=16, address=PCA_ADDRESS) # , frequency=PCA_PWM_FREQUENCY
        else:
            rospy.loginfo("hardware_connected is false. Don't actually connect to I2C")
        # for i in range(0, 12):
        #     self.pwm.servo[i].actuation_range = MAX_ANGLE
        self.joint_positions = None
        self.servo_states_topic = rospy.Publisher(
            SERVO_STATES_TOPIC, UInt16MultiArray, queue_size=OUT_QUEUE_SIZE
        )
        self.joint_states_topic = rospy.Publisher(
            JOINT_STATES_TOPIC, JointState, queue_size=OUT_QUEUE_SIZE
        )
        self.msg = JointState()
        rospy.init_node("servo_interface")
        rospy.Subscriber(
            JOINT_CONTROLLER_TOPIC,
            JointTrajectory,
            self.handle_joint_commands,
            queue_size=IN_QUEUE_SIZE,
        )
        # Wait a moment for things to settle before publishing
        time.sleep(1.0)
        rospy.Timer(rospy.Duration(1.0 / PUBLISH_FREQUENCY), self.publish_positions)

    def servo_calibration_to_pwm_map(self):
        """Converts the servo calibration to a pwm map"""
        pwm_map = []
        for joint in self.joint_names:
            center_pwm = self.servo_calibration[joint]["angle"]
            multiplier = self.servo_calibration[joint]["multiplier"]
            pwm_map.append((center_pwm, multiplier))
        return pwm_map

    def check_connection(self):
        rospy.loginfo("Checking hardware connection")
        if not self.hardware_connected:
            rospy.loginfo("Hardware connection disabled")
            return
        try:
            self.pwm.servo[0].angle
        except OSError:
            rospy.logerr("ServoInterface: Failed to connect to PCA9685")
            self.hardware_connected = False
        rospy.loginfo("Looks good")

    def radians_to_pwm(self, angle, pwm_map_row):
        # TODO: Subtract from 360 if negative
        center_pwm, multiplier = pwm_map_row["angle"], pwm_map_row["multiplier"]
        pwm = (math.degrees(angle) * multiplier) + center_pwm
        if pwm < 0:
            pwm += 360
        pwm = int(pwm % 360)
        pwm = min(self.max_angle, pwm)
        return pwm

    def joint_states_to_pwms(self, angles):
        """Converts joint angles to pwm values"""
        # TODO: Can probably do a batch calculation for all servos with numpy
        pwms = [self.radians_to_pwm(angles[i], self.servo_calibration[a]) for i, a in enumerate(self.get_joint_names())]
        # remapped = [pwms[i] for i in CHANNEL_MAP] # No longer needed since the channels match
        return pwms

    def handle_joint_commands(self, data: JointTrajectory):
        # rospy.loginfo("handle_joint_commands")
        # rospy.loginfo(data)
        self.joint_positions = data
        angles = self.joint_states_to_pwms(data.points[0].positions)
        self.set_servo_positions(angles)


    def publish_positions(self, event):
        if not self.joint_positions:
            # No commands received yet
            return
        # print("Timer called at " + str(event.current_real))
        cmd = self.joint_positions.points[0]
        names = self.get_joint_names()
        self.publish_servo_positions(names, cmd)
        self.publish_joint_state(names, cmd)

    def publish_servo_positions(self, names, cmd: JointTrajectoryPoint):
        # rospy.loginfo("publish_servo_positions")
        # TODO: Transform joint positions to servo positions to pwm values
        angles = cmd.positions
        # print(self.get_joint_names())
        # print(dict(zip(self.get_joint_names(), angles)))
        angles = self.joint_states_to_pwms(angles)
        msg = UInt16MultiArray(data=tuple(int(abs(a)) for a in angles))
        self.servo_states_topic.publish(msg)

    def set_servo_positions(self, angles):
        # rospy.loginfo("set_servo_positions")
        # rospy.loginfo(angles)
        if self.hardware_connected:
            for i, angle in enumerate(angles):
                self.pwm.servo[i].angle = angle

    def publish_joint_state(self, names, cmd: JointTrajectoryPoint):
        msg = self.msg
        msg.header.stamp = rospy.Time.now()
        msg.name = names
        msg.position = tuple(cmd.positions)        
        self.joint_states_topic.publish(msg)

    def get_joint_names(self):
        # return self.joint_positions.joint_names
        return self.joint_names


if __name__ == "__main__":
    rospy.loginfo("Starting servo_interface...")
    servo_interface = ServoInterface()
    servo_interface.check_connection()
    rospy.loginfo("servo_interface started.")
    while not rospy.is_shutdown():
        rospy.spin()

