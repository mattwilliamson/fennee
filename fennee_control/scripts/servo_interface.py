#!/usr/bin/python3

"""Subscribes to the joint group position controller and converts the joint
positions to joint angles. The joint angles are then used to control the
servos.

We will reduce the frequency of the joint group position controller to 10Hz 
to avoid overflowing the serial buffer to the microcontroller.

http://wiki.ros.org/rospy/Overview/Time
"""

FREQUENCY = 50  # Hz
# FREQUENCY = 1  # Hz
SERVO_STATES_TOPIC = "servo_duty_cycles"
JOINT_STATES_TOPIC = "joint_states"
JOINT_CONTROLLER_TOPIC = "joint_group_position_controller/command"
QUEUE_SIZE = 10
RAD_TO_DEG = 57.2958
DEGREES_PER_PWM = 180.0 / 285.0 # 285 pwm = 180 degrees


import rospy
import numpy as np
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from std_msgs.msg import UInt16MultiArray


# Manual calibration for now
pwm_map = [
    # min pwn, max pwm, sit pwm, stand pwm, sit angle, stand angle
    (100.0,     900.0,   400.0,    400.0,   0.0000000,   0.000000),  # front_left_shoulder
    (100.0,     900.0,   351.0,    314.0,   1.579998,    1.132637),  # front_left_leg
    (100.0,     900.0,   113.0,    256.0,   -2.576207,   -1.92721),  # front_left_foot
    (100.0,     900.0,   297.0,    297.0,   0.000000,    -0.00000),  # front_right_shoulder
    (100.0,     900.0,   260.0,    297.0,   1.579998,    1.132637),  # front_right_leg
    (100.0,     900.0,   505.0,    362.0,   -2.576207,   -1.92721),  # front_right_foot
    (100.0,     900.0,   328.0,    328.0,   0.0000000,   0.000000),  # rear_left_shoulder
    (100.0,     900.0,   383.0,    347.0,   1.579998,    1.132637),  # rear_left_leg
    (100.0,     900.0,   113.0,    256.0,   -2.576207,   -1.92721),  # rear_left_foot
    (100.0,     900.0,   318.0,    318.0,   0.0000000,   -0.00000),  # rear_right_shoulder
    (100.0,     900.0,   262.0,    299.0,   1.579998,    1.132637),  # rear_right_leg
    (100.0,     900.0,   509.0,    366.0,   -2.576207,   -1.92721),  # rear_right_foot
]

def joint_state_to_pwm(angle, pwm_map_row):
    min_pwm, max_pwm, sit_pwm, stand_pwm, sit_angle, stand_angle = pwm_map_row
    angle_range = (stand_angle - sit_angle)
    if angle_range == 0:
        pwm = sit_pwm
    else:
        pwm = ( (angle - sit_angle) / angle_range) * (stand_pwm - sit_pwm) + sit_pwm
    # Make sure the pwm is within the range
    if min_pwm > max_pwm:
        pwm = min(min_pwm, max(max_pwm, pwm))
    else:
        pwm = min(max_pwm, max(min_pwm, pwm))
    return abs(int(pwm))

def joint_states_to_pwms(angles):
    """Converts joint angles to pwm values"""
    angles = [joint_state_to_pwm(a, pwm_map[i]) for i, a in enumerate(angles)]
    return angles


class ServoInterface:
    def __init__(self):
        self.joint_positions = None
        self.servo_states_topic = rospy.Publisher(
            SERVO_STATES_TOPIC, UInt16MultiArray, queue_size=QUEUE_SIZE
        )
        self.joint_states_topic = rospy.Publisher(
            JOINT_STATES_TOPIC, JointState, queue_size=QUEUE_SIZE
        )
        rospy.init_node("servo_interface")
        rospy.Subscriber(
            JOINT_CONTROLLER_TOPIC,
            JointTrajectory,
            self.handle_joint_commands,
            queue_size=1,
        )
        rospy.Timer(rospy.Duration(1.0 / FREQUENCY), self.publish_positions)

    def handle_joint_commands(self, data: JointTrajectory):
        self.joint_positions = data


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
        # TODO: Transform joint positions to servo positions to pwm values
        angles = np.array(cmd.positions)
        # print(self.get_joint_names())
        # print(dict(zip(self.get_joint_names(), angles)))
        angles = joint_states_to_pwms(angles)
        msg = UInt16MultiArray(data=tuple(int(abs(a)) for a in angles))
        self.servo_states_topic.publish(msg)

    def publish_joint_state(self, names, cmd: JointTrajectoryPoint):
        msg = JointState(name=names, position=np.array(cmd.positions))
        msg.header.stamp = rospy.Time.now()
        self.joint_states_topic.publish(msg)

    def get_joint_names(self):
        return self.joint_positions.joint_names


if __name__ == "__main__":
    servo_interface = ServoInterface()
    while not rospy.is_shutdown():
        rospy.spin()


def test():
    from pprint import pprint
    sitr = [x[4] for x in pwm_map]
    pprint(joint_states_to_pwms(sitr))
    standr = [x[5] for x in pwm_map]
    pprint(joint_states_to_pwms(standr))
    zeror = [-math.pi] * 12
    pprint(joint_states_to_pwms(zeror))
    