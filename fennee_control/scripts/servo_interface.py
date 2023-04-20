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
QUEUE_SIZE = 0
PCA_PWM_FREQUENCY = 50 # Hz for analog servos
# MAX_ANGLE = 270 # degrees
MAX_ANGLE = 180 # degrees

import rospy
# import numpy as np
import math
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from std_msgs.msg import UInt16MultiArray
from adafruit_servokit import ServoKit


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

# TODO: Publish joint angles to joint_state topic calculated from the pwm values
# So we will show limits and everything

# Sampled angles from controller        
#   expected -> actual                                
# sit pwm      stand pwm    center      sit angle    stand angle  
# 400 -> 431   400 -> 431   400 -> 400  0.0000000   0.000000     # front_left_shoulder                                   
# 351 -> 350   314 -> 309   207 -> 207  1.579998    1.132637     # front_left_leg                                    
# 113 -> 166   256 -> 225   400 -> 400  -2.576207   -1.92721     # front_left_foot                                   
# 297 -> 323   297 -> 323   297 -> 297  0.000000    -0.00000     # front_right_shoulder                                  
# 260 -> 258   297 -> 299   402 -> 402  1.579998    1.132637     # front_right_leg                                   
# 505 -> 447   362 -> 388   214 -> 214  -2.576207   -1.92721     # front_right_foot                                  
# 328 -> 315   328 -> 315   328 -> 328  0.0000000   0.000000     # rear_left_shoulder                                    
# 383 -> 381   347 -> 340   238 -> 238  1.579998    1.132637     # rear_left_leg                                 
# 113 -> 168   256 -> 227   402 -> 402  -2.576207   -1.92721     # rear_left_foot                                    
# 318 -> 309   318 -> 309   318 -> 318  0.0000000   -0.00000     # rear_right_shoulder                                   
# 262 -> 262   299 -> 303   406 -> 406  1.579998    1.132637     # rear_right_leg                                    
# 509 -> 456   366 -> 397   223 -> 223  -2.576207   -1.92721     # rear_right_foot            
#                        

# Example: front left foot
#
#           sit,          stand,       center 
#           ----          -----        ------
#   pwm |   113.0,        256.0,       400.0
# angle |   -2.576207,    -1.92721,    0.000000

# Map the control list to PWM channels
CHANNEL_MAP = [
    2,  # front_left_shoulder
    1,  # front_left_leg
    0,  # front_left_foot
    5,  # front_right_shoulder
    4,  # front_right_leg
    3,  # front_right_foot
    8,  # rear_left_shoulder
    7,  # rear_left_leg
    6,  # rear_left_foot
    11, # rear_right_shoulder
    10, # rear_right_leg
    9,  # rear_right_foot
]


# Manual calibration for now
PWM_MAP = [
    # center pwm, multiplier (reverse)
    
    (168,  1.0),    # 0 front_left_shoulder
    (35,   1.0),    # 1 front_left_leg
    (152,  1.0),    # 2 front_left_foot
    (107,   1.0),   # 3 front_right_shoulder
    (155,  -1.0),   # 4 front_right_leg
    (40,  -1.0),    # 5 front_right_foot
    (102,  -1.0),   # 6 rear_left_shoulder
    (53,   1.0),    # 7 rear_left_leg
    (155,  1.0),    # 8 rear_left_foot
    (93,  -1.0),    # 9 rear_right_shoulder
    (155, -1.0),    # 10 rear_right_leg
    (45,  -1.0),    # 11 rear_right_foot
]

# 0 230 # front_left_foot
# 1 45 # front_left_leg
# 2 250 # front_left_shoulder
# 3 60 # front_right_foot
# 4 240 # front_right_leg
# 5 160 # front_right_shoulder
# 6 230 # rear_left_foot
# 7 90 # rear_left_leg
# 8 150 # rear_left_shoulder
# 9 70 # rear_right_foot
# 10 220 # rear_right_leg
# 11 120 # rear_right_shoulder


def radians_to_pwm(angle, pwm_map_row):
    # TODO: Subtract from 360 if negative
    center_pwm, multiplier = pwm_map_row
    pwm = (math.degrees(angle) * multiplier) + center_pwm
    if pwm < 0:
        pwm += 360
    pwm = int(pwm % 360)
    pwm = min(MAX_ANGLE, pwm)
    return pwm

def joint_states_to_pwms(angles):
    """Converts joint angles to pwm values"""
    # TODO: Can probably do a batch calculation for all servos with numpy
    pwms = [radians_to_pwm(a, PWM_MAP[i]) for i, a in enumerate(angles)]
    remapped = [pwms[i] for i in CHANNEL_MAP]
    return remapped


class ServoInterface:
    def __init__(self):
        # PCA9685 I2C PWM controller
        self.pwm = ServoKit(channels=16)
        # for i in range(0, 12):
        #     self.pwm.servo[i].actuation_range = MAX_ANGLE
        self.joint_positions = None
        self.servo_states_topic = rospy.Publisher(
            SERVO_STATES_TOPIC, UInt16MultiArray, queue_size=QUEUE_SIZE
        )
        self.joint_states_topic = rospy.Publisher(
            JOINT_STATES_TOPIC, JointState, queue_size=QUEUE_SIZE
        )
        self.msg = JointState()
        rospy.init_node("servo_interface")
        rospy.Subscriber(
            JOINT_CONTROLLER_TOPIC,
            JointTrajectory,
            self.handle_joint_commands,
            queue_size=1,
        )
        rospy.Timer(rospy.Duration(1.0 / PUBLISH_FREQUENCY), self.publish_positions)

    def handle_joint_commands(self, data: JointTrajectory):
        # rospy.loginfo("handle_joint_commands")
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
        # rospy.loginfo("publish_servo_positions")
        # TODO: Transform joint positions to servo positions to pwm values
        angles = cmd.positions
        # print(self.get_joint_names())
        # print(dict(zip(self.get_joint_names(), angles)))
        angles = joint_states_to_pwms(angles)
        msg = UInt16MultiArray(data=tuple(int(abs(a)) for a in angles))
        self.servo_states_topic.publish(msg)
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
        return JOINT_NAMES


if __name__ == "__main__":
    rospy.loginfo("Starting servo_interface...")
    servo_interface = ServoInterface()
    rospy.loginfo("servo_interface started.")
    while not rospy.is_shutdown():
        rospy.spin()


def test():
    sit_angles = [
        0.0000000,
        1.579998,
        -2.576207,
        0.000000,
        1.579998,
        -2.576207,
        0.0000000,
        1.579998,
        -2.576207,
        0.0000000,
        1.579998,
        -2.576207,
    ]
    stand_angles = [
        0.000000,
        1.132637,
        -1.92721,
        -0.00000,
        1.132637,
        -1.92721,
        0.000000,
        1.132637,
        -1.92721,
        -0.00000,
        1.132637,
        -1.92721,
    ]
    center_angles = [
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
    ]
    print("")
    print("Center")
    for x in joint_states_to_pwms(center_angles):
        print(x)
    print("")
    print("Sit")
    for x in joint_states_to_pwms(sit_angles):
        print(x)
    print("")
    print("Stand")
    for x in joint_states_to_pwms(stand_angles):
        print(x)
    
    