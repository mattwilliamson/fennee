#!/usr/bin/python3

"""This should publish to the JOINT_CONTROLLER_TOPIC so that the servo_interface will move the servos to the desired position.
Currently, however, we will just set positions to 0 directly so that we can assemble the robot without it moving around.
Ideally we would make this a GUI like the Champ UI so we can adjust each servo individually.
"""

from adafruit_servokit import ServoKit
import time

def calibrate_servos():
    print("Initilizing I2C...")
    pwm = ServoKit(channels=16)
    print("I2C initialized.")
    print("Setting all servos to middle...")

    for i in range(16):
        pwm.servo[i].angle = 90


if __name__ == "__main__":
    calibrate_servos()
    time.sleep(30)
