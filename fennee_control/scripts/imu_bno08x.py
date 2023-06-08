#!/usr/bin/env python3
# Driver: SPDX-FileCopyrightText: 2020 Bryan Siepert, written for Adafruit Industries

# TODO: Take in topic param

import rospy
from sensor_msgs.msg import MagneticField, Imu
from std_msgs.msg import Float64
from diagnostic_msgs.msg import DiagnosticStatus
import time
import sys
import board
import busio
from adafruit_bno08x import (
    BNO_REPORT_ACCELEROMETER,
    BNO_REPORT_GYROSCOPE,
    BNO_REPORT_MAGNETOMETER,
    BNO_REPORT_ROTATION_VECTOR,
    BNO_REPORT_STEP_COUNTER,
    REPORT_ACCURACY_STATUS,
    BNO_REPORT_GEOMAGNETIC_ROTATION_VECTOR,
    BNO_REPORT_GAME_ROTATION_VECTOR,
)
from adafruit_bno08x.i2c import BNO08X_I2C

PUBLISH_FREQUENCY = 50.0
DIAGNOSTIC_FREQUENCY = 1.0
ADDRESS = 0x4b
IMU_FRAME = "imu_link"
# IMU_FRAME = "imu_link_raw"
# DATA_TOPIC = 'imu/data'
DATA_TOPIC = 'imu/data'
MAG_TOPIC = 'imu/mag'
STATUS_TOPIC = 'imu/status'
STEPS_TOPIC = 'imu/steps'
ENABLE_STEPS = False

# TODO: Make orientation configurable

class IMU(object):
    def __init__(self):
        # Initialize ROS node
        self.raw_pub = rospy.Publisher(DATA_TOPIC, Imu, queue_size=20)
        self.mag_pub = rospy.Publisher(MAG_TOPIC, MagneticField, queue_size=20)
        self.status_pub = rospy.Publisher(STATUS_TOPIC, DiagnosticStatus, queue_size=2)
        self.steps_pub = rospy.Publisher(STEPS_TOPIC, Float64, queue_size=2)
        rospy.init_node('bno08x')
        rospy.loginfo(rospy.get_caller_id() + "  bno08x node launched.")

        # i2c = busio.I2C(board.SCL, board.SDA, frequency=800000)
        self.i2c = busio.I2C(board.SCL, board.SDA)
        self.bno = BNO08X_I2C(self.i2c, address=ADDRESS) # BNO080 (0x4b) BNO085 (0x4a)

        self.bno.enable_feature(BNO_REPORT_ACCELEROMETER)
        self.bno.enable_feature(BNO_REPORT_GYROSCOPE)
        self.bno.enable_feature(BNO_REPORT_MAGNETOMETER)
        # self.bno.enable_feature(BNO_REPORT_ROTATION_VECTOR)
        self.bno.enable_feature(BNO_REPORT_GEOMAGNETIC_ROTATION_VECTOR)
        # self.bno.enable_feature(BNO_REPORT_GAME_ROTATION_VECTOR)
        
        time.sleep(5) # ensure IMU is initialized

        rospy.Timer(rospy.Duration(1.0 / PUBLISH_FREQUENCY), self.publish_raw)
        rospy.Timer(rospy.Duration(1.0 / PUBLISH_FREQUENCY), self.publish_mag)
        rospy.Timer(rospy.Duration(1.0 / DIAGNOSTIC_FREQUENCY), self.publish_diagnostic)
        # rospy.Timer(rospy.Duration(10.0), self.check_calibration)

        # if ENABLE_STEPS:
        #     self.bno.enable_feature(BNO_REPORT_STEP_COUNTER)
        #     rospy.Timer(rospy.Duration(1.0 / DIAGNOSTIC_FREQUENCY), self.publish_steps)



    def publish_raw(self, event=None):
        raw_msg = Imu()

        # Swap axis to match ROS
        # accel_y, accel_z, accel_x = self.bno.acceleration
        accel_x, accel_y, accel_z = self.bno.acceleration
        raw_msg.linear_acceleration.x = accel_x
        raw_msg.linear_acceleration.y = accel_y
        raw_msg.linear_acceleration.z = accel_z

        # Swap axis to match ROS
        # gyro_y, gyro_z, gyro_x = self.bno.gyro
        gyro_x, gyro_y, gyro_z = self.bno.gyro
        raw_msg.angular_velocity.x = gyro_x
        raw_msg.angular_velocity.y = gyro_y
        raw_msg.angular_velocity.z = gyro_z
        
        # Swap axis to match ROS
        # quat_j, quat_k, quat_i, quat_real = self.bno.quaternion
        # quat_i, quat_j, quat_k, quat_real = self.bno.quaternion
        quat_i, quat_j, quat_k, quat_real = self.bno.geomagnetic_quaternion
        # quat_i, quat_j, quat_k, quat_real = self.bno.game_quaternion
        raw_msg.orientation.w = quat_real
        raw_msg.orientation.x = quat_i
        raw_msg.orientation.y = quat_j
        raw_msg.orientation.z = quat_k

        raw_msg.orientation_covariance[0] = 0.0025
        raw_msg.orientation_covariance[4] = 0.0025
        raw_msg.orientation_covariance[8] = 0.0025

        raw_msg.linear_acceleration_covariance[0] = 0.0001
        raw_msg.linear_acceleration_covariance[4] = 0.0001
        raw_msg.linear_acceleration_covariance[8] = 0.0001

        raw_msg.angular_velocity_covariance[0] = 0.000001
        raw_msg.angular_velocity_covariance[4] = 0.000001
        raw_msg.angular_velocity_covariance[8] = 0.000001


        raw_msg.linear_acceleration_covariance[0] = 0.0001
        raw_msg.linear_acceleration_covariance[4] = 0.0001
        raw_msg.linear_acceleration_covariance[8] = 0.0001

        raw_msg.header.frame_id = IMU_FRAME
        raw_msg.header.stamp = rospy.Time.now()
        self.raw_pub.publish(raw_msg)


    def publish_mag(self, event=None):
        mag_msg = MagneticField()
        # Swap axis to match ROS
        # mag_y, mag_z, mag_x = self.bno.magnetic
        mag_x, mag_y, mag_z = self.bno.magnetic
        mag_msg.header.stamp = rospy.Time.now()
        mag_msg.magnetic_field.x = mag_x
        mag_msg.magnetic_field.y = mag_y
        mag_msg.magnetic_field.z = mag_z
        mag_msg.magnetic_field_covariance[0] = 0.000001
        mag_msg.magnetic_field_covariance[4] = 0.000001
        mag_msg.magnetic_field_covariance[8] = 0.000001
        mag_msg.header.stamp = rospy.Time.now()
        mag_msg.header.frame_id = IMU_FRAME
        self.mag_pub.publish(mag_msg)
            
    def publish_diagnostic(self, event=None):
        status_msg = DiagnosticStatus()
        status_msg.level = 0
        status_msg.name = "bno08x IMU"
        status_msg.message = ""
        self.status_pub.publish(status_msg)

    def publish_steps(self, event=None):
        steps = self.bno.steps
        rospy.loginfo("imu_bno08x started: %d", steps)
        steps_msg = Float64()
        steps_msg.data = float(steps)
        self.steps_pub.publish(steps_msg)

    def check_calibration(self, event=None):
        try:
            rospy.logdebug("imu_bno08x compass calibration: %s", REPORT_ACCURACY_STATUS[self.bno.calibration_status])
            if self.bno.calibration_status >= 2:
                rospy.loginfo("imu_bno08x compass calibration is good. Saving.")
                self.bno.save_calibration_data()
        except:
            rospy.logerr("imu_bno08x error checking or saving calibration", exc_info=True)


if __name__ == "__main__":
    rospy.loginfo("Starting imu_bno08x...")
    imu = IMU()
    rospy.loginfo("imu_bno08x calibration: %s", REPORT_ACCURACY_STATUS[imu.bno.calibration_status])
    rospy.loginfo("imu_bno08x started.")
    while not rospy.is_shutdown():
        rospy.spin()
    rospy.loginfo(rospy.get_caller_id() + "  bno08x node finished")