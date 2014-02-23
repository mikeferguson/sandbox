#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Imu

class ImuPublisher:

    def __init__(self, ip="192.168.0.41", port=6707):        
        self._pub = rospy.Publisher("imu_raw", Imu)

    # Note, axis are adjusted for pose in robomagellan bot!!
    def publish(self, etherbotix):
        msg = Imu()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "imu_link"

        # TODO: compute actual heading
        #   currently this is a broken message, but this allows recording of
        #   magnetometer values for further process
        msg.orientation.x = -etherbotix.mag_y
        msg.orientation.y = -etherbotix.mag_x
        msg.orientation.z = -etherbotix.mag_z

        # No known orientation
        msg.orientation_covariance[0] = -1.0

        # L3GD20 Gyro
        # 70mdps/digit
        msg.angular_velocity.x = -etherbotix.gyro_y * 0.001221111
        msg.angular_velocity.y = -etherbotix.gyro_x * 0.001221111
        msg.angular_velocity.z = -etherbotix.gyro_z * 0.001221111
        # 0.2% Non Linearityx, 2000dps
        msg.angular_velocity_covariance[0] = 0.004868938
        msg.angular_velocity_covariance[4] = 0.004868938
        msg.angular_velocity_covariance[8] = 0.004868938

        # LSM303DLHC Accelerometer
        # 2g/full scale
        msg.linear_acceleration.x = -etherbotix.acc_y * 0.000598773
        msg.linear_acceleration.y = -etherbotix.acc_x * 0.000598773
        msg.linear_acceleration.z = -etherbotix.acc_z * 0.000598773
        # 60mg noise
        msg.linear_acceleration_covariance[0] = 0.34644996
        msg.linear_acceleration_covariance[4] = 0.34644996
        msg.linear_acceleration_covariance[8] = 0.34644996

        self._pub.publish(msg)
