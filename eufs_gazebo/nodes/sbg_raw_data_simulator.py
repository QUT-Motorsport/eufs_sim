#!/usr/bin/env python

"""
sbg_raw_data_simulator.py

Simulates the raw data (no ekf) from the sbg

It outputs messages the following messages:

`/imu/nav_sat_fix` (of type `sensor_msgs/NavSatFix`)
`/imu/data`    (of type `sensor_msgs/Imu`)

"""

import numpy as np
import math
import rospy
import tf
from sensor_msgs.msg import NavSatFix, Imu
from tf.transformations import *


class SbgRawDataSimulator(object):

    def __init__(self, namespace='sbg_raw_data_simulator'):
        """
        Initialize the publishers and subscribes to necessary channels

        `self.gps` mimics the gps data of the sbg
        `self.imu` mimics the imu data of the sbg

        Raw imu simulator parameters in: eufs_description/sensors/imu.urdf.xacro
        Raw gps simulator parameters in: eufs_description/sensors/gps.urdf.xacro

        Parameters:
                _imu_hz: the hz at which to broadcast the imu data, note that this
                        has implications for accuracy as the noise models are affected by
                        sample rate!
                _gps_hz: the hz at which to broadcast the gps data, has no accuracy implications

        Highly recommended to read the wiki
        https://www.wiki.ed.ac.uk/display/EUF/SBG+Sensor+Simulation
        to understand why the noise is produced the way it is.
        """

        rospy.init_node("sbg_raw_data_simulator", anonymous=True)

        # Create persistent messages
        self.imu_msg = Imu()
        self.gps_msg = NavSatFix()

        # Create bias error accumulator, only relevant for the imu
        self.imu_angular_velocity_bias = [0, 0, 0]

        # Don't publish anything if not received messages yet
        self.got_imu = False
        self.got_gps = False

        # Latitude & longitude to meters
        # Approximations about Edinburgh 55.950191, -3.187566
        # Values represent distance you have to travel to equal 1 meter.
        self.latitude_approx = 0.000018
        self.longitude_approx = 0.000016

        # Publish at fixed rates
        self.IMU_PUBLISH_RATE_HZ = rospy.get_param("~imu_hz", default=200)
        self.GPS_PUBLISH_RATE_HZ = rospy.get_param("~gps_hz", default=5)
        self.gps = rospy.Publisher("/imu/nav_sat_fix", NavSatFix, queue_size=1)
        self.imu = rospy.Publisher("/imu/data",    Imu,  queue_size=1)
        rospy.Timer(
            rospy.Duration(1.0 / self.IMU_PUBLISH_RATE_HZ),
            lambda x: self.imu.publish(self.imu_msg) if self.got_imu and self.got_gps else None
        )
        rospy.Timer(
            rospy.Duration(1.0 / self.GPS_PUBLISH_RATE_HZ),
            lambda x: self.gps.publish(self.gps_msg) if self.got_imu and self.got_gps else None
        )

        # Set calculated standard distribution information
        # check https://www.wiki.ed.ac.uk/display/EUF/SBG+Sensor+Simulation
        # for method and reasons.
        # Note that we had trouble calculating bias constant so for now we're using
        # the white noise constant for bias as well.
        self.imu_angular_velocity_white_noise_std = math.sqrt(0.00000625*self.IMU_PUBLISH_RATE_HZ)
        self.imu_angular_velocity_bias_std = math.sqrt(
            1.476 * math.pow(10, -13) / self.IMU_PUBLISH_RATE_HZ
        )
        self.imu_linear_acceleration_white_noise_std = math.sqrt(0.000000312*self.IMU_PUBLISH_RATE_HZ)
        self.gps_position_white_noise_std = math.sqrt(39.4)

        # Subscribe to channels
        self.imu_sub = rospy.Subscriber("/imu", Imu, self.imu_receiver)
        self.gps_sub = rospy.Subscriber("/gps", NavSatFix, self.gps_receiver)

    def imu_receiver(self, msg):
        """Receives gazebo's simulated imu and converts it to sbg-style SbgImuData format"""
        self.got_imu = True

        # First we add noise to angular velocity
        ang_vel = [msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z]
        for i in (0, 1, 2):
            # Add white noise errors
            ang_vel[i] += np.random.normal(0, self.imu_angular_velocity_white_noise_std)

            # Add accumulation errors
            self.imu_angular_velocity_bias[i] += np.random.normal(
                0,
                self.imu_angular_velocity_bias_std
            )
            ang_vel[i] += self.imu_angular_velocity_bias[i]
        msg.angular_velocity.x = ang_vel[0]
        msg.angular_velocity.y = ang_vel[1]
        msg.angular_velocity.z = ang_vel[2]

        # Add variances to diagonals of covariance matrix
        diagonal = (
            self.imu_angular_velocity_white_noise_std * self.imu_angular_velocity_white_noise_std
        )
        msg.angular_velocity_covariance = (
            diagonal, 0, 0,
            0, diagonal, 0,
            0, 0, diagonal,
        )

        # Now we add noise to linear acceleration
        lin_acc = [msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z]
        for i in (0, 1, 2):
            # Add white noise errors
            lin_acc[i] += np.random.normal(0, self.imu_linear_acceleration_white_noise_std)

        msg.linear_acceleration.x = lin_acc[0]
        msg.linear_acceleration.y = lin_acc[1]
        msg.linear_acceleration.z = lin_acc[2] - 9.81

        # Add variances to diagonals of covariance matrix
        diagonal = (
            self.imu_linear_acceleration_white_noise_std * self.imu_linear_acceleration_white_noise_std
        )
        msg.linear_acceleration_covariance = (
            diagonal, 0, 0,
            0, diagonal, 0,
            0, 0, diagonal,
        )

        # Replace orientation values with error value (9999) as
        # we want to calculate them ourselves.
        msg.orientation.x = 9999
        msg.orientation.y = 9999
        msg.orientation.z = 9999
        msg.orientation.w = 9999

        self.imu_msg = msg

    def gps_receiver(self, msg):
        """Receives gazebo's simulated gps and saves it (already in right format)"""
        self.got_gps = True

        # We only need to add white noise
        msg.latitude += (
            np.random.normal(0, self.gps_position_white_noise_std) * self.latitude_approx
        )
        msg.longitude += (
            np.random.normal(0, self.gps_position_white_noise_std) * self.longitude_approx
        )

        # Add variances to diagonals of covariance matrix
        diagonal = (
            self.gps_position_white_noise_std * self.gps_position_white_noise_std
        )
        msg.position_covariance = (
            diagonal, 0, 0,
            0, diagonal, 0,
            0, 0, diagonal,
        )

        self.gps_msg = msg


if __name__ == "__main__":
    sbg_raw_data_simulator = SbgRawDataSimulator()
    rospy.spin()
