#!/usr/bin/env python

"""
ekf_evaluator.py

Evaluates the ekf by comparing its outputs to the ground truth

It listens messages the following messages:

EKF Message:
`/odometry/filtered` (of type `nav_msgs/Odometry`)
`/accel/filtered` (of type `geometry_msgs/AccelWithCovarianceStamped`)

Ground truths:
`/imu` (of type `sensor_msgs/Imu`)
`/gps_velocity` (of type `geometry_msgs/Vector3Stamped`)
Note: CarState is a message we want to look into, not sure if ever
being published though

It publishes a message:

`/ekf/evaluation` (of type `eufs_msgs/EKFErr`)



"""

import numpy as np
import math
import rospy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3Stamped, AccelWithCovarianceStamped
from nav_msgs.msg import Odometry
from eufs_msgs.msg import EKFErr
import tf


class EKFEvaluator(object):

    def __init__(self, namespace='ekf_evaluator'):
        """
        Initialize the evaluator and subscribes to necessary channels

        Parameters:
                _output_interval: the hz at which to log to the terminal its results
        """

        rospy.init_node("ekf_evaluator", anonymous=True)


        # Don't publish anything if not received messages yet
        self.got_ekf_odom = False
        self.got_ekf_accel = False
        self.got_truth_imu = False
        self.got_truth_gps = False


        # The input message, stored
        self.ekf_odom = Odometry()
        self.ekf_accel = AccelWithCovarianceStamped()
        self.imu = Imu()
        self.gps = Vector3Stamped()
        

        # The output message
        self.out_msg = EKFErr()

        # Read in the parameters
        self.OUTPUT_INTERVAL = rospy.get_param("~output_interval", default=1)


        # Set up output publisher
        self.out = rospy.Publisher("/ekf/evaluation", EKFErr,  queue_size=1)


        # Set the timer to output info
        rospy.Timer(
            rospy.Duration(self.OUTPUT_INTERVAL),
            lambda x: self.evaluate()
        )


        # Subscribe to channels
        self.ekf_odom_sub = rospy.Subscriber("/odometry/filtered", Odometry, self.ekf_receiver)
        self.ekf_accel_sub = rospy.Subscriber(
            "/accel/filtered",
            AccelWithCovarianceStamped,
            self.ekf_receiver
        )
        self.imu_sub = rospy.Subscriber("/imu", Imu, self.ground_truth_receiver)
        self.gps_sub = rospy.Subscriber(
            "/gps_velocity",
            Vector3Stamped,
            self.ground_truth_receiver
        )


    def ekf_receiver(self, msg):
        """Receives the ekf outputs"""
        if str(msg._type) == "nav_msgs/Odometry":
            self.got_ekf_odom = True
            self.ekf_odom = msg
        elif str(msg._type) == "geometry_msgs/AccelWithCovarianceStamped":
            self.got_ekf_accel = True
            self.ekf_accel = msg

    def evaluate(self):
        """Does the actual evaluation of the ekf, called at a consistent rate"""
        if not (self.got_ekf_odom and self.got_ekf_accel):
            return

        if not (self.got_truth_gps and self.got_truth_imu):
            return
        
        # We care about the dynamic state - namely:
        # x/y velocity (from gps), x/y acceleration, yaw rate (from imu)

        gps_data = [self.gps.vector.x, self.gps.vector.y]
        ekf_gps_data = [self.ekf_odom.twist.twist.linear.x, self.ekf_odom.twist.twist.linear.y]
        
        quat = self.imu.orientation
        euler = tf.transformations.euler_from_quaternion((quat.x, quat.y, quat.z, quat.w))
        imu_data = [self.imu.linear_acceleration.x, self.imu.linear_acceleration.y, euler[2]]
        # Note: yaw is the 3rd component of rpy, so it's the "z" component in the Vector3
        ekf_imu_data = [
            self.ekf_accel.accel.accel.linear.x,
            self.ekf_accel.accel.accel.linear.y,
            self.ekf_odom.twist.twist.angular.z
        ]

        compared_data = self.compare(gps_data + imu_data, ekf_gps_data + ekf_imu_data)
        self.out_msg.gps_x_vel_err, self.out_msg.gps_y_vel_err, self.out_msg.imu_x_acc_err = compared_data[:3]
        self.out_msg.imu_y_acc_err, self.out_msg.imu_yaw_err = compared_data[3:]

        # Now we get x/y velo var, x/y accel var, yaw var
        self.ekf_x_vel_var = self.ekf_odom.twist.covariance[0]
        self.ekf_y_vel_var = self.ekf_odom.twist.covariance[7]
        self.ekf_x_acc_var = self.ekf_accel.accel.covariance[0]
        self.ekf_y_acc_var = self.ekf_accel.accel.covariance[7]
        self.ekf_yaw_var = self.ekf_odom.twist.covariance[35]

        self.out_msg.header.stamp = rospy.Time.now()
        self.out.publish(self.out_msg)

    def compare(self, vec1, vec2):
        return [math.sqrt((v1 - v2)**2) for v1, v2 in zip(vec1, vec2)]
        
    def ground_truth_receiver(self, msg):
        """Receives ground truth from the simulation"""
        
        if str(msg._type) == "sensor_msgs/Imu":
            self.got_truth_imu = True
            self.imu = msg
        elif str(msg._type) == "geometry_msgs/Vector3Stamped":
            self.got_truth_gps = True
            self.gps = msg

            
            



if __name__ == "__main__":
    ekf_evaluator = EKFEvaluator()
    rospy.spin()
