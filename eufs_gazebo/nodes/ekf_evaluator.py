#!/usr/bin/env python

"""
ekf_evaluator.py

Evaluates the ekf by comparing its outputs to the ground truth

It listens messages the following messages:

`/ekf/output/here` (of type `ekf/output/type/here`)

Ground truths:
`/wheel_odometry/odom` (of type `nav_msgs/Odometry`)
`/gps_velocity` (of type `geometry_msgs/Vector3Stamped`)
`/imu` (of type `sensor_msgs/Imu`)

It publishes a message:

`/ekf/evaluation` (of type `determine/type/soon`)



"""

import numpy as np
import math
import rospy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3Stamped
from nav_msgs.msg import Odometry


class EKFEvaluator(object):

    def __init__(self, namespace='ekf_evaluator'):
        """
        Initialize the evaluator and subscribes to necessary channels

        Parameters:
                _output_interval: the hz at which to log to the terminal its results
        """

        rospy.init_node("ekf_evaluator", anonymous=True)


        # Don't publish anything if not received messages yet
        self.got_ekf = False
        self.got_truth_odom = False
        self.got_truth_gps = False
        self.got_truth_imu = False


        # The input message, stored
        self.in_msgs = EKF_OUTPUT_MESSAGE_TYPE_HERE()

        # The output message
        self.out_msg = OUR_MESSAGE_TYPE_HERE()


        # Read in the parameters
        self.OUTPUT_INTERVAL = rospy.get_param("~output_interval", default=1)


        # Set up output publisher
        self.out = rospy.Publisher("/ekf/evaluation", EVALUATION_MESSAGE_TYPE_HERE,  queue_size=1)


        # Set the timer to output info
        rospy.Timer(
            rospy.Duration(self.OUTPUT_INTERVAL),
            lambda x: self.out.publish(self.out_msg)
        )


        # Subscribe to channels
        self.ekf_sub = rospy.Subscriber("/ekf/output/here", EKF_MESSAGE_TYPE_HERE, self.ekf_receiver)
        self.odom_sub = rospy.Subscriber("/wheel_odometry/odom", "nav_msgs/Odometry", self.ground_truth_receiver)
        self.gps_sub = rospy.Subscriber("/gps_velocity", "geometry_msgs/Vector3Stamped", self.ground_truth_receiver)
        self.imu_sub = rospy.Subscriber("/imu", "sensor_msgs/Imu", self.ground_truth_receiver)


    def ekf_receiver(self, msg):
        """Receives the ekf outputs"""
        self.got_ekf = True
        self.ekf = msg

"""
Note to self:
This node currently listens to robot_localisation inputs and outputs, but it actually is not listening to the
ground truth.  So we need a way to grab the ground truth as well
"""

        
    def ground_truth_receiver(self, msg):
        """Receives ground truth from the simulation"""
        
        if str(msg._type) == "nav_msgs/Odometry":
            self.got_truth_odom = True
            # Compare to self.ekf
        elif str(msg._type) == "geometry_msgs/Vector3Stamped":
            self.got_truth_gps = True
        elif str(msg._type) == "sensor_msgs/Imu":
            self.got_truth_imu = True



if __name__ == "__main__":
    ekf_evaluator = EKFEvaluator()
    rospy.spin()
