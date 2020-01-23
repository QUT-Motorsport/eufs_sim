#!/usr/bin/env python

"""
ekf_evaluator.py

Evaluates the ekf by comparing its outputs to the ground truth

It listens messages the following messages:

EKF Message:
`/ekf/output/here` (of type `ekf/output/type/here`)

Ground truths:
`/imu` (of type `sensor_msgs/Imu`)
`/gps_velocity` (of type `geometry_msgs/Vector3Stamped`)

It publishes a message:

`/ekf/evaluation` (of type `std_msgs/Float64MultiArray`)



"""

import numpy as np
import math
import rospy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3Stamped
from std_msgs.msg import Float64MultiArray
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
        self.got_ekf = False
        self.got_truth_imu = False
        self.got_truth_gps = False


        # The input message, stored
        self.ekf = EKF_OUTPUT_MESSAGE_TYPE_HERE()
        self.imu = Imu()
        self.gps = Vector3Stamped()
        

        # The output message
        self.out_msg = Float64MultiArray()
        self.out_msg.layout = MultiArrayLayout()
        self.out_msg.layout.data_offset = 0
        self.out_msg.layout.dim[0].label = "height"
        self.out_msg.layout.dim[0].stride = 5
        self.out_msg.layout.dim[0].size = 5


        # Read in the parameters
        self.OUTPUT_INTERVAL = rospy.get_param("~output_interval", default=1)


        # Set up output publisher
        self.out = rospy.Publisher("/ekf/evaluation", EVALUATION_MESSAGE_TYPE_HERE,  queue_size=1)


        # Set the timer to output info
        rospy.Timer(
            rospy.Duration(self.OUTPUT_INTERVAL),
            lambda x: self.evaluate()
        )


        # Subscribe to channels
        self.ekf_sub = rospy.Subscriber("/ekf/output/here", EKF_MESSAGE_TYPE_HERE, self.ekf_receiver)
        self.imu_sub = rospy.Subscriber("/imu", "sensor_msgs/Imu", self.ground_truth_receiver)
        self.gps_sub = rospy.Subscriber("/gps_velocity", "geometry_msgs/Vector3Stamped", self.ground_truth_receiver)


    def ekf_receiver(self, msg):
        """Receives the ekf outputs"""
        self.got_ekf = True
        self.ekf = msg

"""
Note to self:
This node currently listens to robot_localisation inputs and outputs, but it actually is not listening to the
ground truth.  So we need a way to grab the ground truth as well
"""

    def evaluate(self):
        """Does the actual evaluation of the ekf, called at a consistent rate"""
        if not self.got_ekf:
            return
        
        # We care about the dynamic state - namely:
        # x/y velocity (from gps), x/y acceleration, yaw rate (from imu)
        gpsdata = [-1000,-1000]
        if self.got_truth_gps:
            gpsdata = [self.gps.vector.x, self.gps.vector.y]
            # TODO compare with ekf info
        
        imudata = [-1000,-1000,-1000]
        if self.got_truth_imu:
            quat = self.imu.orientation
            euler = tf.transformations.euler_from_quaternion(quat)
            imuddata = [self.imu.linear_acceleration.x, self.imu.linear_acceleration.y, euler[2]]
            # TODO compare with ekf info
            
        self.out_msg.data = gpsdata + imudata
            
        self.out.publish(self.out_msg)

        
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
