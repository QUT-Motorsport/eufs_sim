#!/usr/bin/env python

"""
perception_sensors_simulator.py

Simulates the effect of the LiDAR and camera

It outputs messages the following messages:

`/perception_sim/cones` (of type `eufs_msgs::ConeArrayWithCovariance`)

Listens to:

`ground_truth/all_cones` (of type `eufs_msgs::ConeArray`)
`ground_truth/state_raw` (of type `nav_msgs::Odometry`)

"""

import numpy as np
import math
import rospy
import tf
from eufs_msgs.msg import ConeWithCovariance, ConeArray, ConeArrayWithCovariance
from nav_msgs.msg import Odometry
from tf.transformations import *


class PerceptionSensorsSimulator(object):

    def __init__(self, namespace='perception_sensors_simulator'):
        """
        Initialize the publishers and subscribes to necessary channels

        Highly recommended to read the wiki
        https://www.wiki.ed.ac.uk/display/EUF/Perception+Sensor+Simulation
        to understand why the noise is produced the way it is.
        """

        rospy.init_node("perception_sensors_simulator", anonymous=True)

        # Initialize persistent variables
        self.position = Odometry()
        self.has_received_position = False

        # Set up publisher and subscriber
        self.cones_out = rospy.Publisher(
            "/perception_sim/cones",
            ConeArrayWithCovariance,
            queue_size=1
        )
        self.cones_in = rospy.Subscriber("/ground_truth/cones", ConeArray, self.receiver)
        self.position = rospy.Subscriber("/ground_truth/state_raw", Odometry, self.position_update)

    def receiver(self, msg):
        """Receives ground truth and outputs converted info"""

        # First we convert our input ConeArray into a ConeArrayWithCovariance
        out_message = self.convert_to_have_covariance(msg)

        # Now we loop through all the cones
        cones_in_lidar_view = []
        cones_in_camera_view = []
        cone_lists_to_check = [
            out_message.blue_cones,
            out_message.yellow_cones,
            out_message.orange_cones,
            out_message.big_orange_cones,
            out_message.unknown_color_cones
        ]
        for cone_list in cone_lists_to_check:
            for cone in cone_list:
               if self.lidar_can_see(cone.point):
                   cones_in_lidar_view.append(cone)
               if self.camera_can_see(cone.point):
                   cones_in_camera_view.append(cone)
        
        # Now we remove or uncolor out-of-view cones
        # Uncolored if only visible by lidar, removed if visible in neither
        out_message.unknown_color_cones = [
            cone for cone in sum(cone_lists_to_check, [])
            if cone not in cones_in_camera_view and cone in cones_in_lidar_view
        ]
        out_message.blue_cones = [
            cone for cone in out_message.blue_cones
            if cone in cones_in_camera_view
        ]
        out_message.yellow_cones = [
            cone for cone in out_message.yellow_cones
            if cone in cones_in_camera_view
        ]
        out_message.orange_cones = [
            cone for cone in out_message.orange_cones
            if cone in cones_in_camera_view
        ]
        out_message.big_orange_cones = [
            cone for cone in out_message.big_orange_cones
            if cone in cones_in_camera_view
        ]
        
            

        self.cones_out.publish(out_message)

    def position_update(self, msg):
        """Keeps this node's opinion on the car's position up to date."""
        self.has_received_position = True
        self.position = msg

    def lidar_can_see(self, point):
        """Checks if point is in the FOV of the LiDAR"""
        return True

    def camera_can_see(self, point):
        """Checks if point is in the FOV of the Camera"""
        return False

    def convert_to_have_covariance(self, msg):
        """
        Given a ConeArray, returns ConeArrayWithCovariance
        with all covariances [-1, -1, -1, -1]
        """
        out_message = ConeArrayWithCovariance()
        out_message.header = msg.header
        
        def point_to_cone(p):
            """Takes in point p, returns ConeWithCovariance with dummy covariance"""
            to_return = ConeWithCovariance()
            to_return.point = p
            to_return.covariance = [-1, -1, -1, -1]
            return to_return

        out_message.blue_cones = [point_to_cone(point) for point in msg.blue_cones]
        out_message.yellow_cones = [point_to_cone(point) for point in msg.yellow_cones]
        out_message.orange_cones = [point_to_cone(point) for point in msg.orange_cones]
        out_message.big_orange_cones = [point_to_cone(point) for point in msg.big_orange_cones]
        out_message.unknown_color_cones = [
            point_to_cone(point) for point in msg.unknown_color_cones
        ]

        return out_message


if __name__ == "__main__":
    perception_sensors_simulator = PerceptionSensorsSimulator()
    rospy.spin()
