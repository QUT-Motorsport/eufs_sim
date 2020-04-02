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
        self.car_state = Odometry()
        self.has_received_car_state = False

        # Initialize the parameters
        # Note - limit both max dists to 30, multibly lidar fov by 30/360 to see
        # an example of FastSLAM struggling!
        self.lidar_min_dist = 1
        self.lidar_max_dist = 100
        self.lidar_fov_radians = 2*math.pi
        self.camera_min_dist = 1
        self.camera_max_dist = 100
        self.camera_fov_radians = 120 * (math.pi / 180)

        # Derived parameters
        self.lidar_min_square_dist = self.lidar_min_dist**2
        self.lidar_max_square_dist = self.lidar_max_dist**2
        self.lidar_fov_radians_half = self.lidar_fov_radians/2
        self.camera_min_square_dist = self.camera_min_dist**2
        self.camera_max_square_dist = self.camera_max_dist**2
        self.camera_fov_radians_half = self.camera_fov_radians/2

        # Set up publisher and subscriber
        self.cones_out = rospy.Publisher(
            "/perception_sim/cones",
            ConeArrayWithCovariance,
            queue_size=1
        )
        self.cones_in = rospy.Subscriber("/ground_truth/all_cones", ConeArray, self.receiver)
        self.car_sub = rospy.Subscriber("/ground_truth/state_raw", Odometry, self.car_update)

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

    def car_update(self, msg):
        """Keeps this node's opinion on the car's state up to date."""
        self.has_received_car_state = True
        self.car_state = msg

    def lidar_can_see(self, point):
        """Checks if point is in the FOV of the LiDAR"""
        distance = self.square_dist(point)
        angle_distance = self.angular_dist(point)
        satisfies_distance_requirement = (
            self.lidar_min_square_dist < distance and distance < self.lidar_max_square_dist
        )
        satisfies_fov_requirement = abs(angle_distance) < self.lidar_fov_radians_half
        return satisfies_distance_requirement and satisfies_fov_requirement

    def camera_can_see(self, point):
        """Checks if point is in the FOV of the Camera"""
        distance = self.square_dist(point)
        angle_distance = self.angular_dist(point)
        satisfies_distance_requirement = (
            self.camera_min_square_dist < distance and distance < self.camera_max_square_dist
        )
        satisfies_fov_requirement = abs(angle_distance) < self.camera_fov_radians_half
        return satisfies_distance_requirement and satisfies_fov_requirement

    def square_dist(self, point):
        """Calculates the square distance from point to car"""
        car_loc = self.car_state.pose.pose.position
        return (point.x - car_loc.x)**2 + (point.y - car_loc.y)**2

    def angular_dist(self, point):
        """
        Calculates angular distance between vectors A&B with origin being the car position, where;
        A is the vector to the input point
        B is the vector along which the car faces
        """
        car_loc = self.car_state.pose.pose.position
        car_orientation = self.car_state.pose.pose.orientation
        _, _, car_angle = tf.transformations.euler_from_quaternion(
            [car_orientation.x, car_orientation.y, car_orientation.z, car_orientation.w]
        )

        # Get unit A
        A_ = np.array([point.x - car_loc.x, point.y - car_loc.y])
        A_norm = np.linalg.norm(A_)
        if A_norm == 0:
            return True
        A = A_/A_norm

        # Get unit B
        B = np.array([math.cos(car_angle), math.sin(car_angle)])
        
        # Calculate angle: arccos(A @ B)
        return math.acos(A[0]*B[0] + A[1]*B[1])
    

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
