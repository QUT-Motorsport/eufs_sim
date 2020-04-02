#!/usr/bin/env python

"""
perception_sensors_simulator.py

Simulates the effect of the LiDAR and camera

It outputs messages the following messages:

`/perception_sim/cones` (of type `eufs_msgs::ConeArrayWithCovariance`)

Listens to:

`ground_truth/all_cones` (of type `eufs_msgs::ConeArray`)
`ground_truth/state` (of type `eufs_msgs::CarState`)

"""

import numpy as np
import math
import rospy
import tf
from eufs_msgs.msg import ConeWithCovariance, ConeArray, ConeArrayWithCovariance, CarState
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
        self.car_state = CarState()
        self.has_received_car_state = False
        self.car_position = None
        self.car_yaw = None

        # Initialize the parameters
        # Note - perception doesn't actually use a radius to keep points,
        # but rather compares absolute differences of components, with
        # a different threshold for x & y.  So we run both tests.
        self.lidar_min_dist = rospy.get_param("~lidar_min_dist", default=1)
        self.lidar_max_dist = rospy.get_param("~lidar_max_dist", default=100)
        self.lidar_fov_radians = rospy.get_param("~lidar_fov_radians", default=2*math.pi)
        self.camera_min_dist = rospy.get_param("~camera_min_dist", default=1)
        self.camera_max_dist = rospy.get_param("~camera_max_dist", default=15)
        self.camera_fov_radians = rospy.get_param(
            "~camera_fov_radians",
            default=110 * (math.pi / 180)
        )
        self.lidar_std_dev = rospy.get_param("~lidar_std_dev", default=0.03)
        self.lidar_min_x_dist = rospy.get_param("~lidar_min_x_dist", default=1)
        self.lidar_min_y_dist = rospy.get_param("~lidar_min_y_dist", default=1)
        self.lidar_max_x_dist = rospy.get_param("~lidar_max_x_dist", default=20)
        self.lidar_max_y_dist = rospy.get_param("~lidar_max_y_dist", default=10)
        self.camera_error_coefficient_a = rospy.get_param(
            "~camera_error_coefficient_a",
            default=0.0184
        )
        self.camera_error_coefficient_b = rospy.get_param(
            "~camera_error_coefficient_b",
            default=0.2106
        )

        # Derived parameters
        self.lidar_min_square_dist = self.lidar_min_dist**2
        self.lidar_max_square_dist = self.lidar_max_dist**2
        self.lidar_fov_radians_half = self.lidar_fov_radians/2
        self.camera_min_square_dist = self.camera_min_dist**2
        self.camera_max_square_dist = self.camera_max_dist**2
        self.camera_fov_radians_half = self.camera_fov_radians/2
        self.lidar_variance = self.lidar_std_dev**2

        # For testing purposes, set one of these to false!
        self.camera_active = True
        self.lidar_active = False

        # Set up publisher and subscriber
        self.cones_out = rospy.Publisher(
            "/perception_sim/cones",
            ConeArrayWithCovariance,
            queue_size=1
        )
        self.cones_in = rospy.Subscriber("/ground_truth/all_cones", ConeArray, self.receiver)
        self.car_sub = rospy.Subscriber("/ground_truth/state", CarState, self.car_update)

    def receiver(self, msg):
        """Receives ground truth and outputs converted info"""

        if not self.has_received_car_state:
            return

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
            self.add_error(cone, cone in cones_in_camera_view, cone in cones_in_lidar_view)
            for cone in sum(cone_lists_to_check, [])
            if cone not in cones_in_camera_view and cone in cones_in_lidar_view
        ]
        out_message.blue_cones = [
            self.add_error(cone, cone in cones_in_camera_view, cone in cones_in_lidar_view)
            for cone in out_message.blue_cones
            if cone in cones_in_camera_view
        ]
        out_message.yellow_cones = [
            self.add_error(cone, cone in cones_in_camera_view, cone in cones_in_lidar_view)
            for cone in out_message.yellow_cones
            if cone in cones_in_camera_view
        ]
        out_message.orange_cones = [
            self.add_error(cone, cone in cones_in_camera_view, cone in cones_in_lidar_view)
            for cone in out_message.orange_cones
            if cone in cones_in_camera_view
        ]
        out_message.big_orange_cones = [
            self.add_error(cone, cone in cones_in_camera_view, cone in cones_in_lidar_view)
            for cone in out_message.big_orange_cones
            if cone in cones_in_camera_view
        ]
        
            

        self.cones_out.publish(out_message)

    def car_update(self, msg):
        """Keeps this node's opinion on the car's state up to date."""
        self.has_received_car_state = True
        self.car_state = msg
        self.car_position = self.car_state.pose.pose.position
        car_orientation = self.car_state.pose.pose.orientation
        _, _, self.car_yaw = tf.transformations.euler_from_quaternion(
            [car_orientation.x, car_orientation.y, car_orientation.z, car_orientation.w]
        )

    def add_error(self, cone, in_camera, in_lidar):
        """Adds a noise profile to the cone"""

        # If can be seen by both, it's same as only being seen by lidar
        # (so camera adds no new information here, is just a backup in case
        # of no LiDAR)
        if in_lidar:
            cone.point.x = np.random.normal(cone.point.x, self.lidar_std_dev)
            cone.point.y = np.random.normal(cone.point.y, self.lidar_std_dev)
            cone.covariance = [self.lidar_variance, 0, 0, self.lidar_variance]

        elif in_camera:
            camera_depth_std = self.get_camera_depth_error(
                math.sqrt(self.square_dist(cone.point))
            )
            camera_orthogonal_std = camera_depth_std / 5
            unrotated_error_x = np.random.normal(0, camera_depth_std)
            unrotated_error_y = np.random.normal(0, camera_orthogonal_std)
            off_angle = -self.angular_dist(cone.point)
            sin_ = math.sin(off_angle)
            cos_ = math.cos(off_angle)
            rotated_error_x = unrotated_error_x * cos_ - unrotated_error_y * sin_
            rotated_error_y = unrotated_error_x * sin_ + unrotated_error_x * cos_
            cone.point.x += rotated_error_x
            cone.point.y += rotated_error_y

            # We calculate covariance matrix, check wiki for details
            # First order of business is recalculating the angle based off of our
            # error-introduced position
            off_angle = math.pi/2 + math.atan2(
                self.car_position.y - cone.point.y,
                self.car_position.x - cone.point.x
            )
            sin_ = math.sin(off_angle)
            cos_ = math.cos(off_angle)

            # hey, cool!  it just happens to be a rotation matrix...
            # Since it's orthonormal, its inverse is its transpose
            eigenvector_matrix = np.array([[cos_, -sin_], [sin_, cos_]])
            eigenvalue_matrix = np.array(
                [[camera_depth_std**2, 0], [0, camera_orthogonal_std**2]]
            )
            inverted_eigenvector_matrix = np.transpose(eigenvector_matrix)
            out_mat = np.matmul(
                np.matmul(eigenvector_matrix, eigenvalue_matrix),
                inverted_eigenvector_matrix
            )
            cone.covariance = [
                out_mat[0, 0], out_mat[1, 0],
                out_mat[0, 1], out_mat[1, 1]
            ]

        return cone

    def lidar_can_see(self, point):
        """Checks if point is in the FOV of the LiDAR"""
        if not self.lidar_active:
            return False
        distance = self.square_dist(point)
        angle_distance = self.angular_dist(point)
        satisfies_distance_requirement = (
            self.lidar_min_square_dist < distance and
            distance < self.lidar_max_square_dist and
            abs(point.x - self.car_position.x) < self.lidar_max_x_dist and
            self.lidar_min_x_dist < abs(point.x - self.car_position.x) and
            abs(point.y - self.car_position.y) < self.lidar_max_y_dist and
            self.lidar_min_y_dist < abs(point.y - self.car_position.y)
        )
        satisfies_fov_requirement = abs(angle_distance) < self.lidar_fov_radians_half
        return satisfies_distance_requirement and satisfies_fov_requirement

    def camera_can_see(self, point):
        """Checks if point is in the FOV of the Camera"""
        if not self.camera_active:
            return False
        distance = self.square_dist(point)
        angle_distance = self.angular_dist(point)
        satisfies_distance_requirement = (
            self.camera_min_square_dist < distance and distance < self.camera_max_square_dist
        )
        satisfies_fov_requirement = abs(angle_distance) < self.camera_fov_radians_half
        return satisfies_distance_requirement and satisfies_fov_requirement

    def square_dist(self, point):
        """Calculates the square distance from point to car"""
        return (point.x - self.car_position.x)**2 + (point.y - self.car_position.y)**2

    def angular_dist(self, point):
        """
        Calculates angular distance between vectors A&B with origin being the car position, where;
        A is the vector to the input point
        B is the vector along which the car faces
        """

        # Get unit A
        A_ = np.array([point.x - self.car_position.x, point.y - self.car_position.y])
        A_norm = np.linalg.norm(A_)
        if A_norm == 0:
            return 0
        A = A_/A_norm

        # Get unit B
        B = np.array([math.cos(self.car_yaw), math.sin(self.car_yaw)])
        
        # Calculate angle: arccos(A @ B)
        return math.acos(np.clip(np.dot(A, B), -1, 1))
    

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

    def get_camera_depth_error(self, depth):
        """Estimates the std deviation given a distance from the car"""
        return self.camera_error_coefficient_a * math.exp(self.camera_error_coefficient_a * depth)


if __name__ == "__main__":
    perception_sensors_simulator = PerceptionSensorsSimulator()
    rospy.spin()
