#!/usr/bin/env python

"""
fastslam_evaluator.py

Evaluates FastSLAM by comparing its outputs to the ground truth. For additional landmarks, which are not
in the ground truth map, we give a certain penalty.

It listens messages the following messages:

SLAM Message:
`fast_slam/pose` (of type `geometry_msgs/Pose`)
`/fast_slam/map` (of type `eufs_msgs/ConeArray`)

Ground truths:
`/ground_truth/cones` (of type `eufs_msgs/ConeArray`)
`/ground_truth/state ` (of type `eufs_msgs/CarState`)

It publishes a message:

`/slam/evaluation` (of type `std_msgs/Float64MultiArray`)

"""

import math
import rospy
import numpy as np
from skimage.draw import polygon
from std_msgs.msg import Float64MultiArray, MultiArrayLayout
from geometry_msgs.msg import Pose
from eufs_msgs.msg import ConeArray, CarState
import tf


class SLAMEval(object):

    def __init__(self, namespace='slam_evaluator'):
        """
        Initialize the evaluator and subscribes to necessary channels

        Parameters:
                _output_interval: the hz at which to log to the terminal its results
        """

        rospy.init_node("slam_evaluator", anonymous=True)

        # Don't publish anything if not received messages yet
        self.got_slam_map = False
        self.got_slam_pose = False
        self.got_truth_map = False
        self.got_truth_pose = False

        # The input message, stored
        self.slam_pose = Pose()
        self.slam_map = ConeArray()
        self.ground_truth_pose = Pose()
        self.ground_truth_map = ConeArray()

        # The output message
        self.out_msg = Float64MultiArray()
        self.out_msg.layout = MultiArrayLayout()
        self.out_msg.layout.data_offset = 0

        # Read in the parameters
        self.OUTPUT_INTERVAL = rospy.get_param("~output_interval", default=1)

        # Set up output publisher
        self.out = rospy.Publisher("/slam/evaluation", Float64MultiArray, queue_size=1)

        # Set the timer to output info
        rospy.Timer(
            rospy.Duration(self.OUTPUT_INTERVAL),
            lambda x: self.evaluate()
        )

        # Subscribe to channels
        self.slam_pose_sub = rospy.Subscriber("fast_slam/pose", Pose, self.slam_receiver)
        self.slam_map_sub = rospy.Subscriber(
            "/fast_slam/map",
            ConeArray,
            self.slam_receiver
        )
        self.map_sub = rospy.Subscriber("/ground_truth/cones", ConeArray, self.ground_truth_receiver)
        self.pose_sub = rospy.Subscriber(
            "/ground_truth/state",
            CarState,
            self.ground_truth_receiver
        )

        self.map_shape = (1000, 1000)

    def slam_receiver(self, msg):
        """Receives the ekf outputs"""
        if str(msg._type) == "geometry_msgs/Pose":
            self.got_slam_pose = True
            self.slam_pose = msg
        elif str(msg._type) == "eufs_msgs/ConeArray":
            self.got_slam_map = True
            self.slam_map = msg

    def evaluate(self):
        """Does the actual evaluation of SLAM, called at a consistent rate"""
        if not (self.got_slam_map and self.got_slam_pose):
            return

        if not (self.got_truth_pose and self.got_truth_map):
            return

        # We are comparing the ground truth pose with the estimated pose of FastSLAM and the ground truth cones with
        # the estimated cones from SLAM

        true_pose_data = [self.ground_truth_pose.position, self.ground_truth_pose.orientation]
        slam_pose_data = [self.slam_pose.position, self.slam_pose.position]

        true_cones = {"blue_cones": self.ground_truth_map.blue_cones,
                      "yellow_cones": self.ground_truth_map.yellow_cones,
                      "orange_cones": self.ground_truth_map.orange_cones,
                      "big_orange_cones": self.ground_truth_map.big_orange_cones,
                      "unknown_color_cones": self.ground_truth_map.unknown_color_cones}

        slam_cones = {"blue_cones": self.slam_map.blue_cones,
                      "yellow_cones": self.slam_map.yellow_cones,
                      "orange_cones": self.slam_map.orange_cones,
                      "big_orange_cones": self.slam_map.big_orange_cones,
                      "unknown_color_cones": self.slam_map.unknown_color_cones}

        pose_err = self.compare(true_pose_data, slam_pose_data)
        map_err = self.compare_cones(true_cones, slam_cones)
        self.out_msg.data = pose_err + map_err

        self.out.publish(self.out_msg)

    def compare(self, vec1, vec2):
        return [math.sqrt((v1 - v2) ** 2) for v1, v2 in zip(vec1, vec2)]

    """
    Compares the estimated map by doing the following:
        1. Creates an image of the ground-truth track and the estimated map.
        2. Count the number of pixels which are set in both maps. Count the number of pixels which
           are set in either the ground truth map or the estimated map.
        3. Divide the first result in 2 by the second result in 2. (also known as intersection over union)
    """
    def compare_cones(self, cones1, cones2):
        max_x, min_x, max_y, min_y = self.get_max_min(cones1["blue_cones"],
                                                      cones2["blue_cones"] + cones2["yellow_cones"])
        true_blue = self.rescale(cones1["blue_cones"], max_x, min_x, max_y, min_y)
        true_yellow = self.rescale(cones1["yellow_cones"], max_x, min_x, max_y, min_y)
        est_blue = self.rescale(cones2["blue_cones"], max_x, min_x, max_y, min_y)
        est_yellow = self.rescale(cones2["yellow_cones"], max_x, min_x, max_y, min_y)
        true_blue = self.order_cones(true_blue)
        true_yellow = self.order_cones(true_yellow)
        est_blue = self.order_cones(est_blue)
        est_yellow = self.order_cones(est_yellow)
        true_in_circle = self.draw_map(true_yellow)
        true_out_circle = self.draw_map(true_blue)
        est_in_circle = self.draw_map(est_yellow)
        est_out_circle = self.draw_map(est_blue)
        correct_map = true_in_circle ^ true_out_circle
        est_map = est_in_circle ^ est_out_circle
        intersection = correct_map & est_map
        union = est_map | correct_map
        return np.sum(intersection, dtype=np.float64) / np.sum(union, dtype=np.float64)

    def ground_truth_receiver(self, msg):
        """Receives ground truth from the simulation"""

        if str(msg._type) == "eufs_msgs/CarState":
            self.got_truth_pose = True
            self.ground_truth_pose = msg.pose
        elif str(msg._type) == "eufs_msgs/ConeArray":
            self.got_truth_map = True
            self.ground_truth_map = msg

    """
    Returns the maximum and minimum values of x and y.
    """
    def get_max_min(self, cones1, cones2):
        max_x, max_y = -math.inf, -math.inf
        min_x, min_y = math.inf, math.inf
        for cone in cones1 + cones2:
            if cone.x < min_x:
                min_x = cone.x
            elif cone.x > max_x:
                max_x = cone.x
            if cone.y < min_y:
                min_y = cone.y
            elif cone.y > max_y:
                max_y = cone.y
        return max_x, min_x, max_y, min_y

    """
    Rescales coordinates to lie in between our map image.
    """
    def rescale(self, cones, max_x, min_x, max_y, min_y):
        res = np.array(
            [[(point.y - min_y) / (max_y - min_y) * self.map_shape[0],
              (point.x - min_x) / (max_x - min_x) * self.map_shape[1]]
             for point in cones])
        return res

    """
    Orders cones according to the distance to the previous cone in our new ordered_cones array.
    """
    def order_cones(self, cones):
        ordered_cones = np.zeros(cones.shape)
        ordered_cones[0] = cones[0]
        included_idxs = {0}
        for i in range(1, len(cones)):
            closest_dist = math.inf
            closest_idx = i
            for j in range(len(cones)):
                if j not in included_idxs and np.linalg.norm(ordered_cones[i - 1] - cones[j]) < closest_dist:
                    closest_idx = j
                    closest_dist = np.linalg.norm(ordered_cones[i - 1] - cones[j])
            ordered_cones[i] = cones[closest_idx]
            included_idxs.add(closest_idx)
        return cones

    def draw_map(self, cones):
        new_map = np.zeros(self.map_shape + (3,), "uint8")
        rr, cc = polygon(cones[:, 0], cones[:, 1], new_map.shape) # rr, cc = row coordinates, column coordinates
        new_map[rr, cc] = 255
        return new_map


if __name__ == "__main__":
    ekf_evaluator = SLAMEval()
    rospy.spin()
