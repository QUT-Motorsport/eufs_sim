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
The first 3 elements are the position erros in x, y and z dimension. The following four are the orientation errors. Those
are euclidean distance errors. The last one is the map error, which can be interpreted as a percentage of how well our
estimated map represents the actual map.

"""

import math
import rospy
import numpy as np
from skimage.draw import polygon
from geometry_msgs.msg import Pose
from eufs_msgs.msg import ConeArray, CarState, SlamErr

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
        self.out_msg = SlamErr()

        # Read in the parameters
        self.OUTPUT_INTERVAL = rospy.get_param("~output_interval", default=1)

        # Set up output publisher
        self.out = rospy.Publisher("/slam/evaluation", SlamErr, queue_size=1)

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
        self.map_sub = rospy.Subscriber("/ground_truth/all_cones", ConeArray, self.ground_truth_receiver)
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

    def ground_truth_receiver(self, msg):
        """Receives ground truth from the simulation"""

        if str(msg._type) == "eufs_msgs/CarState":
            self.got_truth_pose = True
            self.ground_truth_pose = msg.pose.pose
        elif str(msg._type) == "eufs_msgs/ConeArray":
            self.got_truth_map = True
            self.ground_truth_map = msg

    def evaluate(self):
        """Does the actual evaluation of SLAM, called at a consistent rate"""
        if not (self.got_slam_map and self.got_slam_pose):
            return

        if not (self.got_truth_pose and self.got_truth_map):
            return

        # We are comparing the ground truth pose with the estimated pose of FastSLAM and the ground truth cones with
        # the estimated cones from SLAM

        true_pose_data = [self.ground_truth_pose.position.x,
                          self.ground_truth_pose.position.y,
                          self.ground_truth_pose.position.z,
                          self.ground_truth_pose.orientation.x,
                          self.ground_truth_pose.orientation.y,
                          self.ground_truth_pose.orientation.z,
                          self.ground_truth_pose.orientation.w]
        slam_pose_data = [self.slam_pose.position.x,
                          self.slam_pose.position.y,
                          self.slam_pose.position.z,
                          self.slam_pose.orientation.x,
                          self.slam_pose.orientation.y,
                          self.slam_pose.orientation.z,
                          self.slam_pose.orientation.w]

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
        self.out_msg.x_err, self.out_msg.y_err, self.out_msg.z_err = pose_err[:3]
        self.out_msg.x_orient_err, self.out_msg.y_orient_err, self.out_msg.z_orient_err, self.out_msg.w_orient_err = \
            pose_err[3:]
        self.out_msg.map_err = map_err
        self.out_msg.header.stamp = rospy.Time.now()
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
    def compare_cones(self, gt_cones, est_cones):
        max_x, min_x, max_y, min_y = self.get_max_min(gt_cones["blue_cones"]) #first max_min of the GT map
        true_blue = self.rescale(gt_cones["blue_cones"], max_x, min_x, max_y, min_y)
        true_yellow = self.rescale(gt_cones["yellow_cones"], max_x, min_x, max_y, min_y)
        max_x, min_x, max_y, min_y = self.get_max_min(est_cones["blue_cones"] + est_cones["yellow_cones"])
        est_blue = self.rescale(est_cones["blue_cones"], max_x, min_x, max_y, min_y)
        est_yellow = self.rescale(est_cones["yellow_cones"], max_x, min_x, max_y, min_y)

        true_blue = self.order_points(true_blue)
        true_yellow = self.order_points(true_yellow)
        est_blue = self.order_points(est_blue)
        est_yellow = self.order_points(est_yellow)
        true_in_circle = self.draw_map(true_yellow)
        true_out_circle = self.draw_map(true_blue)
        est_in_circle = self.draw_map(est_yellow)
        est_out_circle = self.draw_map(est_blue)
        correct_map = true_in_circle ^ true_out_circle
        est_map = est_in_circle ^ est_out_circle
        intersection = correct_map & est_map
        union = est_map | correct_map
        return np.sum(intersection, dtype=np.float64) / np.sum(union, dtype=np.float64)

    """
    Returns the maximum and minimum values of x and y for cones.
    """
    def get_max_min(self, cones):
        max_x, max_y = -np.inf, -np.inf
        min_x, min_y = np.inf, np.inf
        for points in cones:
            if points.x < min_x:
                min_x = points.x
            elif points.x > max_x:
                max_x = points.x
            if points.y < min_y:
                min_y = points.y
            elif points.y > max_y:
                max_y = points.y
        return max_x, min_x, max_y, min_y

    """
    Rescales cone coordinates to lie in between our map image.
    """
    def rescale(self, cones, max_x, min_x, max_y, min_y):
        res = np.array(
            [[(point.y - min_y) / (max_y - min_y) * self.map_shape[0],
              (point.x - min_x) / (max_x - min_x) * self.map_shape[1]]
             for point in cones])
        return res

    """
    Orders points according to the distance to the previous point in our new ordered_points array.
    """
    def order_points(self, points):
        ordered_points = np.zeros(points.shape)
        ordered_points[0] = points[0]
        included_idxs = {0}
        for i in range(1, len(points)):
            closest_dist = np.inf
            closest_idx = i
            for j in range(len(points)):
                if j not in included_idxs and np.linalg.norm(ordered_points[i - 1] - points[j]) < closest_dist:
                    closest_idx = j
                    closest_dist = np.linalg.norm(ordered_points[i - 1] - points[j])
            ordered_points[i] = points[closest_idx]
            included_idxs.add(closest_idx)
        return ordered_points

    """
    Takes an array of cones and draws the polygon in an image by setting pixels in the polygon to 255.
    """
    def draw_map(self, cones):
        new_map = np.zeros(self.map_shape + (3,), "uint8")
        rr, cc = polygon(cones[:, 0], cones[:, 1], new_map.shape) # rr, cc = row coordinates, column coordinates
        new_map[rr, cc] = 255
        return new_map


if __name__ == "__main__":
    ekf_evaluator = SLAMEval()
    rospy.spin()
