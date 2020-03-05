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
        self.imu_sub = rospy.Subscriber("/ground_truth/cones", ConeArray, self.ground_truth_receiver)
        self.gps_sub = rospy.Subscriber(
            "/ground_truth/state",
            CarState,
            self.ground_truth_receiver
        )

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


        compared_poses = self.compare(true_pose_data, slam_pose_data)
        compared_maps = self.compare_cones(true_cones, slam_cones)
        self.out_msg.data = compared_poses + compared_maps

        self.out.publish(self.out_msg)

    def compare(self, vec1, vec2):
        return [math.sqrt((v1 - v2)**2) for v1, v2 in zip(vec1, vec2)]

    def compare_cones(self, cones1, cones2):


    def ground_truth_receiver(self, msg):
        """Receives ground truth from the simulation"""

        if str(msg._type) == "eufs_msgs/CarState":
            self.got_truth_pose = True
            self.ground_truth_pose = msg.pose
        elif str(msg._type) == "eufs_msgs/ConeArray":
            self.got_truth_map = True
            self.ground_truth_map = msg


if __name__ == "__main__":
    ekf_evaluator = SLAMEval()
    rospy.spin()
