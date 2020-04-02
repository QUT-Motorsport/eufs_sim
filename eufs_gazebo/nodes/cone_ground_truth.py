#!/usr/bin/env python

""" Node that simulates cone locations as they would have been received
by the team's cone detection stack. This reads a CSV file generated
from the track files and then publishes the appropriate cone detections
based on the location of the car at a certain frequency.

Subscribed Topics:
    /ground_truth/state (nav_msgs/Odometry)
        Simulated ground truth odometry for teh car

Published Topics:
    /ground_truth/cones (eufs_msgs/ConeArray)
        Cone locations in the frame of the car
    /ground_truth/cones/viz (visualization_msgs/MarkerArray)
        Cone locations to be displayed in Rviz
    /ground_truth/midpoints (eufs_msgs/pointsArray)
        Track midpoints as an array of Points
    /ground_truth/midpoints/viz (visualization_msgs/Marker)
        Track midpoints for visualization in Rviz
    /ground_truth/all_cones (of type `eufs_msgs/ConeArray`)
        Publishes the whole map

Parameters:
    ~view_distance (float, default: 15)
        Only the cones that are within this distance are published
    ~fov (float, default: 1.91986)
        Field of view in front of the car
    ~track_path (string)
        Path to the track csv file
    ~loop_rate (float, default: 25)
        Frequency at which the data is published


The MIT License

Copyright (c) 2018-2018 Edinburgh University Formula Student (EUFS)
http://eufs.eusa.ed.ac.uk

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
"""

# Python
import pandas as pd
import numpy as np
from numpy import matlib
import math

# ROS
import tf
import rospy
from eufs_msgs.msg import CarState
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray
from eufs_msgs.msg import ConeArray, PointArray


class ConeGroundTruth:
    """The ConeGroundTruth class
    """

    def __init__(self):
        rospy.init_node("cone_ground_truth")
        self.trans = None
        self.yaw = None
        self.blue_cones = None
        self.yellow_cones = None
        self.big_orange_cones = None
        self.orange_cones = None
        self.midpoints = None
        self.all_cones = ConeArray()
        self.distance = rospy.get_param("~view_distance", default=15.)
        self.fov = rospy.get_param("~fov", default=1.91986)  # 120 degrees
        self.CONE_FRAME = "/base_footprint"  # frame of topics to be published

        # starting position to be added as an offset to the pose
        self.x_init = rospy.get_param("~x_init", default=0.0)
        self.y_init = rospy.get_param("~y_init", default=0.0)
        self.yaw_init = rospy.get_param("~yaw_init", default=0.0)

        # Load cone locations from CSV file
        track_path = rospy.get_param("~track_path")
        self.load_csv(track_path)

        # Main subscriber and callback
        self.subscriber = rospy.Subscriber("/ground_truth/state", CarState, self.odom_cb)

        # Publishers
        self.cone_pub = rospy.Publisher("/ground_truth/cones", ConeArray, queue_size=1)
        self.cone_marker_pub = rospy.Publisher("/ground_truth/cones/viz", MarkerArray, queue_size=1)
        self.midpoints_pub = rospy.Publisher("/ground_truth/midpoints", PointArray, queue_size=1)
        self.midpoints_marker_pub = rospy.Publisher("/ground_truth/midpoints/viz", Marker, queue_size=1)
        self.all_cones_pub = rospy.Publisher("/ground_truth/all_cones", ConeArray, queue_size=1)

    def pub_ground_truth(self):
        """Function that translates the cone locations and publishes them.

        Returns:
            Nothing
        """

        # If translations and yaw are none, exit
        if (self.trans is None or self.yaw is None):
            rospy.logdebug("The translation and yaw have not been set. Doing nothing")
            return

        trans = self.trans
        yaw = self.yaw

        # If no subscribers to any topics, exit
        if (self.cone_pub.get_num_connections() == 0 and
                self.cone_marker_pub.get_num_connections() == 0 and
                self.midpoints_pub.get_num_connections() == 0 and
                self.midpoints_marker_pub.get_num_connections() == 0 and
                self.all_cones_pub.get_num_connections() == 0):
            rospy.logdebug("Nobody is listening to cone_ground_truth. Doing nothing")
            return

        # Publish the cone data
        if (self.cone_pub.get_num_connections() > 0 or
                self.cone_marker_pub.get_num_connections() > 0 or
                self.all_cones_pub.get_num_connections() > 0):

            # Publish cone ground truth locations
            cone_msg = ConeArray()
            cone_msg.header.frame_id = self.CONE_FRAME
            cone_msg.header.stamp = rospy.Time.now()
            self.all_cones = ConeArray()
            self.all_cones.header.frame_id = self.CONE_FRAME
            self.all_cones.header.stamp = rospy.Time.now()

            cone_markers = MarkerArray()
            marker_id = 0  # IDs are needed for the marker to work

            try:
                blue_closest_cones = self.process_cones(self.blue_cones, yaw, trans)
                if len(blue_closest_cones) != 0:
                    for cone in np.reshape(blue_closest_cones, (-1, 2)):
                        cone_msg.blue_cones.append(Point(cone[0], cone[1], 0))
                        marker = self.get_cone_marker(pose=cone, rgb=(0.2, 0.2, 1), id=marker_id)
                        marker_id += 1
                        cone_markers.markers.append(marker)
                blue_all_cones = self.process_cones(self.blue_cones, yaw, trans, let_all_in=True)
                if len(blue_all_cones) != 0:
                    for cone in np.reshape(blue_all_cones, (-1, 2)):
                        self.all_cones.blue_cones.append(Point(cone[0], cone[1], 0))
            except Exception as e:
                rospy.logerr("Couldn't process blue cones.", e)

            try:
                yellow_closest_cones = self.process_cones(self.yellow_cones, yaw, trans)
                if len(yellow_closest_cones) != 0:
                    for cone in np.reshape(yellow_closest_cones, (-1, 2)):
                        cone_msg.yellow_cones.append(Point(cone[0], cone[1], 0))
                        marker = self.get_cone_marker(pose=cone, rgb=(1, 1, 0), id=marker_id)
                        marker_id += 1
                        cone_markers.markers.append(marker)
                yellow_all_cones = self.process_cones(self.yellow_cones, yaw, trans, let_all_in=True)
                if len(yellow_all_cones) != 0:
                    for cone in np.reshape(yellow_all_cones, (-1, 2)):
                        self.all_cones.yellow_cones.append(Point(cone[0], cone[1], 0))
            except Exception as e:
                rospy.logwarn("Couldn't process yellow cones.", e)

            try:
                orange_closest_cones = self.process_cones(self.orange_cones, yaw, trans)
                if len(orange_closest_cones) != 0:
                    for cone in np.reshape(orange_closest_cones, (-1, 2)):
                        cone_msg.orange_cones.append(Point(cone[0], cone[1], 0))
                        marker = self.get_cone_marker(pose=cone, rgb=(1, 0.549, 0), id=marker_id)
                        marker_id += 1
                        cone_markers.markers.append(marker)
                orange_all_cones = self.process_cones(
                    self.orange_cones, yaw, trans, let_all_in=True
                )
                if len(blue_all_cones) != 0:
                    for cone in np.reshape(orange_all_cones, (-1, 2)):
                        self.all_cones.orange_cones.append(Point(cone[0], cone[1], 0))
            except Exception as e:
                rospy.logwarn("Couldn't process orange cones.", e)

            try:
                big_orange_closest_cones = self.process_cones(self.big_orange_cones, yaw, trans)
                if len(big_orange_closest_cones) != 0:
                    for cone in np.reshape(big_orange_closest_cones, (-1, 2)):
                        cone_msg.big_orange_cones.append(Point(cone[0], cone[1], 0))
                        marker = self.get_cone_marker(
                            pose=cone, rgb=(1, 0.271, 0), id=marker_id, big=True
                        )
                        marker_id += 1
                        cone_markers.markers.append(marker)
                big_orange_all_cones = self.process_cones(
                    self.big_orange_cones, yaw, trans, let_all_in=True
                )
                if len(big_orange_all_cones) != 0:
                    for cone in np.reshape(big_orange_all_cones, (-1, 2)):
                        self.all_cones.big_orange_cones.append(Point(cone[0], cone[1], 0))
            except Exception as e:
                rospy.logwarn("Couldn't process big orange cones.", e)

            # Publish topics
            self.cone_pub.publish(cone_msg)
            self.all_cones_pub.publish(self.all_cones)
            self.cone_marker_pub.publish(cone_markers)

        # Midpoints
        # Process only if there are midpoints and subscribers
        if (self.midpoints is not None and
                (self.midpoints_pub.get_num_connections() > 0 or
                 self.midpoints_marker_pub.get_num_connections() > 0)):
            try:
                closest_midpoints = self.process_cones(self.midpoints, yaw, trans)
            except Exception as e:
                rospy.logwarn("Couldn't process midpoints.", e)
                return

            if len(closest_midpoints) != 0:
                midpoint_msg = PointArray()
                midpoint_marker_msg = self.get_cone_marker(pose=[0, 0, 0], rgb=(0.2, 1, 0.2), id=0)
                midpoint_marker_msg.type = Marker.POINTS
                midpoint_marker_msg.scale.x = 0.5
                midpoint_marker_msg.scale.y = 0.5
                midpoint_marker_msg.scale.z = 0.5
                for cone in np.reshape(closest_midpoints, (-1, 2)):
                    p = Point(cone[0], cone[1], 0)
                    midpoint_msg.points.append(p)
                    midpoint_marker_msg.points.append(p)

                if self.midpoints_pub.get_num_connections() > 0:
                    self.midpoints_pub.publish(midpoint_msg)
                if self.midpoints_marker_pub.get_num_connections() > 0:
                    self.midpoints_marker_pub.publish(midpoint_marker_msg)

    def mul_by_transpose(self, X, size):
        """Helper function for self.dists().
        Multiplies each vector of a matrix by its transpose.

        Args:
            X (np.array): matrix whose vectors to multiply
            size (int): size of the matrix
        Returns:
            Matrix of vectors multiplied by their transposes.
        """
        result = []
        for i in range(size):
            result.append(np.dot(X[i], X[i].T))
        return np.expand_dims(np.array(result).T, axis=1)

    def dists(self, Xtrn, Xtst):
        """Calculates the distance of multiple points (rows)
        to another point

        Args:
            Xtrn (np.array): matrix of points (in rows)
            Xtst (np.array): anchor point

        Returns:
            1D matrix of distances of each row of Xtrn to the
            anchor point Xtst
        """
        [M, L] = np.shape(Xtrn)
        [N, K] = np.shape(Xtst)
        if K == L:  # check if dimensions are the same
            mul = np.dot(Xtst, Xtrn.T)
            XX = self.mul_by_transpose(Xtst, N)
            YY = self.mul_by_transpose(Xtrn, M)

            return np.add(np.subtract(matlib.repmat(XX, 1, M), 2 * mul), (np.matlib.repmat(YY, 1, N)).T)
        else:
            return np.array([[]])

    def process_cones(self, cones_list, yaw, trans, let_all_in = False):
        """Rotates the cones by the current yaw of the car
            and filters them according to the distance provided
            and the Field of View (FOV).

        Args:
            cones_list (np.array): 2D list of cones where each row is a cone
            yaw                  : current yaw of the car
            trans (np.array)     : translation of the car in the map frame
            let_all_in           : if True, disregards max distance and fov

        Returns:
            List of filtered cones in the base_footprint frame.
        """
        transform = np.array([[trans[0], trans[1]]])
        cones_dists = self.dists(np.array(cones_list), transform)

        closest_cones = []
        for i in range(len(cones_dists[0])):
            if cones_dists[0][i] < np.power(self.distance, 2) or let_all_in:
                closest_cones.append(cones_list[i])

        rotation_matrix = np.array([[np.cos(yaw), np.sin(yaw)],
                                    [-np.sin(yaw), np.cos(yaw)]])

        if len(closest_cones) > 0:
            translated_cones = closest_cones - np.repeat(
                np.array(transform),
                np.shape(closest_cones)[0],
                axis=0
            )
            cones_rotated = np.dot(rotation_matrix, translated_cones.T).T

            cones_in_view = []
            # get cones only in the field of view
            for cone in cones_rotated:
                # convert to polar coordinates
                angle = np.arctan2(cone[1], cone[0])
                if np.abs(angle) < self.fov/2 or let_all_in:
                    cones_in_view.append(cone)
            return cones_in_view
        else:
            return []

    def yaw_from_quat(self, quat):
        """Extracts the yaw from a quaternion

        Args:
            quat (nav_msgs/Quaternion): quaternion

        Returns:
            yaw
        """
        q0 = quat.w
        q1 = quat.x
        q2 = quat.y
        q3 = quat.z

        yaw = np.arctan2(2 * q1 * q2 + 2 * q0 * q3, q1 * q1 + q0 * q0 - q3 * q3 - q2 * q2)
        return yaw

    def odom_cb(self, msg):
        """Callback function that updates the stored yaw and trans of the car.

        Args:
            msg (nav_msgs/Odometry): subscriber message

        Returns:
            Nothing
        """

        # get translation and yaw
        pos = msg.pose.pose.position
        self.trans = np.array([pos.x + self.x_init, pos.y + self.y_init, pos.z])
        self.yaw = self.yaw_from_quat(msg.pose.pose.orientation) + self.yaw_init

    def get_cone_marker(self, pose, rgb, id, big=False):
        """Prepares a Marker for publishing

        Args:
            pose (list): xyz positions of cone
            rgb (tuple): red, green and blue colours for cone
            id (int): distinct id of the marker
            big (bool): flag for big cone mesh

        Returns:
            Marker ready for publishing
        """
        m = Marker()
        m.header.stamp = rospy.get_rostime()
        m.header.frame_id = self.CONE_FRAME
        m.id = id
        m.type = Marker.MESH_RESOURCE
        m.action = Marker.ADD
        m.pose.position.x = pose[0]
        m.pose.position.y = pose[1]
        m.scale.x = 1.5
        m.scale.y = 1.5
        m.scale.z = 1.5
        if big:
            m.mesh_resource = "package://eufs_description/meshes/cone_big.dae"
        else:
            m.mesh_resource = "package://eufs_description/meshes/cone.dae"
        m.color.r = rgb[0]
        m.color.g = rgb[1]
        m.color.b = rgb[2]
        m.color.a = 1.0
        m.lifetime = rospy.Duration().from_sec(0.2)  # Fix for stacking cones

        return m

    def load_csv(self, file_path):
        """Loads CSV file of cone location data and store it in the class.
        CSV file must have a 1 line header and then each row should have
        the format tag, x, y where tag is either "big", "left" or "right"

        Args:
            file_path (str): the path to the CSV file to load

        Returns:
            Nothing
        """

        if file_path.find(".csv") == -1:
            print("Please give me a .csv file. Exitting")
            return

        data = pd.read_csv(file_path)
        self.blue_cones = np.array(data[data.tag == "blue"][["x", "y"]])
        self.yellow_cones = np.array(data[data.tag == "yellow"][["x", "y"]])
        self.big_orange_cones = np.array(data[data.tag == "big_orange"][["x", "y"]])
        self.orange_cones = np.array(data[data.tag == "orange"][["x", "y"]])
        self.midpoints = np.array(data[data.tag == "midpoint"][["x", "y"]])
        self.all_cones.blue_cones = [Point(cone[0], cone[1], 0) for cone in self.blue_cones]
        self.all_cones.yellow_cones = [Point(cone[0], cone[1], 0) for cone in self.yellow_cones]
        self.all_cones.big_orange_cones = [Point(cone[0], cone[1], 0) for cone in self.big_orange_cones]
        self.all_cones.orange_cones = [Point(cone[0], cone[1], 0) for cone in self.orange_cones]


if __name__ == "__main__":
    node = ConeGroundTruth()

    frequency = rospy.get_param("~loop_rate", default=25)
    rate = rospy.Rate(frequency)

    while not rospy.is_shutdown():
        # Publish the most recent data
        node.pub_ground_truth()

        # Sleep - to run a the frequency
        rate.sleep()

