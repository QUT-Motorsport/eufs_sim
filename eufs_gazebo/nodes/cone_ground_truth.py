#!/usr/bin/env python

""" Cone ground truth from Gazebo simulation in the frame of the car

Subscribed Topics:
    /ground_truth/state_raw (nav_msgs/Odometry)
        Simulated ground truth odometry for teh car

Published Topics:
    /ground_truth/cones (eufs_msgs/coneArray)
        Cone locations in the frame of the car
    /ground_truth/cones/viz (visualization_msgs/MarkerArray)
        Cone locations to be displayed in Rviz
    /ground_truth/midpoints (eufs_msgs/pointsArray)
        Track midpoints as an array of Points
    /ground_truth/midpoints/viz (visualization_msgs/Marker)
        Track midpoints for visualization in Rviz
    
Parameters:
    ~view_distance (float, default: 15)
        Only the cones that are within this distance are published
    ~track_path (string)
        Path to the track csv file


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
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray
from eufs_msgs.msg import coneArray, pointArray


class ConeGroundTruth:
    """The ConeGroundTruth class
    """

    def __init__(self):
        rospy.init_node("cone_ground_truth")
        self.blue_cones = None
        self.yellow_cones = None
        self.big_orange_cones = None
        self.orange_cones = None
        self.midpoints = None
        self.distance = rospy.get_param("~view_distance", default=15.)
        self.fov = rospy.get_param("~fov", default=1.91986)
        self.CONE_FRAME = "/base_footprint"  # frame of topics to be published

        # Load cone locations from CSV file
        track_path = rospy.get_param("~track_path")
        self.load_csv(track_path)

        # Main subscriber and callback
        self.subscriber = rospy.Subscriber("/ground_truth/state_raw", Odometry, self.odom_cb)

        # Publishers
        self.cone_pub = rospy.Publisher("/ground_truth/cones", coneArray, queue_size=1)
        self.cone_marker_pub = rospy.Publisher("/ground_truth/cones/viz", MarkerArray, queue_size=1)
        self.midpoints_pub = rospy.Publisher("/ground_truth/midpoints", pointArray, queue_size=1)
        self.midpoints_marker_pub = rospy.Publisher("/ground_truth/midpoints/viz", Marker, queue_size=1)

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
        """Calculates distances from one matrix vectors to 
            the other.

        Args:
            Xtrn (np.array): first matrix
            Xtrn (np.array): second matrix matrix

        Returns:
            Matrix of distances from the first matrix vectors to the second.
        """
        [M, L] = np.shape(Xtrn)
        [N, K] = np.shape(Xtst)
        if K == L:
            mul = np.dot(Xtst, Xtrn.T)
            XX = self.mul_by_transpose(Xtst, N)
            YY = self.mul_by_transpose(Xtrn, M)

            return np.add(np.subtract(matlib.repmat(XX, 1, M), 2 * mul), (np.matlib.repmat(YY, 1, N)).T)
        else:
            return np.array([[]])

    def process_cones(self, cones_list, yaw, trans):
        """Rotates the cones by the current yaw of the car
            and filters them according to the distance provided.

        Args:
            cones_list (np.array): the path to the SDF file to load
            yaw                  : current yaw of the car
            trans (np.array)     : translation of the car in the map frame

        Returns:
            List of filtered cones in the base_footprint frame.
        """
        transform = np.array([[trans[0], trans[1]]])
        cones_dists = self.dists(np.array(cones_list), transform)

        closest_cones = []
        for i in range(len(cones_dists[0])):
            if cones_dists[0][i] < np.power(self.distance, 2):
                closest_cones.append(cones_list[i])

        rotation_matrix = np.array([[np.cos(yaw), np.sin(yaw)],
                                    [-np.sin(yaw), np.cos(yaw)]])

        if len(closest_cones) > 0:
            translation = closest_cones - np.repeat(np.array(transform), np.shape(closest_cones)[0], axis=0)
            cones_rotated = np.dot(rotation_matrix, translation.T).T

            cones_in_view = []
            # get cones only in the field of view
            for each in cones_rotated:
                # convert to polar coordinates
                angle = np.arctan2(each[1], each[0])
                if np.abs(angle) < self.fov/2:
                    cones_in_view.append(each)
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
        """Callback function that translates the cone locations
            and publishes them.

        Args:
            msg (nav_msgs/Odometry): subscriber message

        Returns:
            Nothing
        """

        # If no subscribers to any topics, exit
        if self.cone_pub.get_num_connections() == 0 and \
                self.cone_marker_pub.get_num_connections() == 0 and \
                self.midpoints_pub.get_num_connections() == 0 and \
                self.midpoints_marker_pub.get_num_connections() == 0:
            rospy.logdebug("Nobody is listening to cone_ground_truth. Doing nothing")
            return

        # get translation and yaw
        pos = msg.pose.pose.position
        trans = np.array([pos.x, pos.y, pos.z])
        yaw = self.yaw_from_quat(msg.pose.pose.orientation)

        if self.cone_pub.get_num_connections() > 0 or \
                self.cone_marker_pub.get_num_connections() > 0:

            # Publish cone ground truth locations
            cone_msg = coneArray()
            cone_msg.header.frame_id = self.CONE_FRAME
            cone_msg.header.stamp = rospy.Time.now()

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
            except Exception as e:
                rospy.logwarn("Couldn't process orange cones.", e)

            try:
                big_orange_closest_cones = self.process_cones(self.big_orange_cones, yaw, trans)
                if len(big_orange_closest_cones) != 0:
                    for cone in np.reshape(big_orange_closest_cones, (-1, 2)):
                        cone_msg.big_orange_cones.append(Point(cone[0], cone[1], 0))
                        marker = self.get_cone_marker(pose=cone, rgb=(1, 0.271, 0), id=marker_id, big=True)
                        marker_id += 1
                        cone_markers.markers.append(marker)
            except Exception as e:
                rospy.logwarn("Couldn't process big orange cones.", e)

            # Publish topics
            self.cone_pub.publish(cone_msg)
            self.cone_marker_pub.publish(cone_markers)

        # Midpoints
        # Process only if there are midpoints and subscribers
        if self.midpoints is not None and \
                (self.midpoints_pub.get_num_connections() > 0 or
                 self.midpoints_marker_pub.get_num_connections() > 0):
            try:
                closest_midpoints = self.process_cones(self.midpoints, yaw, trans)
            except Exception as e:
                rospy.logwarn("Couldn't process midpoints.", e)
                return

            if len(closest_midpoints) != 0:
                midpoint_msg = pointArray()
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


if __name__ == "__main__":
    node = ConeGroundTruth()
    rospy.spin()
