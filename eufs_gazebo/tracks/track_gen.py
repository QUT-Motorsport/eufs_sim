#!/usr/bin/env python

"""track_gen.py

Script which generates CSV files from SDF Gazebo track models
CSV files consist of type of cones and their x and y positions
these CSV files are later intended to be used as input to a
ground truth cone publisher in simulation
"""

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import xml.etree.ElementTree as ET
from xml.etree.ElementTree import Element, SubElement, tostring
from scipy import interpolate
from scipy.spatial import cKDTree
from xml.dom import minidom
import argparse
import os
import rospkg
import rospy


class Track:
    def __init__(self):
        self.midpoints = None
        self.mid_track = None
        self.blue_cones = None
        self.blue_track = None
        self.yellow_cones = None
        self.yellow_track = None
        self.big_orange_cones = None
        self.orange_cones = None
        self.active_noise = None
        self.inactive_noise = None
        self.lap_counters = None

        # Car data in the format of ("car_start", x, y, yaw)
        # It can be left as ("car_start", 0, 0, 0), only relevant when fed into track_gen through
        # `eufs_launcher/ConversionTools`
        # as in that case it needs to preserve car data so that the conversion is fully bijective.
        self.car_start_data = ("car_start", 0.0, 0.0, 0.0)

    def load_csv(self, file_path):
        """Loads CSV file of cone location data and store it in the class.
        CSV file must have a 1 line header and then each row should have
        the format tag, x, y where tag is either "big", "blue", "orange" or "big_orange"

        Args:
            file_path (str): the path to the CSV file to load

        Returns:
            Nothing
        """

        if file_path.find(".csv") == -1:
            print("Please give me a .csv file. Exitting")
            return

        data = pd.read_csv(
            file_path,
            names=[
                "tag",
                "x",
                "y",
                "direction",
                "x_variance",
                "y_variance",
                "xy_covariance"
            ],
            skiprows=1
        )
        self.blue_cones = np.array(data[data.tag == "blue"][["x", "y"]])
        self.car = np.array(data[data.tag == "car_start"][["x", "y", "direction"]])
        self.yellow_cones = np.array(data[data.tag == "yellow"][["x", "y"]])
        self.big_orange_cones = np.array(data[data.tag == "big_orange"][["x", "y"]])
        self.orange_cones = np.array(data[data.tag == "orange"][["x", "y"]])

    def load_sdf(self, file_path):
        """Loads a Gazebo model .SDF file and identify cones in it
        based on their mesh tag. Store as elements of the class.

        Args:
            file_path (str): the path to the SDF file to load

        Returns:
            Nothing
        """

        if file_path.find(".sdf") == -1:
            print("Please give me a .sdf file. Exiting")
            return

        root = ET.parse(file_path).getroot()
        blue = []
        yellow = []
        orange = []
        big_orange = []
        active_noise = []
        inactive_noise = []
        lap_counters = []

        # iterate over all links of the model
        if len(root[0].findall("include")) != 0:
            for child in root[0].iter("include"):
                pose = child.find("pose").text.split(" ")[0:2]
                cov_node = child.find("covariance")
                if cov_node is None:
                    cov_info = [0.01, 0.01, 0.0]
                else:
                    covariance_x = float(cov_node.attrib["x"])
                    covariance_y = float(cov_node.attrib["y"])
                    covariance_xy = float(cov_node.attrib["xy"])
                    cov_info = [covariance_x, covariance_y, covariance_xy]
                mesh_str = "_".join(child.find("name").text.split("_")[:-1])
                # indentify cones by the name of their mesh
                if "blue_cone" == mesh_str:
                    blue.append(pose + cov_info)
                elif "yellow_cone" == mesh_str:
                    yellow.append(pose + cov_info)
                elif "big_cone" == mesh_str:
                    big_orange.append(pose + cov_info)
                elif "orange_cone" == mesh_str:
                    orange.append(pose + cov_info)
                elif "lap_counter" == mesh_str.split(";")[0]:
                    # Lap counter number stored in other part of `mesh_str`
                    lap_counters.append((pose + cov_info, mesh_str.split(";")[1]))
                elif "active_noise" == mesh_str:
                    active_noise.append(pose + cov_info)
                elif "inactive_noise" == mesh_str:
                    inactive_noise.append(pose + cov_info)
                else:
                    rospy.logerr("[track_gen.py] No such object: " + mesh_str)

        """
        # handle hidden links
        if len(root[0].findall("ghostlink")) != 0:
            for child in root[0].iter("ghostlink"):
                pose = child.find("pose").text.split(" ")[0:2]
                inactive_noise.append(pose)
        """

        """
        # handle includes
        # note, since some names mismatch (cone vs orange_cone), this section might
        # be out of date and require maintenance
        if len(root[0].findall("include")) != 0:
            for child in root[0].iter("include"):
                pose = child.find("pose").text.split(" ")[0:2]
                mesh_str = child.find("uri").text.split("/")[-1].split(".")[0]

                # indentify cones by the name of their mesh
                if "blue_cone" == mesh_str:
                    blue.append(pose)
                elif "yellow_cone" == mesh_str:
                    yellow.append(pose)
                elif "big_cone" == mesh_str:
                    big_orange.append(pose)
                elif "orange_cone" == mesh_str:
                    orange.append(pose)
        """

        # convert all lists to numpy as arrays for efficiency
        if len(blue) != 0:
            self.blue_cones = np.array(blue, dtype="float64")
        else:
            print("WARNING: No blue cones found")

        if len(yellow) != 0:
            self.yellow_cones = np.array(yellow, dtype="float64")
        else:
            print("WARNING: No yellow cones found")

        if len(big_orange) != 0:
            self.big_orange_cones = np.array(big_orange, dtype="float64")
        else:
            print("No big orange cones found")

        if len(orange) != 0:
            self.orange_cones = np.array(orange, dtype="float64")
        else:
            print("No orange cones found")

        if len(active_noise) != 0:
            self.active_noise = np.array(active_noise, dtype="float64")

        if len(inactive_noise) != 0:
            self.inactive_noise = np.array(inactive_noise, dtype="float64")

        if len(lap_counters) != 0:
            lc = [None, None]
            lc[0] = np.array([l[0] for l in lap_counters], dtype="float64")
            lc[1] = np.array([l[1] for l in lap_counters], dtype="int")
            self.lap_counters = lc

    def generate_tracks(self):
        """Generates blue, yellow and centerline tracks for the course
        and saves them within the class

        Args:
            None

        Returns:
            None
        """

        # Deal with blue cones
        if self.blue_cones.size is not None:
            self.blue_cones = self.order_points(self.blue_cones)
            self.blue_track = self.smoothLine(
                np.array([(x, y) for x, y, _, _, _ in self.blue_cones])
            )
        else:
            print("Warning: no blue cones")

        # Deal with yellow cones
        if self.yellow_cones.size is not None:
            self.yellow_cones = self.order_points(self.yellow_cones)
            self.yellow_track = self.smoothLine(
                np.array([(x, y) for x, y, _, _, _ in self.yellow_cones])
            )
        else:
            print("Warning: no yellow cones")

        # Deal with midline
        if self.midpoints.size is not None:
            self.midpoints = self.order_points(self.midpoints)
            self.midline = self.smoothLine(self.midpoints)
        else:
            print("Warning: no midpoints")

    def generate_midpoints(self, _plot=False):
        """Generates track midpoints based on blue and yellow cones

        Args:
            file_path (str): the path to the SDF file to load

        Returns:
            Nothing
        """
        assert self.blue_cones is not None or len(self.yellow_cones) != 0
        assert self.yellow_cones is not None or len(self.yellow_cones) != 0

        midpoints = []

        for blue_cone in self.blue_cones:
            closest_cone, closest_dist = self.find_closest(blue_cone, self.yellow_cones)
            midpoints.append([
                (closest_cone[0] + blue_cone[0]) / 2,
                (closest_cone[1] + blue_cone[1]) / 2]
            )

        if _plot:
            for blue_cone in self.blue_cones:
                closest_cone, closest_dist = self.find_closest(blue_cone, self.yellow_cones)
                self.plot_line(i, blue_cone)
                plt.plot(
                    (closest_cone[0] + blue_cone[0]) / 2,
                    (closest_cone[1] + blue_cone[1]) / 2,
                    'bo'
                )

            plt.show()

        self.midpoints = np.array(midpoints)

        return midpoints

    def plot_track(self):
        """Plot track in 2D with colours

        Args:
            None

        Returns:
            Nothing
        """
        fig, ax = plt.subplots()
        ax.set_xlim((-50, 50))
        ax.set_ylim((-50, 50))

        # Deal with blue cones
        if self.blue_cones.size is not None:
            plt.scatter(self.blue_cones[:, 0], self.blue_cones[:, 1], c="b", alpha="0.5")

            if self.blue_track is not None:
                plt.plot(self.blue_track[:, 0], self.blue_track[:, 1], "b-")
        else:
            print("Warning: no blue/blue cones")

        # Deal with yellow cones
        if self.yellow_cones.size is not None:
            plt.scatter(self.yellow_cones[:, 0], self.yellow_cones[:, 1], c="y", alpha="0.5")

            if self.yellow_track is not None:
                plt.plot(self.yellow_track[:, 0], self.yellow_track[:, 1], "y-")
        else:
            print("Warning: no yellow/yellow cones")

        if self.big_orange_cones is not None:
            plt.scatter(self.big_orange_cones[:, 0], self.big_orange_cones[:, 1], c="r", alpha="0.5")
        else:
            print("Warning: no big cones")

        if self.midpoints is not None:
            plt.scatter(self.midpoints[:, 0], self.midpoints[:, 1], c="g")

            if self.midline is not None:
                plt.plot(self.midline[:, 0], self.midline[:, 1], "g-")

        plt.axis('equal')
        plt.grid()
        plt.savefig("img/track.pdf")
        plt.show()

    def save_csv(self, filename, hdr=None):
        """Save track as a csv file along with tags: blue, yellow or big

        Args:
            file_path (str): the name and path of the file to save
            hdr (str): header of the CSV file

        Returns:
            Nothing
        """

        if filename.find(".csv") == -1:
            filename = filename + ".csv"

        df = pd.DataFrame(
            columns=["tag", "x", "y", "direction", "x_variance", "y_variance", "xy_covariance"]
        )

        # assuming there always are blue and yellow cones
        df["x"] = np.hstack((self.blue_cones[:, 0], self.yellow_cones[:, 0]))
        df["y"] = np.hstack((self.blue_cones[:, 1], self.yellow_cones[:, 1]))
        df["tag"].iloc[:] = "blue"
        df["tag"].iloc[-self.yellow_cones.shape[0]:] = "yellow"
        df["direction"] = 0
        df["x_variance"] = np.hstack((
            self.blue_cones[:, 2], self.yellow_cones[:, 2]
        ))
        df["y_variance"] = np.hstack((
            self.blue_cones[:, 3], self.yellow_cones[:, 3]
        ))
        df["xy_covariance"] = np.hstack((
            self.blue_cones[:, 4], self.yellow_cones[:, 4]
        ))

        if self.big_orange_cones is not None:
            empty = pd.DataFrame(
                np.nan,
                index=np.arange(self.big_orange_cones.shape[0]),
                columns=[
                    "tag", "x", "y", "direction",
                    "x_variance", "y_variance", "xy_covariance"
                ]
            )
            df = df.append(empty)
            df["x"] = np.hstack((df["x"].dropna().values, self.big_orange_cones[:, 0]))
            df["y"] = np.hstack((df["y"].dropna().values, self.big_orange_cones[:, 1]))
            df["tag"].iloc[-self.big_orange_cones.shape[0]:] = "big_orange"
            df["direction"] = np.hstack((
                df["direction"].dropna().values, [0 for _ in self.big_orange_cones[:, 2]]
            ))
            df["x_variance"] = np.hstack((
                df["x_variance"].dropna().values, self.big_orange_cones[:, 2]
            ))
            df["y_variance"] = np.hstack((
                df["y_variance"].dropna().values, self.big_orange_cones[:, 3]
            ))
            df["xy_covariance"] = np.hstack((
                df["xy_covariance"].dropna().values, self.big_orange_cones[:, 4]
            ))

        if self.orange_cones is not None:
            empty = pd.DataFrame(
                np.nan,
                index=np.arange(self.orange_cones.shape[0]),
                columns=[
                    "tag", "x", "y", "direction",
                    "x_variance", "y_variance", "xy_covariance"
                ]
            )
            df = df.append(empty)
            df["x"] = np.hstack((df["x"].dropna().values, self.orange_cones[:, 0]))
            df["y"] = np.hstack((df["y"].dropna().values, self.orange_cones[:, 1]))
            df["tag"].iloc[-self.orange_cones.shape[0]:] = "orange"
            df["direction"] = np.hstack((
                df["direction"].dropna().values, [0 for _ in self.orange_cones[:, 2]]
            ))
            df["x_variance"] = np.hstack((
                df["x_variance"].dropna().values, self.orange_cones[:, 2]
            ))
            df["y_variance"] = np.hstack((
                df["y_variance"].dropna().values, self.orange_cones[:, 3]
            ))
            df["xy_covariance"] = np.hstack((
                df["xy_covariance"].dropna().values, self.orange_cones[:, 4]
            ))

        if self.midpoints is not None:
            empty = pd.DataFrame(
                np.nan,
                index=np.arange(self.midpoints.shape[0]),
                columns=[
                    "tag", "x", "y", "direction",
                    "x_variance", "y_variance", "xy_covariance"
                ]
            )
            df = df.append(empty)
            df["x"] = np.hstack((df["x"].dropna().values, self.midpoints[:, 0]))
            df["y"] = np.hstack((df["y"].dropna().values, self.midpoints[:, 1]))
            df["tag"].iloc[-self.midpoints.shape[0]:] = "midpoint"
            df["direction"] = np.hstack((
                df["direction"].dropna().values, [0 for _ in self.midpoints[:, 1]]
            ))
            df["x_variance"] = np.hstack((
                df["x_variance"].dropna().values, [0.01 for _ in self.midpoints[:, 1]]
            ))
            df["y_variance"] = np.hstack((
                df["y_variance"].dropna().values, [0.01 for _ in self.midpoints[:, 1]]
            ))
            df["xy_covariance"] = np.hstack((
                df["xy_covariance"].dropna().values, [0 for _ in self.midpoints[:, 1]]
            ))

        if self.active_noise is not None:
            empty = pd.DataFrame(
                np.nan,
                index=np.arange(self.active_noise.shape[0]),
                columns=[
                    "tag", "x", "y", "direction",
                    "x_variance", "y_variance", "xy_covariance"
                ]
            )
            df = df.append(empty)
            df["x"] = np.hstack((df["x"].dropna().values, self.active_noise[:, 0]))
            df["y"] = np.hstack((df["y"].dropna().values, self.active_noise[:, 1]))
            df["tag"].iloc[-self.active_noise.shape[0]:] = "active_noise"
            df["direction"] = np.hstack((
                df["direction"].dropna().values, [0 for _ in self.active_noise[:, 2]]
            ))
            df["x_variance"] = np.hstack((
                df["x_variance"].dropna().values, self.active_noise[:, 2]
            ))
            df["y_variance"] = np.hstack((
                df["y_variance"].dropna().values, self.active_noise[:, 3]
            ))
            df["xy_covariance"] = np.hstack((
                df["xy_covariance"].dropna().values, self.active_noise[:, 4]
            ))

        if self.inactive_noise is not None:
            empty = pd.DataFrame(
                np.nan,
                index=np.arange(self.inactive_noise.shape[0]),
                columns=[
                    "tag", "x", "y", "direction",
                    "x_variance", "y_variance", "xy_covariance"
                ]
            )
            df = df.append(empty)
            df["x"] = np.hstack((df["x"].dropna().values, self.inactive_noise[:, 0]))
            df["y"] = np.hstack((df["y"].dropna().values, self.inactive_noise[:, 1]))
            df["tag"].iloc[-self.inactive_noise.shape[0]:] = "inactive_noise"
            df["direction"] = np.hstack((
                df["direction"].dropna().values, [0 for _ in self.inactive_noise[:, 2]]
            ))
            df["x_variance"] = np.hstack((
                df["x_variance"].dropna().values, self.inactive_noise[:, 2]
            ))
            df["y_variance"] = np.hstack((
                df["y_variance"].dropna().values, self.inactive_noise[:, 3]
            ))
            df["xy_covariance"] = np.hstack((
                df["xy_covariance"].dropna().values, self.inactive_noise[:, 4]
            ))

        if self.lap_counters is not None:
            position_values = self.lap_counters[0]
            lap_values = self.lap_counters[1]
            empty = pd.DataFrame(
                np.nan,
                index=np.arange(position_values.shape[0]),
                columns=[
                    "tag", "x", "y", "direction",
                    "x_variance", "y_variance", "xy_covariance"
                ]
            )
            df = df.append(empty)
            df["x"] = np.hstack((df["x"].dropna().values, position_values[:, 0]))
            df["y"] = np.hstack((df["y"].dropna().values, position_values[:, 1]))
            df["direction"] = np.hstack((df["direction"].dropna().values, lap_values))
            df["tag"].iloc[-position_values.shape[0]:] = "lap_counter"
            df["x_variance"] = np.hstack((
                df["x_variance"].dropna().values, position_values[:, 2]
            ))
            df["y_variance"] = np.hstack((
                df["y_variance"].dropna().values, position_values[:, 3]
            ))
            df["xy_covariance"] = np.hstack((
                df["xy_covariance"].dropna().values, position_values[:, 4]
            ))

        # Add car data (always ("car_start",0,0,0,0,0,0)
        # unless this file is called from ConversionTools))
        cardf = pd.DataFrame(
            [self.car_start_data],
            columns=[
                "tag",
                "x",
                "y",
                "direction",
                "x_variance",
                "y_variance",
                "xy_covariance"
            ]
        )
        df = df.append(cardf)

        df.to_csv(
            filename,
            index=False,
            columns=[
                "tag",
                "x",
                "y",
                "direction",
                "x_variance",
                "y_variance",
                "xy_covariance"
            ]
        )
        print("Succesfully saved to csv")

    def save_sdf(self, model_name):
        cone_meshes = {"yellow": "model://models/yellow_cone",
                       "blue": "model://models/blue_cone",
                       "big": "model://models/big_cone",
                       "orange": "model://models/orange_cone"}

        root = Element("sdf")
        root.set("version", "1.6")
        model = SubElement(root, "model")
        model.set("name", model_name)

        # deal with blue cones
        for i, each in enumerate(self.blue_cones):
            include = SubElement(model, "include")
            uri = SubElement(include, "uri")
            uri.text = cone_meshes["blue"]
            pose = SubElement(include, "pose")
            pose.text = str(each[0]) + " " + str(each[1]) + " 0 0 0 0"
            name = SubElement(include, "name")
            name.text = "blue_cone_" + str(i)

        # deal with yellow cones
        for i, each in enumerate(self.yellow_cones):
            include = SubElement(model, "include")
            uri = SubElement(include, "uri")
            uri.text = cone_meshes["yellow"]
            pose = SubElement(include, "pose")
            pose.text = str(each[0]) + " " + str(each[1]) + " 0 0 0 0"
            name = SubElement(include, "name")
            name.text = "yellow_cone_" + str(i)

        # deal with big cones
        for i, each in enumerate(self.big_orange_cones):
            include = SubElement(model, "include")
            uri = SubElement(include, "uri")
            uri.text = cone_meshes["big"]
            pose = SubElement(include, "pose")
            pose.text = str(each[0]) + " " + str(each[1]) + " 0 0 0 0"
            name = SubElement(include, "name")
            name.text = "big_cone_" + str(i)

        # deal with orange cones
        for i, each in enumerate(self.orange_cones):
            include = SubElement(model, "include")
            uri = SubElement(include, "uri")
            uri.text = cone_meshes["orange"]
            pose = SubElement(include, "pose")
            pose.text = str(each[0]) + " " + str(each[1]) + " 0 0 0 0"
            name = SubElement(include, "name")
            name.text = "big_cone_" + str(i)

        tree = ET.ElementTree(root)

        xmlstr = minidom.parseString(ET.tostring(root)).toprettyxml(indent="   ")

        with open("model.sdf", "w") as f:
            f.write(xmlstr)
        print("Created SDF file successfully!")

    def remove_point(self, p, ps):
        """Removes a point from a numpy array

        Args:
            p (2D-point): point to remove
            ps (2D numpy array): points to remove from

        Returns:
            Numpy array with point p removed
        """
        points = []
        for point in ps:
            if list(p) != list(point):
                points.append(point)
        return np.array(points)

    def find_closest(self, p, ps):
        """Find the closes point to p from points ps

        Args:
            p (2D-point): source point
            ps (2D numpy array): array of target points

        Returns:
            [x,y] of the closest point
            distance to the closest point
        """
        # drop the point if in points
        points = self.remove_point(p, ps)
        dist_2 = np.sum((points - p)**2, axis=1)
        return points[np.argmin(dist_2), :], np.min(dist_2)

    def plot_line(self, a, b, marker_style="ro-"):
        plt.plot([a[0], b[0]], [a[1], b[1]], marker_style)

    # Smooth the line ana generated nice and uniform waypoints
    def smoothLine(self, points, mul=5):
        """Centerline is 2xN (x y coordinates of midpoints) np array
            close_track (Connect first and last point)
            track limit track widtd
        """
        tck, u = interpolate.splprep(points.T, u=None, s=1.0, per=True)
        linargs = np.linspace(u.min(), u.max(), mul * points.shape[0])
        x_i, y_i = interpolate.splev(linargs, tck)
        output = np.array((x_i, y_i)).T
        return output

    def order_points(self, points):
        """Order points based on closest next point

        Args:
            points (2D numpy array): points to order

        Returns:
            Numpy array with point p removed
        """
        ordered = []
        current_point = points[0]
        points = points[1:]
        ordered.append(current_point)

        while points.shape[0] is not 0:
            closest_cone, closest_dist = self.find_closest(current_point, points)
            ordered.append(closest_cone)
            points = self.remove_point(closest_cone, points)
            current_point = closest_cone

        ordered.append(current_point)
        return np.array(ordered)

    def rotate_origin_only(self, xy, radians):
        """Only rotate a point around the origin (0, 0)."""
        x = xy[:, 0]
        y = xy[:, 1]
        xx = x * np.cos(radians) + y * np.sin(radians)
        yy = -x * np.sin(radians) + y * np.cos(radians)

        out = np.vstack((xx, yy))
        return out.T

    @staticmethod
    def runConverter(track_name,
                     midpoints=False,
                     car_start_data=("car_start", 0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
                     conversion_suffix="",
                     override_name=None):
        """
        Creates a csv from the sdf passed in (through track_name)

        track_name:        The name (not path) of the folder of the sdf for the track.
                           For example, if you desire a csv for the track with sdf stored
                           in YourTrack/model.sdf, then track_name should be "YourTrack".

        midpoints:         A boolean, when true the csv will also be populated with data
                           about the midpoints of cones in the track

        car_start_data:    Information about the start location of the car, as it
                           cannot be gleaned from the sdf.  It is used to preserve
                           information so that csvs can be converted back to .launches,
                           and is not necessary if you do not desire that functionality.

        conversion_suffix: This string will be appended to the end of track_name when
                           naming the csv file.  It is useful if there may already be
                           a csv file for the track and for whatever reason you do not
                           desire to overwrite it.
        """
        track_path = os.path.join(rospkg.RosPack().get_path('eufs_description'), "models",
                                  track_name, "model.sdf")

        # check if eufs_description exists
        try:
            assert os.path.isdir(rospkg.RosPack().get_path('eufs_description'))
        except:
            raise(AssertionError(("Can't find eufs_description directory."
                  "it is in the same location as eufs_gazebo!")))

        # check if requested track exists
        try:
            assert os.path.exists(track_path)
        except:
            raise(AssertionError(("Can't find track called {} make sure that it is"
                  "within eufs_description/models/".format(track_name))))

        track = Track()
        track.car_start_data = car_start_data
        track.load_sdf(track_path)
        if midpoints:
            track.generate_midpoints()
            track.generate_tracks()
        out_name = track_name+conversion_suffix if override_name is None else override_name
        track.save_csv(os.path.join(rospkg.RosPack().get_path('eufs_gazebo'), "tracks", out_name))

if __name__ == "__main__":
    # Just a heads up, you can run this with a gui by running the launcher:
    # `roslaunch eufs_launcher eufs_launcher.launch`
    # And using the "Conversion Tools" section.
    parser = argparse.ArgumentParser(description="Generates a CSV file of cone\
                                     locations based on the SDF Gazebo models")
    parser.add_argument('track_name', metavar='track_name', type=str,
                        help="the name of the Gazebo model track")
    parser.add_argument('--midpoints', metavar='midpoints', type=bool, default=False,
                        help="If true, midpoints will be generated and saved in the CSV")

    args = parser.parse_args()
    runConverter(args.track_name, args.midpoints)
