import os
import numpy as np
import pandas as pd
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node


# Class which generates CSV files from SDF Gazebo track models
# CSV files consist of type of cones and their x and y positions
class Track(Node):
    def __init__(self):
        self.blue_cones = None
        self.yellow_cones = None
        self.big_orange_cones = None
        self.orange_cones = None

        # Car data in the format of ("car_start", x, y, yaw)
        # It can be left as ("car_start", 0, 0, 0), only relevant when fed
        # into track_gen through `eufs_tracks/ConversionTools`
        # as in that case it needs to preserve car data so that the
        # conversion is fully bijective.
        self.car_start_data = ("car_start", 0.0, 0.0, 0.0)

    def load_sdf(self, file_path):
        """
        Loads a Gazebo model .SDF file and identify cones in it
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
                else:
                    print("[track_gen.py] No such object: " + mesh_str)

        # convert all lists to numpy as arrays for efficiency
        if len(blue) != 0:
            self.blue_cones = np.array(blue, dtype="float64")
        else:
            self.blue_cones = None
            print("No blue cones found!")

        if len(yellow) != 0:
            self.yellow_cones = np.array(yellow, dtype="float64")
        else:
            self.yellow_cones = None
            print("No yellow cones found!")

        if len(big_orange) != 0:
            self.big_orange_cones = np.array(big_orange, dtype="float64")
        else:
            self.big_orange_cones = None
            print("No big orange cones found!")

        if len(orange) != 0:
            self.orange_cones = np.array(orange, dtype="float64")
        else:
            self.orange_cones = None
            print("No orange cones found!")

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
        df["x_variance"] = np.hstack((self.blue_cones[:, 2], self.yellow_cones[:, 2]))
        df["y_variance"] = np.hstack((self.blue_cones[:, 3], self.yellow_cones[:, 3]))
        df["xy_covariance"] = np.hstack((self.blue_cones[:, 4], self.yellow_cones[:, 4]))

        if self.big_orange_cones is not None:
            empty = pd.DataFrame(
                np.nan,
                index=np.arange(self.big_orange_cones.shape[0]),
                columns=["tag", "x", "y", "direction", "x_variance", "y_variance", "xy_covariance"]
            )
            df = df.append(empty)
            df["x"] = np.hstack((df["x"].dropna().values, self.big_orange_cones[:, 0]))
            df["y"] = np.hstack((df["y"].dropna().values, self.big_orange_cones[:, 1]))
            df["tag"].iloc[-self.big_orange_cones.shape[0]:] = "big_orange"
            df["direction"] = np.hstack((
                df["direction"].dropna().values,
                [0 for _ in self.big_orange_cones[:, 2]]))
            df["x_variance"] = np.hstack(
                (df["x_variance"].dropna().values, self.big_orange_cones[:, 2]))
            df["y_variance"] = np.hstack((
                df["y_variance"].dropna().values, self.big_orange_cones[:, 3]))
            df["xy_covariance"] = np.hstack((
                df["xy_covariance"].dropna().values,
                self.big_orange_cones[:, 4]))

        if self.orange_cones is not None:
            empty = pd.DataFrame(
                np.nan,
                index=np.arange(self.orange_cones.shape[0]),
                columns=["tag", "x", "y", "direction", "x_variance", "y_variance", "xy_covariance"]
            )
            df = df.append(empty)
            df["x"] = np.hstack((df["x"].dropna().values, self.orange_cones[:, 0]))
            df["y"] = np.hstack((df["y"].dropna().values, self.orange_cones[:, 1]))
            df["tag"].iloc[-self.orange_cones.shape[0]:] = "orange"
            df["direction"] = np.hstack((
                df["direction"].dropna().values,
                [0 for _ in self.orange_cones[:, 2]]
            ))
            df["x_variance"] = np.hstack((
                df["x_variance"].dropna().values, self.orange_cones[:, 2]))
            df["y_variance"] = np.hstack((
                df["y_variance"].dropna().values, self.orange_cones[:, 3]))
            df["xy_covariance"] = np.hstack((
                df["xy_covariance"].dropna().values, self.orange_cones[:, 4]))

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

    @staticmethod
    def sdf_to_csv(track_name,
                   car_start_data=("car_start", 0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
                   override_name=None):
        """
        Creates a csv from the sdf passed in (through track_name)

        track_name:        The name (not path) of the folder of the sdf for the track.
                           For example, if you desire a csv for the track with sdf stored
                           in YourTrack/model.sdf, then track_name should be "YourTrack".

        car_start_data:    Information about the start location of the car, as it
                           cannot be gleaned from the sdf.  It is used to preserve
                           information so that csvs can be converted back to .launches,
                           and is not necessary if you do not desire that functionality.
        """
        TRACKS_SHARE = get_package_share_directory('eufs_tracks')
        track_path = os.path.join(TRACKS_SHARE, "models", track_name, "model.sdf")

        # Check for existence
        assert os.path.isdir(TRACKS_SHARE), "Can't find eufs_tracks"
        assert os.path.exists(track_path), f"Can't find {track_name} track in eufs_tracks/models/"

        track = Track()
        track.car_start_data = car_start_data
        track.load_sdf(track_path)
        out_name = track_name if override_name is None else override_name
        track.save_csv(os.path.join(TRACKS_SHARE, "csv", out_name))


# Here are all the track formats we care about:
# - `launch` (well, we actually want the model data, not the .launch, but we'll treat it as wanting
#             the .launch since the end user shouldn't have to care about the distinction)
# - `csv`
class Converter(Node):
    def __init__(self):
        pass

    # Keep track of how many we've placed so that we can give each a unique name.
    link_num = -1

    #########################################################
    #                Main Conversion Method                 #
    #########################################################

    @staticmethod
    def convert(cfrom, cto, which_file, params={}):
        """
        Will convert which_file of filetype cfrom to filetype cto with filename which_file

        cfrom:      Type to convert from (launch, csv) [should be a string]
        cto:        Type to convert to   (launch, csv) [should be a string]

        which_file: The file to be converted - should be a full filepath.

        params:     Additional parameters that may be necessary. These will depend on conversion
                    type, so check the docstrings of the specific desired conversion function for
                    full information.
        """

        if cfrom == "launch" and cto == "csv":
            return Converter.launch_to_csv(which_file, params)
        elif cfrom == "csv" and cto == "launch":
            return Converter.csv_to_launch(which_file, params)
        return None

    @staticmethod
    def launch_to_csv(which_file, params={}):
        """
        Converts a .launch to a .csv

        which_file: The name of the launch file to convert example: rand.launch
        """

        filename = which_file.split("/")[-1].split(".")[0]
        with open(which_file) as car_data_reader:
            car_data = car_data_reader.read()
        car_x = car_data.split("<arg name=\"x\" default=\"")[1].split("\"")[0]
        car_y = car_data.split("<arg name=\"y\" default=\"")[1].split("\"")[0]
        car_yaw = car_data.split("<arg name=\"yaw\" default=\"")[1].split("\"")[0]
        Track.sdf_to_csv(
            filename,
            car_start_data=("car_start", car_x, car_y, car_yaw, 0.0, 0.0, 0.0),
            override_name=params.get("override_name", None)
        )

    @staticmethod
    def csv_to_launch(which_file, params={}):
        """
        Converts a .csv to a .launch

        which_file: The name of the csv file to convert example: rand.csv
        """

        # Save eufs_tracks directory
        TRACKS_SHARE = get_package_share_directory("eufs_tracks")

        # Use override name if provided
        GENERATED_FILENAME = params.get("override_name", which_file.split("/")[-1].split(".")[0])

        # First, we read in the csv data into a dataframe
        df = pd.read_csv(which_file)
        blue_cones = df[df['tag'] == "blue"]
        yellow_cones = df[df['tag'] == "yellow"]
        orange_cones = df[df['tag'] == "orange"]
        big_orange_cones = df[df['tag'] == "big_orange"]
        car_location = df[df['tag'] == "car_start"]

        # Here we parse the data and get a full list of all relevant info of
        # the cones and the car (type,x,y,yaw)
        raw_blue = []
        raw_yellow = []
        raw_orange = []
        raw_big_orange = []
        raw_car_location = (0, 0, 0, 0, 0, 0, 0, 0)

        # Note that the indexing may be confusing, pandas has inserted an "index" column,
        # so all indices that you would expect should be shifted upwards by 1

        # Blue cones
        for bluecone in blue_cones.itertuples():
            x = bluecone[2]
            y = bluecone[3]
            x_cov = bluecone[5]
            y_cov = bluecone[6]
            xy_cov = bluecone[7]
            raw_blue.append(("blue", 1.0 * x, 1.0 * y, 0, x_cov, y_cov, xy_cov))

        # Yellow cones
        for yellowcone in yellow_cones.itertuples():
            x = yellowcone[2]
            y = yellowcone[3]
            x_cov = bluecone[5]
            y_cov = bluecone[6]
            xy_cov = bluecone[7]
            raw_yellow.append(("yellow", 1.0 * x, 1.0 * y, 0, x_cov, y_cov, xy_cov))

        # Orange cones
        for orangecone in orange_cones.itertuples():
            x = orangecone[2]
            y = orangecone[3]
            x_cov = bluecone[5]
            y_cov = bluecone[6]
            xy_cov = bluecone[7]
            raw_orange.append(("orange", 1.0 * x, 1.0 * y, 0, x_cov, y_cov, xy_cov))

        # Big orange cones
        for big_orangecone in big_orange_cones.itertuples():
            x = big_orangecone[2]
            y = big_orangecone[3]
            x_cov = bluecone[5]
            y_cov = bluecone[6]
            xy_cov = bluecone[7]
            raw_big_orange.append(("big_orange", 1.0 * x, 1.0 * y, 0, x_cov, y_cov, xy_cov))

        # Car start
        for c in car_location.itertuples():
            raw_car_location = ("car", 1.0 * c[2], 1.0 * c[3], c[4], 0, 0, 0)

        # Combine
        all_cones = (raw_blue + raw_yellow + raw_orange + raw_big_orange)

        # Create launch file
        launch_template_file = os.path.join(TRACKS_SHARE, 'resource/randgen_launch_template')
        with open(launch_template_file, "r") as launch_template:
            # .launches need to point to .worlds and model files of the same name,
            # so here we are pasting in copies of the relevant filename.
            launch_merged = "".join(launch_template)
            launch_merged = GENERATED_FILENAME.join(launch_merged.split("%FILLNAME%"))

            # Fill in the car's position
            launch_merged = str(raw_car_location[1]).join(launch_merged.split('%PLACEX%'))
            launch_merged = str(raw_car_location[2]).join(launch_merged.split('%PLACEY%'))
            launch_merged = str(raw_car_location[3]).join(launch_merged.split('%PLACEROTATION%'))

            # Write launch file.
            launch_out_filepath = os.path.join(
                TRACKS_SHARE, 'launch', GENERATED_FILENAME + '.launch')
            with open(launch_out_filepath, "w") as launch_out:
                launch_out.write(launch_merged)

        # Create world file
        world_template_filepath = os.path.join(TRACKS_SHARE, 'resource', 'randgen_world_template')
        with open(world_template_filepath, "r") as world_template:
            # The world file needs to point to the correct model folder,
            # which conveniently has the same name as the world file itself.
            world_merged = "".join(world_template)
            world_merged = GENERATED_FILENAME.join(world_merged.split("%FILLNAME%"))

            # Write world file
            world_out_filepath = os.path.join(TRACKS_SHARE, 'worlds', GENERATED_FILENAME + ".world")
            with open(world_out_filepath, "w") as world_out:
                world_out.write(world_merged)

        # Create model folder
        MODEL_TEMPLATE_SHARE = os.path.join(TRACKS_SHARE, 'resource', 'randgen_model_template')

        # 1. Create the folder
        # If the folder does exist, it gets automatically overridden by the rest of this function
        MODEL_FOLDER = os.path.join(TRACKS_SHARE, 'models', GENERATED_FILENAME)
        if not os.path.exists(MODEL_FOLDER):
            os.mkdir(MODEL_FOLDER)

        # 2. Config file
        config_template_filepath = os.path.join(MODEL_TEMPLATE_SHARE, 'model.config')
        with open(config_template_filepath, "r") as config_template:
            # Let the config file know the name of the track it represents
            config_merged = "".join(config_template)
            config_merged = GENERATED_FILENAME.join(config_merged.split("%FILLNAME%"))

            # Write config file
            config_out_filepath = os.path.join(MODEL_FOLDER, 'model.config')
            with open(config_out_filepath, "w") as config_out:
                config_out.write(config_merged)

        # 3. SDF file
        sdf_template_filepath = os.path.join(MODEL_TEMPLATE_SHARE, 'model.sdf')
        with open(sdf_template_filepath, "r") as sdf_template:
            sdf_merged = "".join(sdf_template)
            sdf_split_again = sdf_merged.split("$===$")

            # sdf_split_again list contents:
            #        0: Main body of sdf file
            #        1: Outline of noise mesh visual data
            #        2: Outline of noise mesh collision data
            #        3: Noisecube collision data, meant for noise as
            #            a low-complexity collision to prevent falling out the world
            #        4: Outline of noise mesh visual data for innactive noise

            # Here we break up sdf_split_again into its constituent parts
            # At the end of this process we will have:
            #   sdf_main:                  The global template for the sdf
            #   sdf_model_with_collisions: The template for cone and noise models with
            #                              collision data.
            #
            # And the corresponding sdf_ghost_model and sdf_ghost_model_with_collisions
            # which store inactive (hidden) models.
            sdf_main = sdf_split_again[0]
            sdf_split_along_collisions = sdf_split_again[6].split("%FILLCOLLISION%")
            sdf_model_with_collisions = sdf_split_again[2].join(sdf_split_along_collisions)

            # Let the sdf file know which launch file it represents.
            sdf_main = GENERATED_FILENAME.join(sdf_main.split("%FILLNAME%"))

            # Calculate model data for cones
            collision_template = sdf_model_with_collisions.split("%MODELNAME%")

            def join_cone_model_data(cone_type):
                cone_name = "_".join(cone_type.split("_")[:2])
                cone_model = "model://" + cone_name
                return cone_model.join(collision_template)

            sdf_blue_cone_model = join_cone_model_data("blue_cone")
            sdf_yellow_cone_model = join_cone_model_data("yellow_cone")
            sdf_orange_cone_model = join_cone_model_data("orange_cone")
            sdf_big_orange_cone_model = join_cone_model_data("big_cone")

            # Let's place all the models!
            # We'll keep track of how many we've placed
            # so that we can give each a unique name.
            Converter.link_num = -1

            def setup_covariance(x, y, xy):
                output = str(x).join(sdf_split_again[5].split("%XCOV%"))
                output = str(y).join(output.split("%YCOV%"))
                output = str(xy).join(output.split("%XYCOV%"))
                return output

            def put_model_at_position(mod, x, y, modtype, x_cov=0.01, y_cov=0.01, xy_cov=0):
                """
                mod: model template to be placed
                x,y: x and y positions for mod

                returns model template with x,y, and link_num inserted.
                """
                Converter.link_num += 1
                mod_filled = str(y).join(mod.split("%PLACEY%"))
                mod_filled = str(x).join(mod_filled.split("%PLACEX%"))
                mod_filled = str(Converter.link_num).join(mod_filled.split("%LINKNUM%"))
                mod_filled = modtype.join(mod_filled.split("%LINKTYPE%"))
                mod_with_cov = setup_covariance(str(x_cov), str(y_cov), str(xy_cov)).join(
                    mod_filled.split("%FILLCOVARIANCE%"))
                return mod_with_cov

            sdf_allmodels = ""

            def expand_allmodels(allmods, mod, modtype, x, y, x_cov=0.01,
                                 y_cov=0.01, xy_cov=0):
                """
                Takes in a model and a pixel location, converts the pixel location to a raw
                location, and places the model inside sdf_allmodels
                """
                mod_at_p = put_model_at_position(mod, x, y, modtype, x_cov, y_cov, xy_cov)
                return allmods + "\n" + mod_at_p

            # Add all models into a template
            color_to_model = {
                "yellow": sdf_yellow_cone_model,
                "blue": sdf_blue_cone_model,
                "orange": sdf_orange_cone_model,
                "big_orange": sdf_big_orange_cone_model
            }
            for cone in all_cones:
                name, x, y, direction, x_cov, y_cov, xy_cov = cone
                if name in color_to_model:
                    # Normal cones.
                    sdf_allmodels = expand_allmodels(
                        sdf_allmodels,
                        color_to_model[name],
                        name + "_cone" if name != "big_orange" else "big_cone",
                        x,
                        y,
                        x_cov,
                        y_cov,
                        xy_cov
                    )

            # Splice the sdf file back together.
            sdf_main = sdf_allmodels.join(sdf_main.split("%FILLDATA%"))

            # Write it out.
            sdf_out_filepath = os.path.join(MODEL_FOLDER, "model.sdf")
            with open(sdf_out_filepath, "w") as sdf_out:
                sdf_out.write(sdf_main)
