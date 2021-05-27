import math
from collections import OrderedDict
from os import listdir
from os.path import join, isfile
from subprocess import Popen
from random import uniform

import yaml

import pandas as pd

from ament_index_python.packages import get_package_share_directory

from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget, QComboBox, QPushButton, QSlider, QCheckBox, QLabel, QApplication
from python_qt_binding.QtGui import QFont
from qt_gui.plugin import Plugin

from eufs_tracks.ConversionTools import ConversionTools as Converter


class EUFSLauncher(Plugin):

    def __init__(self, context):
        """
                This function handles loading the launcher GUI
                and all the setting-up of the values and buttons displayed.
                """
        super(EUFSLauncher, self).__init__(context)

        # Give QObjects reasonable names
        self.setObjectName('EUFSLauncher')

        self.node = context.node

        self.logger = self.node.get_logger()

        # Declare and then get parameters inputted from launch file
        self.node.declare_parameter(
            "config",
            value="config/eufs_launcher.yaml"
        )
        self.node.declare_parameter(
            "config_loc",
            value="eufs_launcher"
        )

        self.node.declare_parameter(
            "gui",
            value=True
        )

        yaml_to_load = self.node.get_parameter("config").value
        loc_to_load = self.node.get_parameter("config_loc").value
        use_gui = self.node.get_parameter("gui").value

        # Load in eufs_launcher parameters
        # yaml_to_load is a special variable that is set by `eufs_launcher.py`
        # in `eufs_launcher/scripts`.  It defaults to `config/eufs_launcher.yaml`, but
        # can be passed in using the `config` param.  Example:
        # `ros2 launch eufs_launcher eufs_launcher.launch.py config:=config/example.yaml`
        # Will load in example.yaml instead.

        yaml_loc = join(
            get_package_share_directory(loc_to_load),
            yaml_to_load.split(".")[0] + ".yaml"
        )

        with open(yaml_loc, 'r') as stream:
            try:
                self.default_config = yaml.safe_load(stream)
            except yaml.YAMLError as exc:
                print(exc)
                return

        # Create QWidget
        self._widget = QWidget()

        # Get path to UI file which should be in the "resource" folder of this package
        self.main_ui_file = join(get_package_share_directory('eufs_launcher'),
                                 'resource',
                                 'launcher.ui',
                                 )

        # Extend the widget with all attributes and children from UI file
        loadUi(self.main_ui_file, self._widget)

        # Give QObjects reasonable names
        self._widget.setObjectName('EUFSLauncherUI')

        # Show _widget.windowTitle on left-top of each plugin (when
        # it's set in _widget). This is useful when you open multiple
        # plugins at once. Also if you open multiple instances of your
        # plugin at once, these lines add number to make it easy to
        # tell from pane to pane.
        if context.serial_number() > 1:
            the_title = (self._widget.windowTitle() + (' (%d)' % context.serial_number()))
            self._widget.setWindowTitle(the_title)

        # Set the magic number for the maximum cone noise allowed
        self.MAX_CONE_NOISE = 0.4
        self.MAX_COLOR_NOISE = 1.0

        # Store gazebo's path as it is used quite a lot:
        self.GAZEBO = get_package_share_directory('eufs_gazebo')

        # Give widget components permanent names
        self.TRACK_SELECTOR = self._widget.findChild(QComboBox, "WhichTrack")
        self.LAUNCH_BUTTON = self._widget.findChild(QPushButton, "LaunchButton")

        self.NOISE_SLIDER = self._widget.findChild(QSlider, "Noisiness")
        self.VEHICLE_MODEL_MENU = self._widget.findChild(QComboBox, "WhichVehicleModel")
        self.COMMAND_MODE_MENU = self._widget.findChild(QComboBox, "WhichCommandMode")
        self.CONE_NOISE_SLIDER = self._widget.findChild(QSlider, "ConeNoisiness")
        self.COLOR_NOISE_SLIDER = self._widget.findChild(QSlider, "ConeColorNoisiness")

        # Check the file directory to update drop-down menu
        self.load_track_dropdowns()

        # Hook up buttons to onclick functions
        self.LAUNCH_BUTTON.clicked.connect(self.launch_button_pressed)

        # Create array of popen processes
        self.popens = []

        # Add widget to the user interface
        context.add_widget(self._widget)

        # Miscellaneous final initializations:
        self.has_launched_ros = False
        self.launch_file_override = None
        self.popen_process = None

        # While in the process of changing sliders,
        # we don't want our monitor function to be rapidly firing
        # So we toggle this variable when ready
        self.ignore_slider_changes = False

        # Setup Vehicle Models menu
        # Clear the dropdowns
        self.VEHICLE_MODEL_MENU.clear()

        # Remove "blacklisted" files (ones that don't define vehicle models)
        models_filepath = join(
            get_package_share_directory('eufs_gazebo_plugins'),
            'gazebo_race_car_model/src/models/models.txt'
        )
        vehicle_models_ = open(models_filepath, "r")
        vehicle_models = [model.strip() for model in vehicle_models_]  # remove \n

        default_model = self.default_config["eufs_launcher"]["default_vehicle_model"]
        if default_model in vehicle_models:
            self.VEHICLE_MODEL_MENU.addItem(default_model)
        for model in vehicle_models:
            if model != default_model:
                self.VEHICLE_MODEL_MENU.addItem(model)

        # Setup Command Modes menu
        self.COMMAND_MODE_MENU.clear()
        default_mode = self.default_config["eufs_launcher"]["default_command_mode"]
        modes = ["acceleration", "velocity"]
        if default_mode in modes:
            self.COMMAND_MODE_MENU.addItem(default_mode)
        for mode in modes:
            if mode != default_mode:
                self.COMMAND_MODE_MENU.addItem(mode)

        # Add buttons from yaml file
        checkboxes = OrderedDict(
            sorted(
                self.default_config["eufs_launcher"]["checkboxes"].items(),
                key=lambda x: x[1]["priority"]
            )
        )
        self.checkbox_effect_mapping = []
        self.checkbox_parameter_mapping = []
        starting_xpos = 170
        starting_ypos = 290
        counter = 0
        for key, value in checkboxes.items():
            cur_xpos = starting_xpos + 100 * (counter % 2)
            cur_ypos = starting_ypos + 15 * (counter // 2)
            cur_cbox = QCheckBox(checkboxes[key]["label"], self._widget)
            cur_cbox.setChecked(checkboxes[key]["checked_on_default"])
            cur_cbox.setGeometry(cur_xpos, cur_ypos, 300, 30)
            cur_cbox.setFont(QFont("Sans Serif", 7))
            if "package" in checkboxes[key] and "launch_file" in checkboxes[key]:
                # This handles any launch files that the checkbox will launch
                # if selected.
                if "args" in checkboxes[key]:
                    cur_cbox_args = self.arg_to_list(checkboxes[key]["args"])
                else:
                    cur_cbox_args = []
                # The weird double-lambda thing is because python lambda-captures
                # by reference, not value, and so consequently since the `key`
                # variable is used every loop, at the end of the loops all of
                # these lambda functions would actually be refering to the *last*
                # loop's key, not each individual loop's key.
                # To solve that, we force key into a local variable by wrapping
                # our contents with a lambda taking it as a parameter, then
                # immediately passing it in.
                # We do the same for all other changing variables.
                self.checkbox_effect_mapping.append((
                    cur_cbox,
                    (lambda key, cur_cbox_args: (
                        lambda: self.launch_node_with_args(
                            checkboxes[key]["package"],
                            checkboxes[key]["launch_file"],
                            cur_cbox_args
                        )
                    ))(key, cur_cbox_args),
                    (lambda key: lambda: None)(key)
                ))
            if "parameter_triggering" in checkboxes[key]:
                # This handles parameter details that will be passed to the
                # `simulation.launch.py` backbone file depending on whether the
                # checkbox is on or off
                self.checkbox_parameter_mapping.append((
                    cur_cbox,
                    self.arg_to_list(checkboxes[key]["parameter_triggering"]["if_on"]),
                    self.arg_to_list(checkboxes[key]["parameter_triggering"]["if_off"])
                ))
            if "ros_param_triggering" in checkboxes[key]:
                # This handles ros parameters that need to be set
                check = checkboxes[key]["ros_param_triggering"]
                self.checkbox_effect_mapping.append((
                    cur_cbox,
                    (lambda check: (
                        lambda: rospy.set_param(
                            check["param_name"],
                            check["if_on"]
                        )
                    ))(check),
                    (lambda check: (
                        lambda: rospy.set_param(
                            check["param_name"],
                            check["if_off"])
                    ))(check)
                ))

            setattr(self, checkboxes[key]["name"].upper(), cur_cbox)
            counter += 1

        # Hide color and cone noise slider by default, since the average user shouldn't
        # stumble upon them.
        self.COLOR_NOISE_SLIDER.setVisible(False)
        self.CONE_NOISE_SLIDER.setVisible(False)
        self._widget.findChild(QLabel, "ConeColorNoiseLabel").setVisible(False)
        self._widget.findChild(QLabel, "ConeNoiseLabel").setVisible(False)

        # Read in default noise levels
        self.set_noise_level(
            float(self.default_config["eufs_launcher"]["object_noise_default"])
        )
        self.set_cone_noise_level(
            float(self.default_config["eufs_launcher"]["cone_noise_default"])
        )
        self.set_color_noise_level(
            float(self.default_config["eufs_launcher"]["color_noise_default"])
        )

        self.DEBUG_SHUTDOWN = False

        # Looping over all widgest to fix scaling issue via manual scaling
        # Scaling done via magically comparing the width to the 'default' 1700 pixels
        rec = QApplication.desktop().screenGeometry()
        scaler_multiplier = rec.width() / 1700.0
        for widget in self._widget.children():
            if hasattr(widget, 'geometry'):
                geom = widget.geometry()
                new_width = (
                    geom.width() * (scaler_multiplier) if not isinstance(widget, QLabel)
                    else geom.width() * (scaler_multiplier) + 200
                )
                widget.setGeometry(
                    geom.x() * scaler_multiplier,
                    geom.y() * scaler_multiplier,
                    new_width,
                    geom.height() * (scaler_multiplier)
                )

        # If use_gui is false, we jump straight into launching the track
        # use_gui is a special variable set by `eufs_launcher.py`
        if not use_gui:
            self.launch_button_pressed()

    def load_track_dropdowns(self):
        """
                Peruses file system for files to add to the drop-down menus of the launcher.
                """

        # Clear the dropdowns
        self.TRACK_SELECTOR.clear()
        # Get tracks from eufs_gazebo package
        relevant_path = join(self.GAZEBO, 'launch')
        launch_files = [
            f for f in listdir(relevant_path) if isfile(join(relevant_path, f))
        ]

        # Remove "blacklisted" files (ones that don't define tracks)
        blacklist_filepath = join(
            self.GAZEBO,
            'launch/blacklist.txt'
        )
        blacklist_ = open(blacklist_filepath, "r")
        blacklist = [f.strip() for f in blacklist_]  # remove \n
        launch_files = [f for f in launch_files if f not in blacklist]

        # Add Tracks to Track Selector
        base_track = self.default_config["eufs_launcher"]["base_track"]
        if base_track in launch_files:
            self.TRACK_SELECTOR.addItem(base_track.split(".")[0])
        for f in launch_files:
            if f != base_track:
                self.TRACK_SELECTOR.addItem(f.split(".")[0])

    def arg_to_list(self, d):
        """Converts yaml arg dict to parameter list"""
        to_return = []
        for k, v in d.items():
            to_return.append(str(k))
        return list(to_return)

    def get_noise_level(self):
        """Returns the object noise slider's noise level."""

        noise_level_widget = self.NOISE_SLIDER
        numerator = (1.0 * (noise_level_widget.value() - noise_level_widget.minimum()))
        denominator = (noise_level_widget.maximum() - noise_level_widget.minimum())
        return numerator / denominator

    def set_noise_level(self, new_noise_level):
        """
                Sets the object noise slider's level.
                Code may look complicated, but it's really just inverting get_noise_level to solve
                for `noise_level_widget.value()`
                """
        noise_level_widget = self.NOISE_SLIDER
        denominator = (noise_level_widget.maximum() - noise_level_widget.minimum())
        numerator = new_noise_level * denominator
        new_value = numerator + noise_level_widget.minimum()
        noise_level_widget.setValue(
            new_value
        )

    def get_cone_noise_level(self):
        """Returns the cone noise slider's noise level."""

        noise_level_widget = self.CONE_NOISE_SLIDER
        numerator = (1.0 * (noise_level_widget.value() - noise_level_widget.minimum()))
        denominator = (noise_level_widget.maximum() - noise_level_widget.minimum())
        return self.MAX_CONE_NOISE * numerator / denominator

    def set_cone_noise_level(self, new_noise_level):
        """
                Sets the cone noise slider's level.
                Code may look complicated, but it's really just inverting get_cone_noise_level to solve
                for `cone_noise_level_widget.value()`
                """
        noise_level_widget = self.CONE_NOISE_SLIDER
        denominator = (noise_level_widget.maximum() - noise_level_widget.minimum())
        numerator = new_noise_level * denominator / self.MAX_CONE_NOISE
        new_value = numerator + noise_level_widget.minimum()
        noise_level_widget.setValue(
            new_value
        )

    def get_color_noise_level(self):
        """Returns the color noise slider's noise level."""

        noise_level_widget = self.COLOR_NOISE_SLIDER
        numerator = (1.0 * (noise_level_widget.value() - noise_level_widget.minimum()))
        denominator = (noise_level_widget.maximum() - noise_level_widget.minimum())
        return self.MAX_COLOR_NOISE * numerator / denominator

    def set_color_noise_level(self, new_noise_level):
        """
                Sets the color noise slider's level.
                Code may look complicated, but it's really just inverting get_color_noise_level to solve
                for `color_noise_level_widget.value()`
                """
        noise_level_widget = self.COLOR_NOISE_SLIDER
        denominator = (noise_level_widget.maximum() - noise_level_widget.minimum())
        numerator = new_noise_level * denominator / self.MAX_COLOR_NOISE
        new_value = numerator + noise_level_widget.minimum()
        noise_level_widget.setValue(
            new_value
        )

    def launch_button_pressed(self):
        """Launches Gazebo."""
        if self.has_launched_ros:
            # Don't let people press launch twice
            return
        self.has_launched_ros = True

        self.logger.info("Launching Nodes...")

        """
                Here is our pre-launch process:
                1: Create csv from track_to_launch
                2: Kill random noise tiles in accordance with the noise value
                3: Shuffle cone positions slightly in accordance with cone noise value
                4: Convert that to "LAST_LAUNCH.launch"
                Launch LAST_LAUNCH.launch
                """

        # Create csv from track_to_launch
        self.logger.info("Creating csv...")
        track_to_launch = self.TRACK_SELECTOR.currentText() + ".launch"
        full_path = join(
            self.GAZEBO,
            'launch',
            track_to_launch
        )
        Converter.convert(
            "launch",
            "csv",
            full_path,
            override_name="LAST_LAUNCH"
        )

        # Get noise level
        noise_level = self.get_noise_level()
        cone_noise_level = self.get_cone_noise_level()
        color_noise_level = self.get_color_noise_level()
        self.logger.info("Launching " + track_to_launch + " With Noise Level: " + str(noise_level))

        # Remove relevant random noise tiles from the csv
        csv_path = join(
            self.GAZEBO,
            'tracks',
            "LAST_LAUNCH.csv"
        )
        loaded_csv = pd.read_csv(
            csv_path,
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
        loaded_csv = loaded_csv[
            (
                    (loaded_csv["tag"] != "inactive_noise") &
                    (loaded_csv["tag"] != "active_noise")
            ) |
            (uniform(0, 1) < noise_level)
            ]

        for idx, val in loaded_csv.iterrows():
            # Activate remaining noise
            if loaded_csv.loc[idx, "tag"] == "inactive_noise":
                loaded_csv.loc[idx, "tag"] = "active_noise"

            # Shuffle cones
            cone_types = ["blue", "yellow", "orange", "big_orange"]
            if loaded_csv.loc[idx, "tag"] in cone_types:
                uniform_angle = uniform(0, 2 * math.pi)
                uniform_radius = uniform(0, cone_noise_level)
                del_x = uniform_radius * math.cos(uniform_angle)
                del_y = uniform_radius * math.sin(uniform_angle)
                loaded_csv.loc[idx, "x"] = float(loaded_csv.loc[idx, "x"]) + del_x
                loaded_csv.loc[idx, "y"] = float(loaded_csv.loc[idx, "y"]) + del_y

        loaded_csv.to_csv(
            csv_path,
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

        # Convert csv to launch file
        Converter.convert(
            "csv",
            "launch",
            csv_path,
            params={"keep_all_noise": True, "noise": 1},
            override_name="LAST_LAUNCH"
        )
        track_to_launch = "LAST_LAUNCH.launch"

        # Re-create csv file because the csv-to-launch file re-centers the data
        # A different fix would be to add another metapixel to trackimages that stores
        # What the car's position should be.
        # But this will work for now
        Converter.convert(
            "launch",
            "csv",
            join(
                self.GAZEBO,
                'launch',
                "LAST_LAUNCH.launch"
            ),
            conversion_suffix=""
        )

        # And now strip the launch file of the metadata that causes warnings
        # This step is non-essential, but will result in a lot of terminal warnings
        # if left out
        Converter.convert(
            "csv",
            "launch",
            join(
                self.GAZEBO,
                'tracks',
                "LAST_LAUNCH.csv"
            ),
            params={"keep_all_noise": False, "noise": 1},
            conversion_suffix=""
        )

        # Get vehicle model information
        self.logger.info("With " + self.VEHICLE_MODEL_MENU.currentText() + " Vehicle Model " +
                         "using the " + self.COMMAND_MODE_MENU.currentText() + " command mode")

        vehicle_model = "vehicleModel:=" + self.VEHICLE_MODEL_MENU.currentText()
        command_mode = "commandMode:=" + self.COMMAND_MODE_MENU.currentText()

        # Get checkbox parameter information
        parameters_to_pass = ["track:=" + track_to_launch.split(".")[0], vehicle_model, command_mode]
        for checkbox, param_if_on, param_if_off in self.checkbox_parameter_mapping:
            if checkbox.isChecked():
                parameters_to_pass.extend(param_if_on)
            else:
                parameters_to_pass.extend(param_if_off)

        # Here we launch `simulation.launch.py`.
        self.popen_process = self.launch_node_with_args(
            'eufs_launcher',
            'simulation.launch.py',
            parameters_to_pass
        )

        # Trigger launch files hooked to checkboxes
        for checkbox, effect_on, effect_off in self.checkbox_effect_mapping:
            if checkbox.isChecked():
                effect_on()
            else:
                effect_off()

        # Hard-Coded Map Effect
        if hasattr(self, "FAST_SLAM_LOAD_MAP") and self.FAST_SLAM_LOAD_MAP.isChecked():
            in_path = join(
                self.GAZEBO,
                'tracks',
                "LAST_LAUNCH.csv"
            )
            the_csv = pd.read_csv(
                in_path,
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
            cone_types = ["blue", "yellow", "orange", "big_orange"]
            for index, row in the_csv.iterrows():
                if uniform(0, 1) < color_noise_level and row["tag"] in cone_types:
                    the_csv.at[index, "tag"] = "unknown_color"
            out_path = join(
                self.GAZEBO,
                'tracks',
                "FAST_SLAM_PRELOADED_MAP.csv"
            )
            the_csv.to_csv(
                out_path,
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
            rospy.set_param("/slam/map_path", out_path)

        # Auto-launch default scripts in yaml
        scripts = self.default_config["eufs_launcher"]["on_startup"]
        for key, value in scripts.items():
            if "args" in scripts[key]:
                cur_script_args = self.arg_to_list(scripts[key]["args"])
            else:
                cur_script_args = []
            self.launch_node_with_args(scripts[key]["package"], scripts[key]["launch_file"], cur_script_args)

        self.LAUNCH_BUTTON.setEnabled(False)

    def launch_node(self, package, launch_file):
        """Wrapper for launch_node_with_args"""
        self.launch_node_with_args(package, launch_file, [])

    def launch_node_with_args(self, package, launch_file, args):
        """
                Launches ros node.
                """
        command = ' '.join(["ros2 launch", package, launch_file] + args)
        process = Popen(command, shell=True)
        self.popens.append(process)
        return process

    def shutdown_plugin(self):
        """Kill all nodes."""
        self.logger.info("Shutdown Engaged...")

        for process in self.popens:
            process.terminate()

        self.logger.info("All nodes killed")
