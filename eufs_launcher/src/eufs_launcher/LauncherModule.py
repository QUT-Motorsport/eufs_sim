from collections import OrderedDict
from os import environ, listdir
from os.path import expanduser, isfile, join
from subprocess import Popen

import yaml
from ament_index_python.packages import get_package_share_directory
from python_qt_binding import loadUi
from python_qt_binding.QtGui import QFont
from python_qt_binding.QtWidgets import (QApplication, QCheckBox, QComboBox,
                                         QLabel, QPushButton, QWidget)
from qt_gui.plugin import Plugin


class EUFSLauncher(Plugin):
    def __init__(self, context):
        """
        This function handles loading the launcher GUI
        and all the setting-up of the values and buttons displayed.
        """

        super(EUFSLauncher, self).__init__(context)

        # Give QObjects reasonable names
        self.setObjectName("EUFSLauncher")

        # State variables
        self.node = context.node
        self.logger = self.node.get_logger()
        self.LAUNCHER_SHARE = get_package_share_directory("eufs_launcher")
        self.TRACKS_SHARE = get_package_share_directory("eufs_tracks")
        self.CONFIG_SHARE = get_package_share_directory("eufs_config")
        self.popens = []  # Create array of popen processes

        default_plugin_yaml = join(self.CONFIG_SHARE, "config", "pluginParams.yaml")
        # copy of file
        self.plugin_yaml = join(self.CONFIG_SHARE, "config", "pluginUserParams.yaml")

        # Initialise plugin defaults that may have been changed by the user
        self.init_plugin_config(default_plugin_yaml)

        # Declare Launcher Parameters
        default_config_path = join(self.CONFIG_SHARE, "config", "launcherOptions.yaml")
        # Load in eufs_launcher parameters
        with open(default_config_path, "r") as stream:
            try:
                self.default_config = yaml.safe_load(stream)
            except yaml.YAMLError as exc:
                print(exc)
                return

        with open(self.plugin_yaml, "r") as f:
            plugin_params = yaml.safe_load(f)
        # get frame ros params
        self.BASE_FRAME = plugin_params["/**"]["ros__parameters"]["base_frame"]

        # Create QWidget
        self._widget = QWidget()
        self._widget.setObjectName("EUFSLauncherUI")
        context.add_widget(self._widget)

        # Extend the widget with all attributes and children from UI file
        self.main_ui_file = join(self.LAUNCHER_SHARE, "ui", "launcher.ui")
        loadUi(self.main_ui_file, self._widget)

        # Show _widget.windowTitle on left-top of each plugin (when it's set in _widget). This is
        # useful when you open multiple plugins at once. Also if you open multiple instances of
        # your plugin at once, these lines add number to make it easy to tell from pane to pane.
        if context.serial_number() > 1:
            the_title = self._widget.windowTitle() + (" (%d)" % context.serial_number())
            self._widget.setWindowTitle(the_title)

        # Give widget components permanent names
        self.TRACK_SELECTOR = self._widget.findChild(QComboBox, "WhichTrack")
        self.LAUNCH_BUTTON = self._widget.findChild(QPushButton, "LaunchButton")
        self.GENERATOR_BUTTON = self._widget.findChild(QPushButton, "Trackgenerator")
        self.CONVERTER_BUTTON = self._widget.findChild(QPushButton, "Trackconverter")
        self.REFRESH_TRACK_BUTTON = self._widget.findChild(
            QPushButton, "RefreshTrackButton"
        )
        self.VEHICLE_MODEL_MENU = self._widget.findChild(QComboBox, "WhichVehicleModel")
        self.COMMAND_MODE_MENU = self._widget.findChild(QComboBox, "WhichCommandMode")
        self.MODEL_PRESET_MENU = self._widget.findChild(QComboBox, "WhichModelPreset")
        self.ROBOT_NAME_MENU = self._widget.findChild(QComboBox, "WhichRobotName")

        # Check the file directory to update drop-down menu
        self.load_track_dropdowns()

        # Hook up buttons to onclick functions
        self.LAUNCH_BUTTON.clicked.connect(self.launch_button_pressed)
        self.REFRESH_TRACK_BUTTON.clicked.connect(self.load_track_dropdowns)
        self.GENERATOR_BUTTON.clicked.connect(self.generator_button_pressed)
        self.CONVERTER_BUTTON.clicked.connect(self.converter_button_pressed)
        # Setup Vehicle Models menu
        models_filepath = join(
            get_package_share_directory("eufs_models"), "models/models.txt"
        )
        vehicle_models_ = open(models_filepath, "r")
        vehicle_models = [model.strip() for model in vehicle_models_]  # remove \n
        default_model = self.default_config["eufs_launcher"]["default_vehicle_model"]
        EUFSLauncher.setup_q_combo_box(
            self.VEHICLE_MODEL_MENU, default_model, vehicle_models
        )

        # Setup Command Modes menu
        default_mode = self.default_config["eufs_launcher"]["default_command_mode"]
        modes = ["velocity", "acceleration"]
        EUFSLauncher.setup_q_combo_box(self.COMMAND_MODE_MENU, default_mode, modes)

        # Setup Conditions menu
        default_mode = self.default_config["eufs_launcher"]["default_vehicle_preset"]
        self.MODEL_CONFIGS = {
            "DryTrack": "configDry.yaml",
            "WetTrack": "configWet.yaml",
        }
        EUFSLauncher.setup_q_combo_box(
            self.MODEL_PRESET_MENU, default_mode, self.MODEL_CONFIGS.keys()
        )

        # Setup Robot Name menu
        default_mode = self.default_config["eufs_launcher"]["default_robot_name"]
        racecars_filepath = join(
            get_package_share_directory("eufs_racecar"), "racecars.txt"
        )
        racecars_ = open(racecars_filepath, "r")
        racecars = [racecar.strip() for racecar in racecars_]  # remove \n
        EUFSLauncher.setup_q_combo_box(self.ROBOT_NAME_MENU, default_mode, racecars)

        # Add buttons from yaml file
        checkboxes = OrderedDict(
            sorted(
                self.default_config["eufs_launcher"]["checkboxes"].items(),
                key=lambda x: x[1]["priority"],
            )
        )
        self.checkbox_effect_mapping = []
        self.checkbox_parameter_mapping = []
        starting_xpos = 170
        starting_ypos = 257
        counter = 0
        for key, value in checkboxes.items():
            cur_xpos = starting_xpos + 100 * (counter % 2)
            cur_ypos = starting_ypos + 15 * (counter // 2)
            cur_cbox = QCheckBox(checkboxes[key]["label"], self._widget)
            cur_cbox.setChecked(checkboxes[key]["checked_on_default"])
            cur_cbox.setGeometry(cur_xpos, cur_ypos, 300, 30)
            cur_cbox.setFont(QFont("Sans Serif", 7))
            if "parameter_triggering" in checkboxes[key]:
                # This handles parameter details that will be passed to the
                # `simulation.launch.py` backbone file depending on whether the
                # checkbox is on or off
                self.checkbox_parameter_mapping.append(
                    (
                        cur_cbox,
                        checkboxes[key]["parameter_triggering"]["if_on"].keys(),
                        checkboxes[key]["parameter_triggering"]["if_off"].keys(),
                    )
                )

            setattr(self, checkboxes[key]["name"].upper(), cur_cbox)
            counter += 1

        # Looping over all widget to fix scaling issue via manual scaling
        # Scaling done via magically comparing the width to the 'default' 1700 pixels
        rec = QApplication.desktop().screenGeometry()
        scalar_multiplier = rec.width() / 1700.0
        for widget in self._widget.children():
            if hasattr(widget, "geometry"):
                geom = widget.geometry()
                if not isinstance(widget, QLabel):
                    new_width = geom.width() * scalar_multiplier
                else:
                    new_width = geom.width() * scalar_multiplier + 200
                widget.setGeometry(
                    int(geom.x() * scalar_multiplier),
                    int(geom.y() * scalar_multiplier),
                    int(new_width),
                    int(geom.height() * (scalar_multiplier)),
                )

    @staticmethod
    def setup_q_combo_box(q_combo_box, default_mode, modes):
        q_combo_box.clear()
        if default_mode in modes:
            q_combo_box.addItem(default_mode)
        for mode in modes:
            if mode != default_mode:
                q_combo_box.addItem(mode)

    def load_track_dropdowns(self):
        """Peruses file system for files to add to the drop-down menus of the launcher."""

        # Clear the dropdowns
        self.TRACK_SELECTOR.clear()
        # Get tracks from eufs_tracks package
        world_dir_path = join(self.TRACKS_SHARE, "worlds")
        world_files = [
            f for f in listdir(world_dir_path) if isfile(join(world_dir_path, f))
        ]

        # Remove "blacklisted" files (ones that don't define tracks)
        blacklist_filepath = join(self.TRACKS_SHARE, "worlds/blacklist.txt")
        with open(blacklist_filepath, "r") as f:
            blacklist = [f.strip() for f in f.readlines()]  # remove \n
            world_files = [f for f in world_files if f not in blacklist]

        # Add Tracks to Track Selector
        base_track = self.default_config["eufs_launcher"]["base_track"]
        if base_track in world_files:
            self.TRACK_SELECTOR.addItem(base_track.split(".")[0])
        for f in world_files:
            if f != base_track:
                self.TRACK_SELECTOR.addItem(f.split(".")[0])

    def launch_button_pressed(self):
        """
        Launches Gazebo and spawns the car.
        """
        self.logger.info("Launching Nodes...")

        # Calculate parameters to pass
        track_layout = f"track:={self.TRACK_SELECTOR.currentText()}"
        vehicle_model = f"vehicle_model:={self.VEHICLE_MODEL_MENU.currentText()}"
        command_mode = f"command_mode:={self.COMMAND_MODE_MENU.currentText()}"
        model_config = self.MODEL_CONFIGS[self.MODEL_PRESET_MENU.currentText()]
        vehicle_config = f"vehicle_config:={model_config}"
        robot_name = f"robot_name:={self.ROBOT_NAME_MENU.currentText()}"
        base_frame = f"base_frame:={self.BASE_FRAME}"
        parameters_to_pass = [
            track_layout,
            vehicle_model,
            command_mode,
            vehicle_config,
            robot_name,
            base_frame,
        ]

        # Get vehicle model information
        self.logger.info(f"Vehicle model: {self.VEHICLE_MODEL_MENU.currentText()}")
        self.logger.info(f"Command mode: {self.COMMAND_MODE_MENU.currentText()}")
        self.logger.info(f"Preset: {model_config}")
        self.logger.info(
            f"Robot description file: {self.ROBOT_NAME_MENU.currentText()}"
        )

        # Get checkbox parameter information
        for checkbox, param_if_on, param_if_off in self.checkbox_parameter_mapping:
            if checkbox.isChecked():
                self.logger.info(f"Checkbox enabled: {param_if_on}")
                parameters_to_pass.extend(param_if_on)
            else:
                self.logger.info(f"Checkbox disabled: {param_if_off}")
                parameters_to_pass.extend(param_if_off)

        # Rewrite yaml file with new parameters
        for parameter in parameters_to_pass:
            self.rewrite_yaml_config(parameter)

        noise_config = join(
            get_package_share_directory("eufs_config"),
            "config",
            "motionNoise.yaml",
        )
        self.rewrite_yaml_config(str("noise_config:=" + noise_config))

        vehicle_config = join(
            get_package_share_directory("eufs_config"),
            "config",
            model_config,
        )
        self.rewrite_yaml_config(str("vehicle_config:=" + vehicle_config))

        # Here we launch `simulation.launch.py`.
        self.launch_with_args(
            "eufs_launcher", "simulation.launch.py", parameters_to_pass
        )

        # Trigger launch files hooked to checkboxes
        for checkbox, effect_on, effect_off in self.checkbox_effect_mapping:
            if checkbox.isChecked():
                effect_on()
            else:
                effect_off()

        self.LAUNCH_BUTTON.setEnabled(False)

    def rewrite_yaml_config(self, parameter):
        """Rewrites a yaml file with a new key-value pair."""
        # extract key and value from parameter, form: key:=value
        key, value = parameter.split(":=")
        with open(self.plugin_yaml, "r") as f:
            data = yaml.safe_load(f)

        # iterate through the three nodes in the yaml file
        for node in data:
            # check if key already exists
            if key in data[node]["ros__parameters"]:
                self.logger.info(f"Overwriting {key} in {node} with {value}")
                data[node]["ros__parameters"][key] = value
                # check if boolean
                if value == "true":
                    data[node]["ros__parameters"][key] = True
                elif value == "false":
                    data[node]["ros__parameters"][key] = False

        with open(self.plugin_yaml, "w") as f:
            yaml.safe_dump(data, f)

    def init_plugin_config(self, default_yaml_file):
        """Initializes a yaml file with default values."""
        # make a copy of the default yaml file from config folder
        # so any user options can be updated without affecting the default

        with open(default_yaml_file, "r") as f:
            data = yaml.safe_load(f)
        with open(self.plugin_yaml, "w") as f:
            yaml.safe_dump(data, f)

    def generator_button_pressed(self):
        self.run_without_args(
            "eufs_tracks",
            "eufs_tracks_generator",
        )

    def converter_button_pressed(self):
        self.run_without_args(
            "eufs_tracks",
            "eufs_tracks_converter",
        )

    def run_without_args(self, package, program_name):
        """Runs ros node."""
        command = ["stdbuf", "-o", "L", "ros2", "run", package, program_name]
        self.logger.info(f"Command: {' '.join(command)}")
        process = Popen(command)
        self.popens.append(process)

    def launch_with_args(self, package, launch_file, args):
        """Launches ros node."""
        command = [
            "stdbuf",
            "-o",
            "L",
            "ros2",
            "launch",
            package,
            launch_file,
            "use_sim_time:=True",
            "log_level:=debug",
        ] + args
        self.logger.info(f"Command: {' '.join(command)}")
        process = Popen(command)
        self.popens.append(process)

    def shutdown_plugin(self):
        """Kill all nodes."""
        self.logger.info("Shutdown Engaged...")

        for process in self.popens:
            process.terminate()

        self.logger.info("All nodes killed")
