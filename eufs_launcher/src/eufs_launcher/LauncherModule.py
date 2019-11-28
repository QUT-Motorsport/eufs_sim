import os
import rospy
import rospkg
import roslaunch
import rosnode
import math
import time

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import (QWidget, QComboBox, QPushButton, QSlider, QRadioButton, QCheckBox, QMainWindow,
                                         QLabel, QLineEdit, QApplication)

from os import listdir
from os.path import isfile, join

from subprocess import Popen

from PIL import Image
from PIL import ImageDraw
from random import randrange, uniform

from TrackGenerator import TrackGenerator as Generator
from TrackGenerator import GeneratorContext

from ConversionTools import ConversionTools as Converter


class EufsLauncher(Plugin):

        def __init__(self, context):
                """
                This function handles loading the launcher GUI
                and all the setting-up of the values and buttons displayed.
                """
                super(EufsLauncher, self).__init__(context)

                # Give QObjects reasonable names
                self.setObjectName('EufsLauncher')

                # Process standalone plugin command-line arguments
                from argparse import ArgumentParser
                parser = ArgumentParser()
                # Add argument(s) to the parser.
                parser.add_argument("-q", "--quiet", action="store_true",
                                    dest="quiet",
                                    help="Put plugin in silent mode")
                args, unknowns = parser.parse_known_args(context.argv())
                if not args.quiet:
                        print 'arguments: ', args
                        print 'unknowns: ', unknowns

                # Create QWidget
                self._widget = QWidget()

                # Get path to UI file which should be in the "resource" folder of this package
                self.main_ui_file = os.path.join(
                                                 rospkg.RosPack().get_path('eufs_launcher'),
                                                 'resource',
                                                 'Launcher.ui'
                )
                self.sketcher_ui_file = os.path.join(
                                                 rospkg.RosPack().get_path('eufs_launcher'),
                                                 'resource',
                                                 'Sketcher.ui'
                )

                # Extend the widget with all attributes and children from UI file
                loadUi(self.main_ui_file, self._widget)

                # Give QObjects reasonable names
                self._widget.setObjectName('EufsLauncherUI')

                # Show _widget.windowTitle on left-top of each plugin (when
                # it's set in _widget). This is useful when you open multiple
                # plugins at once. Also if you open multiple instances of your
                # plugin at once, these lines add number to make it easy to
                # tell from pane to pane.
                if context.serial_number() > 1:
                        the_title = (self._widget.windowTitle() + (' (%d)' % context.serial_number()))
                        self._widget.setWindowTitle(the_title)

                # Resize correctly
                self._widget.setFixedWidth(1200)

                # Store gazebo's path as it is used quite a lot:
                self.GAZEBO = rospkg.RosPack().get_path('eufs_gazebo')

                # Give widget components permanent names
                self.PRESET_SELECTOR = self._widget.findChild(QComboBox, "WhichPreset")
                self.TRACK_SELECTOR = self._widget.findChild(QComboBox, "WhichTrack")
                self.IMAGE_SELECTOR = self._widget.findChild(QComboBox, "WhichImage")
                self.LAUNCH_BUTTON = self._widget.findChild(QPushButton, "LaunchButton")
                self.GENERATOR_BUTTON = self._widget.findChild(QPushButton, "GenerateButton")
                self.LOAD_IMAGE_BUTTON = self._widget.findChild(QPushButton, "LoadFromImageButton")

                self.CONVERT_BUTTON = self._widget.findChild(QPushButton, "ConvertButton")
                self.RENAME_BUTTON = self._widget.findChild(QPushButton, "RenameButton")
                self.SKETCHER_BUTTON = self._widget.findChild(QPushButton, "SketcherButton")
                self.LAX_CHECKBOX = self._widget.findChild(QCheckBox, "LaxCheckBox")
                self.CONVERT_FROM_MENU = self._widget.findChild(QComboBox, "ConvertFrom")
                self.CONVERT_TO_MENU = self._widget.findChild(QComboBox, "ConvertTo")
                self.MIDPOINT_CHECKBOX = self._widget.findChild(QCheckBox, "MidpointBox")

                self.SUFFIX_CHECKBOX = self._widget.findChild(QCheckBox, "SuffixBox")
                self.USER_FEEDBACK_LABEL = self._widget.findChild(QLabel, "UserFeedbackLabel")
                self.RENAME_FILE_TEXTBOX = self._widget.findChild(QLineEdit, "RenameFileTextbox")
                self.RENAME_FILE_HEADER = self._widget.findChild(QLabel, "RenameFileHeader")
                self.NOISE_SLIDER = self._widget.findChild(QSlider, "Noisiness")
                self.SPEED_RADIO = self._widget.findChild(QRadioButton, "SpeedRadio")
                self.TORQUE_RADIO = self._widget.findChild(QRadioButton, "TorqueRadio")
                self.PERCEPTION_CHECKBOX = self._widget.findChild(QCheckBox, "PerceptionCheckbox")

                self.VISUALISATOR_CHECKBOX = (
                        self._widget.findChild(QCheckBox, "VisualisatorCheckbox")
                )
                self.GAZEBO_GUI_CHECKBOX = (
                        self._widget.findChild(QCheckBox, "GazeboGuiCheckbox")
                )

                self.FILE_FOR_CONVERSION_BOX = self._widget.findChild(
                        QComboBox,
                        "FileForConversion"
                )
                self.FULL_STACK_COPY_BUTTON = self._widget.findChild(
                        QCheckBox,
                        "FullStackCopyButton"
                )
                self.FULL_STACK_TRACK_GEN_BUTTON = self._widget.findChild(
                        QCheckBox,
                        "FullStackTrackGenButton"
                )
                self.FULL_STACK_IMAGE_BUTTON = self._widget.findChild(
                        QCheckBox,
                        "FullStackImageButton"
                )

                self.MIN_STRAIGHT_SLIDER = self._widget.findChild(QSlider, "Param_MIN_STRAIGHT")
                self.MAX_STRAIGHT_SLIDER = self._widget.findChild(QSlider, "Param_MAX_STRAIGHT")
                self.MIN_CTURN_SLIDER = self._widget.findChild(QSlider, "Param_MIN_CTURN")
                self.MAX_CTURN_SLIDER = self._widget.findChild(QSlider, "Param_MAX_CTURN")
                self.MIN_HAIRPIN_SLIDER = self._widget.findChild(QSlider, "Param_MIN_HAIRPIN")
                self.MAX_HAIRPIN_SLIDER = self._widget.findChild(QSlider, "Param_MAX_HAIRPIN")
                self.HAIRPIN_PAIRS_SLIDER = self._widget.findChild(QSlider, "Param_HAIRPIN_PAIRS")
                self.MAX_LENGTH_SLIDER = self._widget.findChild(QSlider, "Param_MAX_LENGTH")
                self.TRACK_WIDTH_SLIDER = self._widget.findChild(QSlider, "Param_TRACK_WIDTH")

                self.MIN_STRAIGHT_LABEL = self._widget.findChild(QLabel, "Label_MIN_STRAIGHT")
                self.MAX_STRAIGHT_LABEL = self._widget.findChild(QLabel, "Label_MAX_STRAIGHT")
                self.MIN_CTURN_LABEL = self._widget.findChild(QLabel, "Label_MIN_CTURN")
                self.MAX_CTURN_LABEL = self._widget.findChild(QLabel, "Label_MAX_CTURN")
                self.MIN_HAIRPIN_LABEL = self._widget.findChild(QLabel, "Label_MIN_HAIRPIN")
                self.MAX_HAIRPIN_LABEL = self._widget.findChild(QLabel, "Label_MAX_HAIRPIN")
                self.HAIRPIN_PAIRS_LABEL = self._widget.findChild(QLabel, "Label_HAIRPIN_PAIRS")
                self.MAX_LENGTH_LABEL = self._widget.findChild(QLabel, "Label_MAX_LENGTH")
                self.TRACK_WIDTH_LABEL = self._widget.findChild(QLabel, "Label_TRACK_WIDTH")

                # Check the file directory to update drop-down menu
                self.load_track_and_images()

                # Get presets
                preset_names = Generator.get_preset_names()

                # Add Presets to Preset Selector (always put Computer Friendly first)
                default_preset = Generator.get_default_preset()
                if default_preset in preset_names:
                        self.PRESET_SELECTOR.addItem(default_preset)
                for f in preset_names:
                        if f != default_preset:
                                self.PRESET_SELECTOR.addItem(f)

                # Hook up buttons to onclick functions
                self.LAUNCH_BUTTON.clicked.connect(self.launch_button_pressed)
                self.GENERATOR_BUTTON.clicked.connect(self.generator_button_pressed)
                self.LOAD_IMAGE_BUTTON.clicked.connect(self.track_from_image_button_pressed)
                self.CONVERT_BUTTON.clicked.connect(self.convert_button_pressed)
                self.RENAME_BUTTON.clicked.connect(self.copy_button_pressed)
                self.SKETCHER_BUTTON.clicked.connect(self.sketcher_button_pressed)

                # Create array of running processes
                self.processes = []
                # And also an array of running launches
                self.launches = []
                # And array of running popens (things launched by popen)
                self.popens = []

                # Add widget to the user interface
                context.add_widget(self._widget)

                # Miscellaneous final initializations:
                self.has_launched_ros = False
                self.launch_file_override = None
                self.popen_process = None

                # Space the load track button better
                self.LOAD_IMAGE_BUTTON.setText("Load Track\nFrom Image")

                # Hide track draw button as not currently working
                self.SKETCHER_BUTTON.setVisible(False)

                # Set up the Generator Params
                self.update_preset()

                # Give sliders the correct range
                self.set_slider_ranges()

                # Relabel the params
                self.keep_params_up_to_date()
                self.keep_sliders_up_to_date()

                # Hook up sliders to function that monitors when they've been changed
                self.keep_track_of_slider_changes()
                self.keep_track_of_preset_changes()

                # While in the process of changing sliders,
                # we don't want our monitor function to be rapidly firing
                # So we toggle this variable when ready
                self.ignore_slider_changes = False

                # Setup Lax Generation button
                self.LAX_CHECKBOX.setChecked(self.LAX_GENERATION)

                # Setup Gazebo Gui button
                self.GAZEBO_GUI_CHECKBOX.setChecked(True)

                # Setup Conversion Tools dropdowns
                for f in ["launch", "png", "csv"]:
                        self.CONVERT_FROM_MENU.addItem(f)
                for f in ["csv", "png", "launch", "ALL"]:
                        self.CONVERT_TO_MENU.addItem(f)

                self.update_converter_dropdown()
                self.CONVERT_FROM_MENU.currentTextChanged.connect(self.update_converter_dropdown)
                self.CONVERT_TO_MENU.currentTextChanged.connect(self.update_midpoints_box)
                self.FILE_FOR_CONVERSION_BOX.currentTextChanged.connect(self.update_copier)

                # Prep midpoints checkbox
                convert_to = self.CONVERT_TO_MENU.currentText()
                self.MIDPOINT_CHECKBOX.setChecked(True)

                # Suffix checkbox
                suffix_box = self.SUFFIX_CHECKBOX
                suffix_box.setChecked(True)

                # Track Gen full stack checkbox
                track_generator_full_stack = self.FULL_STACK_TRACK_GEN_BUTTON
                track_generator_full_stack.setChecked(True)

                # Image full stack checkbox
                image_launcher_full_stack = self.FULL_STACK_IMAGE_BUTTON
                image_launcher_full_stack.setChecked(True)

                # Copier full stack checkbox
                copier_full_stack = self.FULL_STACK_COPY_BUTTON
                copier_full_stack.setChecked(True)

                # Change label to show current selected file for the copier
                self.update_copier()

                # Get uuid of roslaunch
                self.uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
                roslaunch.configure_logging(self.uuid)
                self.DEBUG_SHUTDOWN = False

        def tell_launchella(self, what):
                """Display text in feedback box (lower left corner)."""

                self.USER_FEEDBACK_LABEL.setText(what)
                QApplication.processEvents()

        def sketcher_button_pressed(self):
                """Called when sketcher button is pressed."""

                loadUi(self.sketcher_ui_file, self._widget)

        def load_track_and_images(self):
                """
                Peruses file system for files to add to the drop-down menus of the launcher.
                """

                # Clear the dropdowns
                self.TRACK_SELECTOR.clear()
                self.IMAGE_SELECTOR.clear()
                # Get tracks from eufs_gazebo package
                relevant_path = os.path.join(self.GAZEBO, 'launch')
                launch_files = [
                        f for f in listdir(relevant_path) if isfile(join(relevant_path, f))
                ]

                # Remove "blacklisted" files (ones that don't define tracks)
                blacklist_filepath = os.path.join(
                        self.GAZEBO,
                        'launch/blacklist.txt'
                )
                blacklist_ = open(blacklist_filepath, "r")
                blacklist = [f.strip() for f in blacklist_]  # remove \n
                launch_files = [f for f in launch_files if f not in blacklist]

                # Add Tracks to Track Selector
                if "small_track.launch" in launch_files:
                        self.TRACK_SELECTOR.addItem("small_track.launch")
                for f in launch_files:
                        if f != "small_track.launch":
                                self.TRACK_SELECTOR.addItem(f)

                # Get images
                relevant_path = os.path.join(
                        self.GAZEBO,
                        'randgen_imgs'
                )
                image_files = [f for f in listdir(relevant_path) if isfile(join(relevant_path, f))]

                # Add Images to Image Selector (always put rand.png first)
                if "rand.png" in image_files:
                        self.IMAGE_SELECTOR.addItem("rand.png")
                for f in image_files:
                        if f != "rand.png" and f[-3:] == "png":
                                self.IMAGE_SELECTOR.addItem(f)

        def copy_button_pressed(self):
                """When copy button is pressed, launch ConversionTools"""

                self.tell_launchella("Copying...")

                # Copy the current file
                is_full_stack = self.FULL_STACK_COPY_BUTTON.isChecked()
                file_to_copy_to = self.RENAME_FILE_TEXTBOX.text()
                file_to_copy_from = self.FILE_FOR_CONVERSION_BOX.currentText()
                raw_name_to = file_to_copy_to.split(".")[0]
                raw_name_from = file_to_copy_from.split(".")[0]
                ending = file_to_copy_from.split(".")[-1]

                # Don't let them create null-named files
                if len(file_to_copy_to) == 0:
                        return

                # For launch files, we also need to move around the model folders
                if ending == "launch":
                        model_path = os.path.join(
                                rospkg.RosPack().get_path('eufs_description'),
                                'models',
                                raw_name_to
                        )
                        if not os.path.exists(model_path):
                                os.mkdir(model_path)

                        # Copy sdf files
                        path_from = os.path.join(
                                rospkg.RosPack().get_path('eufs_description'),
                                'models',
                                raw_name_from,
                                "model.sdf"
                        )
                        path_to = os.path.join(
                                rospkg.RosPack().get_path('eufs_description'),
                                'models',
                                raw_name_to,
                                "model.sdf"
                        )
                        Converter.copy_file(path_from, path_to)

                        # Copy config files
                        path_from = os.path.join(
                                rospkg.RosPack().get_path('eufs_description'),
                                'models',
                                raw_name_from,
                                "model.config"
                        )
                        path_to = os.path.join(
                                rospkg.RosPack().get_path('eufs_description'),
                                'models',
                                raw_name_to,
                                "model.config"
                        )
                        Converter.copy_file(path_from, path_to)

                        # Copy launch files
                        path_from = os.path.join(
                                self.GAZEBO,
                                'launch',
                                file_to_copy_from
                        )
                        path_to = os.path.join(
                                self.GAZEBO,
                                'launch',
                                raw_name_to + "." + ending
                        )
                        Converter.copy_file(path_from, path_to)

                # Copy pngs
                elif ending == "png":
                        path_from = os.path.join(
                                self.GAZEBO,
                                'randgen_imgs',
                                file_to_copy_from
                        )
                        path_to = os.path.join(
                                self.GAZEBO,
                                'randgen_imgs',
                                raw_name_to + "." + ending
                        )
                        Converter.copy_file(path_from, path_to)

                # Copy csvs
                elif ending == "csv":
                        path_from = os.path.join(
                                self.GAZEBO,
                                'tracks',
                                file_to_copy_from
                        )
                        path_to = os.path.join(
                                self.GAZEBO,
                                'tracks',
                                raw_name_to + "." + ending
                        )
                        Converter.copy_file(path_from, path_to)

                # If full stack copying, convert to all file formats
                if self.FULL_STACK_COPY_BUTTON.isChecked():
                        Converter.convert(
                                ending,
                                "ALL",
                                path_to,
                                params={"noise": self.get_noise_level()}
                        )

                # Update drop-downs with new files in directory
                self.load_track_and_images()
                self.tell_launchella("Copy Succeeded!")

        def update_copier(self):
                """Change label to show current selected file for the copier"""
                copy_head = self.RENAME_FILE_HEADER
                copy_head.setText("Copy: " + self.FILE_FOR_CONVERSION_BOX.currentText())

        def update_midpoints_box(self):
                """
                Controls the handling of the box that, when ticked,
                tells the to-csv converter to calculate cone midpoints.
                """
                # Toggle checkbox
                convert_to = self.CONVERT_TO_MENU.currentText()

        def update_converter_dropdown(self):
                """Keep the drop-down menus of ConversionTools in sync with the filesystem."""
                from_type = self.CONVERT_FROM_MENU.currentText()
                all_files = []

                if from_type == "launch":
                        # Get tracks from eufs_gazebo package
                        relevant_path = os.path.join(
                                self.GAZEBO,
                                'launch'
                        )
                        launch_files = [
                               f for f in listdir(relevant_path) if isfile(join(relevant_path, f))
                        ]

                        # Remove "blacklisted" files (ones that don't define tracks)
                        blacklist_filepath = os.path.join(
                                self.GAZEBO,
                                'launch/blacklist.txt'
                        )
                        blacklist_ = open(blacklist_filepath, "r")
                        blacklist = [f.strip() for f in blacklist_]
                        all_files = [f for f in launch_files if f not in blacklist]
                elif from_type == "png":
                        # Get images
                        relevant_path = os.path.join(
                                self.GAZEBO,
                                'randgen_imgs'
                        )
                        all_files = [
                                f for f in listdir(relevant_path)
                                if isfile(join(relevant_path, f))
                        ]
                elif from_type == "csv":
                        # Get csvs
                        relevant_path = os.path.join(
                                self.GAZEBO,
                                'tracks'
                        )
                        all_files = [
                                f for f in listdir(relevant_path)
                                if isfile(join(relevant_path, f)) and f[-3:] == "csv"
                        ]

                # Remove old files from selector
                the_selector = self.FILE_FOR_CONVERSION_BOX
                the_selector.clear()

                # Add files to selector
                for f in all_files:
                        the_selector.addItem(f)

                self.update_copier()

        def update_preset(self):
                """When preset is changed, change the sliders accordingly."""
                which = self.PRESET_SELECTOR.currentText()
                preset_data = Generator.get_preset(which)
                self.MIN_STRAIGHT = preset_data["MIN_STRAIGHT"]
                self.MAX_STRAIGHT = preset_data["MAX_STRAIGHT"]
                self.MIN_CTURN = preset_data["MIN_CONSTANT_TURN"]
                self.MAX_CTURN = preset_data["MAX_CONSTANT_TURN"]
                self.MIN_HAIRPIN = preset_data["MIN_HAIRPIN"] * 2
                self.MAX_HAIRPIN = preset_data["MAX_HAIRPIN"] * 2
                self.HAIRPIN_PAIRS = preset_data["MAX_HAIRPIN_PAIRS"]
                self.MAX_LENGTH = preset_data["MAX_LENGTH"]
                self.LAX_GENERATION = preset_data["LAX_GENERATION"]
                self.TRACK_WIDTH = preset_data["TRACK_WIDTH"]
                self.LAX_CHECKBOX.setChecked(self.LAX_GENERATION)

        def keep_track_of_preset_changes(self):
                """Hooks up the preset button with the preset_changed function."""
                self.PRESET_SELECTOR .currentTextChanged.connect(self.preset_changed)

        def preset_changed(self):
                """
                When preset is changed, set everything into motion that needs to happen.

                Updates dropdowns and sliders.
                """
                self.ignore_slider_changes = True
                self.update_preset()
                self.keep_params_up_to_date()
                self.keep_sliders_up_to_date()
                self.ignore_slider_changes = False

        def keep_track_of_slider_changes(self):
                """
                Hooks up all sliders with functions to respond to their changes.
                """
                self.MIN_STRAIGHT_SLIDER .valueChanged.connect(self.slider_changed)
                self.MAX_STRAIGHT_SLIDER .valueChanged.connect(self.slider_changed)
                self.MIN_CTURN_SLIDER    .valueChanged.connect(self.slider_changed)
                self.MAX_CTURN_SLIDER    .valueChanged.connect(self.slider_changed)
                self.MIN_HAIRPIN_SLIDER  .valueChanged.connect(self.slider_changed)
                self.MAX_HAIRPIN_SLIDER  .valueChanged.connect(self.slider_changed)
                self.HAIRPIN_PAIRS_SLIDER.valueChanged.connect(self.slider_changed)
                self.MAX_LENGTH_SLIDER   .valueChanged.connect(self.slider_changed)
                self.TRACK_WIDTH_SLIDER  .valueChanged.connect(self.slider_changed)

        def slider_changed(self):
                """When a slider is changed, update the parameters."""
                if self.ignore_slider_changes:
                    return
                self.keep_variables_up_to_date()
                self.keep_params_up_to_date()

        def keep_params_up_to_date(self):
                """This function keeps the labels next to the sliders up to date."""
                self.MIN_STRAIGHT_LABEL.setText("MIN_STRAIGHT: " + str(self.MIN_STRAIGHT))
                self.MAX_STRAIGHT_LABEL.setText("MAX_STRAIGHT: " + str(self.MAX_STRAIGHT))
                self.MIN_CTURN_LABEL.setText("MIN_CTURN: " + str(self.MIN_CTURN))
                self.MAX_CTURN_LABEL.setText("MAX_CTURN: " + str(self.MAX_CTURN))
                self.MIN_HAIRPIN_LABEL.setText("MIN_HAIRPIN: " + str((self.MIN_HAIRPIN / 2.0)))
                self.MAX_HAIRPIN_LABEL.setText("MAX_HAIRPIN: " + str((self.MAX_HAIRPIN / 2.0)))
                self.HAIRPIN_PAIRS_LABEL.setText("HAIRPIN_PAIRS: " + str(self.HAIRPIN_PAIRS))
                self.MAX_LENGTH_LABEL.setText("MAX_LENGTH: " + str(self.MAX_LENGTH))
                self.TRACK_WIDTH_LABEL.setText("TRACK_WIDTH: " + str(self.TRACK_WIDTH))

        def keep_sliders_up_to_date(self):
                """This function keeps the values of the sliders up to date."""
                self.set_slider_value("Param_MIN_STRAIGHT",  self.MIN_STRAIGHT)
                self.set_slider_value("Param_MAX_STRAIGHT",  self.MAX_STRAIGHT)
                self.set_slider_value("Param_MIN_CTURN",     self.MIN_CTURN)
                self.set_slider_value("Param_MAX_CTURN",     self.MAX_CTURN)
                self.set_slider_value("Param_MIN_HAIRPIN",   self.MIN_HAIRPIN)
                self.set_slider_value("Param_MAX_HAIRPIN",   self.MAX_HAIRPIN)
                self.set_slider_value("Param_HAIRPIN_PAIRS", self.HAIRPIN_PAIRS)
                self.set_slider_value("Param_MAX_LENGTH",    self.MAX_LENGTH)
                self.set_slider_value("Param_TRACK_WIDTH",   self.TRACK_WIDTH)

        def keep_variables_up_to_date(self):
                """This function keeps LauncherModule's variables up to date."""
                self.MIN_STRAIGHT = self.get_slider_value("Param_MIN_STRAIGHT")
                self.MAX_STRAIGHT = self.get_slider_value("Param_MAX_STRAIGHT")
                self.MIN_CTURN = self.get_slider_value("Param_MIN_CTURN")
                self.MAX_CTURN = self.get_slider_value("Param_MAX_CTURN")
                self.MIN_HAIRPIN = self.get_slider_value("Param_MIN_HAIRPIN")
                self.MAX_HAIRPIN = self.get_slider_value("Param_MAX_HAIRPIN")
                self.HAIRPIN_PAIRS = self.get_slider_value("Param_HAIRPIN_PAIRS")
                self.MAX_LENGTH = self.get_slider_value("Param_MAX_LENGTH")
                self.TRACK_WIDTH = self.get_slider_value("Param_TRACK_WIDTH")

        def set_slider_ranges(self):
                """
                This function specifies the bounds of the sliders.

                The following values are entirely arbitrary:
                                Straights are between 0 and 150
                                Turns are between 0 and 50
                                Hairpins are between 0 and 20,
                                     and have half-step rather than integer step
                                     (hence scaling by 2s)
                                Hairpin pairs are between 0 and 5
                                Max Length is between 200 and 2000

                The values were chosen because the sliders needed min and max values, and
                these values seemed to encompass the range of desired functionality.
                """
                min_straight = 0
                max_straight = 150
                min_turn = 0
                max_turn = 50
                min_hairpin = 0
                max_hairpin = 20
                min_hairpin_pairs = 0
                max_hairpin_pairs = 5
                min_max_length = 200
                max_max_length = 2000
                min_width = 2
                max_width = 10
                self.set_slider_data("Param_MIN_STRAIGHT", min_straight, max_straight)
                self.set_slider_data("Param_MAX_STRAIGHT", min_straight, max_straight)
                self.set_slider_data("Param_MIN_CTURN", min_turn, max_turn)
                self.set_slider_data("Param_MAX_CTURN", min_turn, max_turn)
                self.set_slider_data("Param_MIN_HAIRPIN", min_hairpin * 2, max_hairpin * 2)
                self.set_slider_data("Param_MAX_HAIRPIN", min_hairpin * 2, max_hairpin * 2)
                self.set_slider_data("Param_HAIRPIN_PAIRS", min_hairpin_pairs, max_hairpin_pairs)
                self.set_slider_data("Param_MAX_LENGTH", min_max_length, max_max_length)
                self.set_slider_data("Param_TRACK_WIDTH", min_width, max_width)

        def get_slider_value(self, slidername):
                """Returns the value of the specified slider."""
                slider = self._widget.findChild(QSlider, slidername)
                return slider.value()

        def set_slider_value(self, slidername, sliderval):
                """Sets the value of the specified slider."""
                slider = self._widget.findChild(QSlider, slidername)
                slider.setValue(sliderval)

        def set_slider_data(self, slidername, slidermin, slidermax):
                """Sets the minimum and maximum values of sliders."""
                slider = self._widget.findChild(QSlider, slidername)
                slider.setMinimum(slidermin)
                slider.setMaximum(slidermax)

        def generator_button_pressed(self):
                """Handles random track generation."""

                self.tell_launchella("Generating Track...")

                isLaxGenerator = self.LAX_CHECKBOX.isChecked()

                # Prepare and pass in all the parameters of the track
                # If track takes too long to generate, this section will
                # throw an error, to be handled after this try clause.
                component_data = Generator.get_preset(
                        self.PRESET_SELECTOR.currentText()
                )["COMPONENTS"]

                generator_values = {
                                    "MIN_STRAIGHT": self.MIN_STRAIGHT,
                                    "MAX_STRAIGHT": self.MAX_STRAIGHT,
                                    "MIN_CONSTANT_TURN": self.MIN_CTURN,
                                    "MAX_CONSTANT_TURN": self.MAX_CTURN,
                                    "MIN_HAIRPIN": self.MIN_HAIRPIN/2,
                                    "MAX_HAIRPIN": self.MAX_HAIRPIN/2,
                                    "MAX_HAIRPIN_PAIRS": self.HAIRPIN_PAIRS,
                                    "MAX_LENGTH": self.MAX_LENGTH,
                                    "LAX_GENERATION": isLaxGenerator,
                                    "TRACK_WIDTH": self.TRACK_WIDTH,
                                    "COMPONENTS": component_data
                }

                def failure_function():
                        self.tell_launchella("Track Gen Failed :(  Try different parameters?")

                with GeneratorContext(generator_values, failure_function):
                        xys, twidth, theight = Generator.generate()

                        # If track is generated successfully, turn it into a track image
                        # and display it to the user.
                        self.tell_launchella("Loading Image...")
                        im = Converter.convert(
                                "comps",
                                "png",
                                "rand",
                                params={
                                        "track data": (xys, twidth, theight)
                                }
                        )

                        # If full stack selected, convert into csv and launch as well
                        track_generator_full_stack = self.FULL_STACK_TRACK_GEN_BUTTON
                        if track_generator_full_stack.isChecked():
                                img_path = os.path.join(
                                        self.GAZEBO,
                                        'randgen_imgs/rand.png'
                                )
                                Converter.convert(
                                        "png",
                                        "ALL",
                                        img_path,
                                        params={"noise": self.get_noise_level()}
                                )

                        self.tell_launchella("Track Gen Complete!")

                        im.show()

                self.load_track_and_images()

        def track_from_image_button_pressed(self):
                """
                Converts .png to .launch by interfacing with ConversionTools,
                then launches said .launch.
                """

                self.tell_launchella("Preparing to launch image as a track... ")
                filename = self.IMAGE_SELECTOR.currentText()
                filename_full = os.path.join(
                        self.GAZEBO,
                        'randgen_imgs/'+filename
                )
                image_launcher_full_stack = self.FULL_STACK_IMAGE_BUTTON
                if image_launcher_full_stack.isChecked():
                        Converter.convert(
                                "png",
                                "ALL",
                                filename_full,
                                params={"noise": self.get_noise_level()}
                        )
                else:
                        Converter.convert(
                                "png",
                                "launch",
                                filename_full,
                                params={"noise": self.get_noise_level()}
                        )

                self.launch_file_override = filename[:-4] + ".launch"
                self.load_track_and_images()
                self.launch_button_pressed()

        def get_noise_level(self):
                """Returns the noise slider's noise level."""

                noise_level_widget = self.NOISE_SLIDER
                numerator = (1.0 * (noise_level_widget.value() - noise_level_widget.minimum()))
                denominator = (noise_level_widget.maximum() - noise_level_widget.minimum())
                return numerator/denominator

        def convert_button_pressed(self):
                """Handles interfacing with ConversionTools."""

                # Get info from launcher gui
                from_type = self.CONVERT_FROM_MENU.currentText()
                to_type = self.CONVERT_TO_MENU.currentText()
                filename = self.FILE_FOR_CONVERSION_BOX.currentText()
                self.tell_launchella("Conversion Button Pressed!  From: " + from_type +
                                     " To: " + to_type + " For: " + filename)
                midpoint_widget = self.MIDPOINT_CHECKBOX

                # Calculate correct full filepath for file to convert
                if from_type == "png":
                        filename = os.path.join(self.GAZEBO, 'randgen_imgs/'+filename)
                elif from_type == "launch":
                        filename = os.path.join(self.GAZEBO, 'launch/'+filename)
                elif from_type == "csv":
                        filename = os.path.join(self.GAZEBO, 'tracks/'+filename)

                # Calculate some parameters
                suffix = "_CT" if self.SUFFIX_CHECKBOX.isChecked() else ""
                use_midpoints = midpoint_widget.isVisible() and midpoint_widget.isChecked()

                # Convert it
                Converter.convert(
                        from_type,
                        to_type,
                        filename,
                        params={
                                "noise": self.get_noise_level(),
                                "midpoints": use_midpoints
                        },
                        conversion_suffix=suffix
                )
                self.load_track_and_images()
                self.tell_launchella("Conversion Succeeded!  From: " + from_type +
                                     " To: " + to_type + " For: " + filename)

        def launch_button_pressed(self):
                """Launches Gazebo."""

                if self.has_launched_ros:
                        # Don't let people press launch twice
                        return
                self.has_launched_ros = True

                self.tell_launchella("--------------------------")
                self.tell_launchella("\t\t\tLaunching Nodes...")

                # If we have set a specific file to run regardless of selected file,
                # which may happen when we launch from an image,
                # we make sure to launch the overriding track instead.
                track_to_launch = self.launch_file_override
                self.launch_file_override = None
                if not track_to_launch:
                        track_to_launch = self.TRACK_SELECTOR.currentText()

                # Get noise level
                self.tell_launchella("Launching " + track_to_launch)
                noise_level = self.get_noise_level()
                self.tell_launchella("With Noise Level: " + str(noise_level))

                # Get control information
                control_method = "controlMethod:=speed"
                if self.SPEED_RADIO.isChecked():
                        self.tell_launchella("With Speed Controls")
                        control_method = "controlMethod:=speed"
                elif self.TORQUE_RADIO.isChecked():
                        self.tell_launchella("With Torque Controls")
                        control_method = "controlMethod:=torque"

                # Get perception information
                perception_stack = ["launch_group:=no_perception"]
                if self.PERCEPTION_CHECKBOX.isChecked():
                        perception_stack = []  # is on

                # Check if we should launch the gazebo gui!
                gui_on = "gui:=" + (
                        "true" if self.GAZEBO_GUI_CHECKBOX.isChecked() else "false"
                )

                # How we launch the simulation changes depending on whether
                # we are using the eufs_sim package standalone, or working within
                # the eufs-master ecosystem.
                dir_to_check = os.path.dirname(os.path.dirname(os.path.dirname(self.GAZEBO)))
                if dir_to_check.split("/")[-1] == "eufs-master":
                        launch_location = os.path.join(
                                                dir_to_check,
                                                'launch',
                                                'simulation.launch'
                        )
                        self.popen_process = self.launch_node_with_args(
                                                launch_location,
                                                [
                                                        control_method,
                                                        gui_on,
                                                        "track:=" + track_to_launch.split(".")[0]
                                                ] + perception_stack
                                        )
                else:
                        self.popen_process = self.launch_node_with_args(
                                                os.path.join(
                                                        self.GAZEBO,
                                                        'launch',
                                                        track_to_launch
                                                ),
                                                [control_method, gui_on]
                                        )

                # Launch the visualisator if applicable.
                if self.VISUALISATOR_CHECKBOX.isChecked():
                        self.tell_launchella("And With LIDAR Data Visualisator.")
                        node_path = os.path.join(
                                rospkg.RosPack().get_path('eufs_description'),
                                "launch",
                                "visualisator.launch"
                        )
                        self.launch_node(node_path)

                self.tell_launchella("As I have fulfilled my purpose in guiding you " +
                                     "to launch a track, this launcher will no longer " +
                                     "react to input.")

                # Launch SBG Simulator Node
                self.launch_node_with_args(
                        os.path.join(
                                self.GAZEBO,
                                "launch",
                                "sbg_raw_data_simulator.launch"
                        ),
                        [
                                "_imu_hz:=200",
                                "_gps_hz:=5"
                        ]
                )

                # Hide launcher
                self._widget.setVisible(False)
                self._widget.window().showMinimized()

        def launch_node(self, filepath):
                """Wrapper for launch_node_with_args"""
                self.launch_node_with_args(filepath, [])

        def launch_node_with_args(self, filepath, args):
                """
                Launches ros node.

                If arguments are supplied, it has to use Popen
                rather than the default launch method.
                """
                if len(args) > 0:
                        process = Popen(["roslaunch", filepath] + args)
                        self.popens.append(process)
                        return process
                else:
                        launch = roslaunch.parent.ROSLaunchParent(self.uuid, [filepath])
                        launch.start()
                        self.launches.append(launch)
                        return launch

        def shutdown_plugin(self):
                """Unregister all publishers, kill all nodes."""
                self.tell_launchella("Shutdown Engaged...")

                # (Stop all processes)
                for p in self.processes:
                    p.stop()

                for l in self.launches:
                    l.shutdown()

                for p in self.popens:
                    p.kill()

                # Manual node killer (needs to be used on nodes opened by Popen):
                extra_nodes = rosnode.get_node_names()
                extra_nodes.remove("/eufs_launcher")
                extra_nodes.remove("/rosout")
                left_open = len(extra_nodes)
                if (left_open > 0 and self.DEBUG_SHUTDOWN):
                        rospy.logerr("Warning, after shutting down the launcher, " +
                                     "these nodes are still running: " + str(extra_nodes))

                nodes_to_kill = [
                                    "/cone_ground_truth",
                                    "/eufs/controller_spawner",
                                    "/gazebo",
                                    "/gazebo_gui",
                                    "/robot_state_publisher",
                                    "/ros_can_sim",
                                    "/twist_to_ackermannDrive",
                                    "/spawn_platform",
                                    "/eufs_sim_rqt",
                                ]
                for bad_node in extra_nodes:
                        if bad_node in nodes_to_kill:
                                Popen(["rosnode", "kill", bad_node])
                Popen(["killall", "-9", "gzserver"])
                time.sleep(0.25)
                extra_nodes = rosnode.get_node_names()
                extra_nodes.remove("/eufs_launcher")
                extra_nodes.remove("/rosout")
                if left_open > 0 and self.DEBUG_SHUTDOWN:
                        rospy.logerr("Pruned to: " + str(extra_nodes))
