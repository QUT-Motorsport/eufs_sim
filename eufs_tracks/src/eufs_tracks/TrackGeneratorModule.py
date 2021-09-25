from os import listdir, mkdir
from os.path import isfile, join, exists

from ament_index_python.packages import get_package_share_directory

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import (QWidget, QComboBox, QPushButton, QSlider, QRadioButton, QCheckBox, QMainWindow,
                                         QLabel, QLineEdit, QApplication)

from .TrackGenerator import TrackGenerator as Generator
from .TrackGenerator import GeneratorContext

from .ConversionTools import ConversionTools as Converter


class EUFSTrackGenerator(Plugin):

    def __init__(self, context):
        """
                This function handles loading the track generator GUI
                and all the setting-up of the values and buttons displayed.
                """
        super(EUFSTrackGenerator, self).__init__(context)

        # Give QObjects reasonable names
        self.setObjectName('EUFSTrackGenerator')

        self.node = context.node

        self.logger = self.node.get_logger()

        # Create QWidget
        self._widget = QWidget()

        # Get path to UI file which should be in the "resource" folder of this package
        self.main_ui_file = join(get_package_share_directory('eufs_tracks'),
                                 'resource',
                                 'track_generator.ui',
                                 )

        # Extend the widget with all attributes and children from UI file
        loadUi(self.main_ui_file, self._widget)

        # Give QObjects reasonable names
        self._widget.setObjectName('EUFSTrackGeneratorUI')

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
        self.TRACKS = get_package_share_directory('eufs_tracks')

        # Give widget components permanent names
        self.PRESET_SELECTOR = self._widget.findChild(QComboBox, "WhichPreset")
        self.GENERATOR_BUTTON = self._widget.findChild(QPushButton, "GenerateButton")
        self.NOISE_SLIDER = self._widget.findChild(QSlider, "Noisiness")

        self.CONVERT_BUTTON = self._widget.findChild(QPushButton, "ConvertButton")
        self.RENAME_BUTTON = self._widget.findChild(QPushButton, "RenameButton")
        self.SKETCHER_BUTTON = self._widget.findChild(QPushButton, "SketcherButton")
        self.LAX_CHECKBOX = self._widget.findChild(QCheckBox, "LaxCheckBox")
        self.CONVERT_FROM_MENU = self._widget.findChild(QComboBox, "ConvertFrom")
        self.CONVERT_TO_MENU = self._widget.findChild(QComboBox, "ConvertTo")
        self.MIDPOINT_CHECKBOX = self._widget.findChild(QCheckBox, "MidpointBox")
        self.USER_FEEDBACK_LABEL = self._widget.findChild(QLabel, "UserFeedbackLabel")

        self.SUFFIX_CHECKBOX = self._widget.findChild(QCheckBox, "SuffixBox")
        self.RENAME_FILE_TEXTBOX = self._widget.findChild(QLineEdit, "RenameFileTextbox")
        self.RENAME_FILE_HEADER = self._widget.findChild(QLabel, "RenameFileHeader")

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
        self.GENERATOR_BUTTON.clicked.connect(self.generator_button_pressed)
        self.CONVERT_BUTTON.clicked.connect(self.convert_button_pressed)
        self.RENAME_BUTTON.clicked.connect(self.copy_button_pressed)

        # Add widget to the user interface
        context.add_widget(self._widget)

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

        # Copier full stack checkbox
        copier_full_stack = self.FULL_STACK_COPY_BUTTON
        copier_full_stack.setChecked(True)

        # Change label to show current selected file for the copier
        self.update_copier()

        self.DEBUG_SHUTDOWN = False

        # TEMPORARY: Disable Noise Slider as it's broken
        self.NOISE_SLIDER.setVisible(False)
        self._widget.findChild(QLabel, "NoiseLabel").setVisible(False)
        self._widget.findChild(QLabel, 'ObjectNoiseToolTip').setVisible(False)

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

    def copy_button_pressed(self):
        """When copy button is pressed, launch ConversionTools"""

        self.logger.info("Copying...")

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
            model_path = join(
                get_package_share_directory('eufs_tracks'),
                'models',
                raw_name_to
            )
            if not exists(model_path):
                mkdir(model_path)

            # Copy sdf files
            path_from = join(
                get_package_share_directory('eufs_tracks'),
                'models',
                raw_name_from,
                "model.sdf"
            )
            path_to = join(
                get_package_share_directory('eufs_tracks'),
                'models',
                raw_name_to,
                "model.sdf"
            )
            Converter.copy_file(path_from, path_to)

            # Copy config files
            path_from = join(
                get_package_share_directory('eufs_tracks'),
                'models',
                raw_name_from,
                "model.config"
            )
            path_to = join(
                get_package_share_directory('eufs_tracks'),
                'models',
                raw_name_to,
                "model.config"
            )
            Converter.copy_file(path_from, path_to)

            # Copy launch files
            path_from = join(
                self.TRACKS,
                'launch',
                file_to_copy_from
            )
            path_to = join(
                self.TRACKS,
                'launch',
                raw_name_to + "." + ending
            )
            Converter.copy_file(path_from, path_to)

        # Copy pngs
        elif ending == "png":
            path_from = join(
                self.TRACKS,
                'image',
                file_to_copy_from
            )
            path_to = join(
                self.TRACKS,
                'image',
                raw_name_to + "." + ending
            )
            Converter.copy_file(path_from, path_to)

        # Copy csvs
        elif ending == "csv":
            path_from = join(
                self.TRACKS,
                'csv',
                file_to_copy_from
            )
            path_to = join(
                self.TRACKS,
                'csv',
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

    def get_noise_level(self):
        """Returns the object noise slider's noise level."""

        noise_level_widget = self.NOISE_SLIDER
        numerator = (1.0 * (noise_level_widget.value() - noise_level_widget.minimum()))
        denominator = (noise_level_widget.maximum() - noise_level_widget.minimum())
        return numerator / denominator

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
            # Get tracks from eufs_tracks package
            relevant_path = join(
                self.TRACKS,
                'launch'
            )
            launch_files = [
                f for f in listdir(relevant_path) if isfile(join(relevant_path, f))
            ]

            # Remove "blacklisted" files (ones that don't define tracks)
            blacklist_filepath = join(
                self.TRACKS,
                'launch/blacklist.txt'
            )
            blacklist_ = open(blacklist_filepath, "r")
            blacklist = [f.strip() for f in blacklist_]
            all_files = [f for f in launch_files if f not in blacklist]
        elif from_type == "png":
            # Get images
            relevant_path = join(
                self.TRACKS,
                'image'
            )
            all_files = [
                f for f in listdir(relevant_path)
                if isfile(join(relevant_path, f))
            ]
        elif from_type == "csv":
            # Get csvs
            relevant_path = join(
                self.TRACKS,
                'csv'
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
        self.TRACK_WIDTH = self.deconvert_track_width(preset_data["TRACK_WIDTH"])
        self.LAX_CHECKBOX.setChecked(self.LAX_GENERATION)

    def keep_track_of_preset_changes(self):
        """Hooks up the preset button with the preset_changed function."""
        self.PRESET_SELECTOR.currentTextChanged.connect(self.preset_changed)

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
        self.MIN_STRAIGHT_SLIDER.valueChanged.connect(self.slider_changed)
        self.MAX_STRAIGHT_SLIDER.valueChanged.connect(self.slider_changed)
        self.MIN_CTURN_SLIDER.valueChanged.connect(self.slider_changed)
        self.MAX_CTURN_SLIDER.valueChanged.connect(self.slider_changed)
        self.MIN_HAIRPIN_SLIDER.valueChanged.connect(self.slider_changed)
        self.MAX_HAIRPIN_SLIDER.valueChanged.connect(self.slider_changed)
        self.HAIRPIN_PAIRS_SLIDER.valueChanged.connect(self.slider_changed)
        self.MAX_LENGTH_SLIDER.valueChanged.connect(self.slider_changed)
        self.TRACK_WIDTH_SLIDER.valueChanged.connect(self.slider_changed)

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
        self.TRACK_WIDTH_LABEL.setText(
            "TRACK_WIDTH: " + str(self.convert_track_width(self.TRACK_WIDTH))
        )

    def keep_sliders_up_to_date(self):
        """This function keeps the values of the sliders up to date."""
        self.set_slider_value("Param_MIN_STRAIGHT", self.MIN_STRAIGHT)
        self.set_slider_value("Param_MAX_STRAIGHT", self.MAX_STRAIGHT)
        self.set_slider_value("Param_MIN_CTURN", self.MIN_CTURN)
        self.set_slider_value("Param_MAX_CTURN", self.MAX_CTURN)
        self.set_slider_value("Param_MIN_HAIRPIN", self.MIN_HAIRPIN)
        self.set_slider_value("Param_MAX_HAIRPIN", self.MAX_HAIRPIN)
        self.set_slider_value("Param_HAIRPIN_PAIRS", self.HAIRPIN_PAIRS)
        self.set_slider_value("Param_MAX_LENGTH", self.MAX_LENGTH)
        self.set_slider_value("Param_TRACK_WIDTH", self.TRACK_WIDTH)

    def keep_variables_up_to_date(self):
        """This function keeps TrackGeneratorModule's variables up to date."""
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
        min_width = 0
        max_width = 5
        self.set_slider_data("Param_MIN_STRAIGHT", min_straight, max_straight)
        self.set_slider_data("Param_MAX_STRAIGHT", min_straight, max_straight)
        self.set_slider_data("Param_MIN_CTURN", min_turn, max_turn)
        self.set_slider_data("Param_MAX_CTURN", min_turn, max_turn)
        self.set_slider_data("Param_MIN_HAIRPIN", min_hairpin * 2, max_hairpin * 2)
        self.set_slider_data("Param_MAX_HAIRPIN", min_hairpin * 2, max_hairpin * 2)
        self.set_slider_data("Param_HAIRPIN_PAIRS", min_hairpin_pairs, max_hairpin_pairs)
        self.set_slider_data("Param_MAX_LENGTH", min_max_length, max_max_length)
        self.set_slider_data("Param_TRACK_WIDTH", min_width, max_width)

    def convert_track_width(self, w):
        """
                Track width officially between 0 and 5,
                but it's really between 2.5 and 5
                """
        return w / 2.0 + 2.5

    def deconvert_track_width(self, w):
        """
                Track width officially between 0 and 5,
                but it's really between 2.5 and 5
                This is inverse of convert_track_width
                """
        return (w - 2.5) * 2.0

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

        self.logger.info("Generating Track...")

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
            "MIN_HAIRPIN": self.MIN_HAIRPIN / 2,
            "MAX_HAIRPIN": self.MAX_HAIRPIN / 2,
            "MAX_HAIRPIN_PAIRS": self.HAIRPIN_PAIRS,
            "MAX_LENGTH": self.MAX_LENGTH,
            "LAX_GENERATION": isLaxGenerator,
            "TRACK_WIDTH": self.convert_track_width(self.TRACK_WIDTH),
            "COMPONENTS": component_data
        }

        def failure_function():
            self.logger.error("Track Gen Failed :(  Try different parameters?")

        with GeneratorContext(generator_values, failure_function):
            xys, twidth, theight = Generator.generate()

            # If track is generated successfully, turn it into a track image
            # and display it to the user.
            self.logger.info("Loading Image...")
            im = Converter.convert(
                "comps",
                "csv",
                "rand",
                params={
                    "track data": (xys, twidth, theight)
                }
            )

            # If full stack selected, convert into csv and launch as well
            track_generator_full_stack = self.FULL_STACK_TRACK_GEN_BUTTON
            if track_generator_full_stack.isChecked():
                csv_path = join(
                    self.TRACKS,
                    'csv/rand.csv'
                )
                Converter.convert(
                    "csv",
                    "ALL",
                    csv_path,
                    params={"noise": self.get_noise_level()}
                )

            self.logger.info("Track Gen Complete!")

            im.show()

    def convert_button_pressed(self):
        """Handles interfacing with ConversionTools."""

        # Get info from track generator gui
        from_type = self.CONVERT_FROM_MENU.currentText()
        to_type = self.CONVERT_TO_MENU.currentText()
        filename = self.FILE_FOR_CONVERSION_BOX.currentText()
        self.logger.info("Conversion Button Pressed!  From: " + from_type +
                         " To: " + to_type + " For: " + filename)
        midpoint_widget = self.MIDPOINT_CHECKBOX

        # Calculate correct full filepath for file to convert
        if from_type == "png":
            filename = join(self.TRACKS, 'image/' + filename)
        elif from_type == "launch":
            filename = join(self.TRACKS, 'launch/' + filename)
        elif from_type == "csv":
            filename = join(self.TRACKS, 'csv/' + filename)

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
        self.logger.info("Conversion Succeeded!  From: " + from_type +
                         " To: " + to_type + " For: " + filename)
