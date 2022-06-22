from shutil import copyfile
from os import listdir, mkdir
from os.path import isfile, join, exists

from ament_index_python.packages import get_package_share_directory

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget, QComboBox, QPushButton
from python_qt_binding.QtWidgets import QLabel, QLineEdit, QApplication

from eufs_tracks.converter_tool import Converter


class EUFSConverterGUI(Plugin):
    def __init__(self, context):
        super(EUFSConverterGUI, self).__init__(context)

        # Give QObjects reasonable names
        self.setObjectName('EUFSConverterGUI')
        self.node = context.node
        self.logger = self.node.get_logger()

        # Create QWidget
        self._widget = QWidget()
        self._widget.setObjectName('EUFSConverterUI')

        # Store gazebo's path as it is used quite a lot:
        self.TRACKS = get_package_share_directory('eufs_tracks')

        # Extend the widget with all attributes and children from UI file
        # UI file which should be in the "resource" folder of this package
        main_ui_file = join(self.TRACKS, 'resource', 'conversion_tool.ui')
        loadUi(main_ui_file, self._widget)

        # Show _widget.windowTitle on left-top of each plugin (when
        # it's set in _widget). This is useful when you open multiple
        # plugins at once. Also if you open multiple instances of your
        # plugin at once, these lines add number to make it easy to
        # tell from pane to pane.
        if context.serial_number() > 1:
            the_title = (self._widget.windowTitle() + (' (%d)' % context.serial_number()))
            self._widget.setWindowTitle(the_title)

        # Give widget components permanent names
        self.CONVERT_BUTTON = self._widget.findChild(QPushButton, "ConvertButton")
        self.RENAME_BUTTON = self._widget.findChild(QPushButton, "RenameButton")
        self.CONVERT_FROM_MENU = self._widget.findChild(QComboBox, "ConvertFrom")
        self.CONVERT_TO_MENU = self._widget.findChild(QComboBox, "ConvertTo")
        self.RENAME_FILE_TEXTBOX = self._widget.findChild(QLineEdit, "RenameFileTextbox")
        self.RENAME_FILE_HEADER = self._widget.findChild(QLabel, "RenameFileHeader")
        self.FILE_FOR_CONVERSION_BOX = self._widget.findChild(QComboBox, "FileForConversion")

        # Hook up buttons to onclick functions
        self.CONVERT_BUTTON.clicked.connect(self.convert_button_pressed)
        self.RENAME_BUTTON.clicked.connect(self.copy_button_pressed)

        # Setup Conversion Tools dropdowns
        for f in ["launch", "csv"]:
            self.CONVERT_FROM_MENU.addItem(f)
        for f in ["csv", "launch"]:
            self.CONVERT_TO_MENU.addItem(f)

        self.update_converter_dropdown()
        self.CONVERT_FROM_MENU.currentTextChanged.connect(self.update_converter_dropdown)
        self.FILE_FOR_CONVERSION_BOX.currentTextChanged.connect(self.update_copier)

        # Change label to show current selected file for the copier
        self.update_copier()

        # Fix scaling issue
        self.fix_scaling()

        # Add widget to the user interface
        context.add_widget(self._widget)

    def fix_scaling(self):
        # Looping over all widgest to fix scaling issue via manual scaling
        # Scaling done via magically comparing the width to the 'default'
        # 1700 pixels
        rec = QApplication.desktop().screenGeometry()
        scaler_multiplier = rec.width() / 1700.0
        for widget in self._widget.children():
            if hasattr(widget, 'geometry'):
                geom = widget.geometry()
                new_width = (
                    geom.width() * (scaler_multiplier) if not isinstance(
                        widget, QLabel)
                    else geom.width() * (scaler_multiplier) + 200
                )
                widget.setGeometry(
                    geom.x() * scaler_multiplier,
                    geom.y() * scaler_multiplier,
                    new_width,
                    geom.height() * (scaler_multiplier)
                )

    ################
    #    Copier    #
    ################

    def copy_button_pressed(self):
        """When copy button is pressed, launch ConversionTools"""

        # Copy the current file
        file_to_copy_to = self.RENAME_FILE_TEXTBOX.text()
        file_to_copy_from = self.FILE_FOR_CONVERSION_BOX.currentText()
        raw_name_to = file_to_copy_to.split(".")[0]
        raw_name_from = file_to_copy_from.split(".")[0]
        ending = file_to_copy_from.split(".")[-1]

        # Don't let them create null-named files
        if len(file_to_copy_to) == 0:
            self.logger.warn("Cannot create copy with no file name.")
            return

        self.logger.info("Copying...")

        # For launch files, we also need to move around the model folders
        if ending == "launch":
            if not exists((model_path := join(self.TRACKS, 'models', raw_name_to))):
                mkdir(model_path)

            # Copy sdf files
            path_from = join(self.TRACKS, 'models', raw_name_from, "model.sdf")
            path_to = join(self.TRACKS, 'models', raw_name_to, "model.sdf")
            copyfile(path_from, path_to)

            # Copy config files
            path_from = join(self.TRACKS, 'models', raw_name_from, "model.config")
            path_to = join(self.TRACKS, 'models', raw_name_to, "model.config")
            copyfile(path_from, path_to)

            # Copy launch files
            path_from = join(self.TRACKS, 'launch', file_to_copy_from)
            path_to = join(self.TRACKS, 'launch', raw_name_to + "." + ending)
            copyfile(path_from, path_to)

        elif ending == "csv":
            path_from = join(self.TRACKS, 'csv', file_to_copy_from)
            path_to = join(self.TRACKS, 'csv', raw_name_to + "." + ending)
            copyfile(path_from, path_to)

        self.logger.info("Copy created Successfully!")

    def update_copier(self):
        """Change label to show current selected file for the copier"""
        copy_head = self.RENAME_FILE_HEADER
        copy_head.setText("Copy: " + self.FILE_FOR_CONVERSION_BOX.currentText())

    ###################
    #    Converter    #
    ###################

    def convert_button_pressed(self):
        """Handles interfacing with ConversionTools."""

        # Get info from track generator gui
        from_type = self.CONVERT_FROM_MENU.currentText()
        to_type = self.CONVERT_TO_MENU.currentText()
        filename = self.FILE_FOR_CONVERSION_BOX.currentText()
        self.logger.info("Converting from: " + from_type + " to: " + to_type + " for: " + filename)

        # Calculate correct full filepath for file to convert
        if from_type == "launch":
            filename = join(self.TRACKS, 'launch/' + filename)
        elif from_type == "csv":
            filename = join(self.TRACKS, 'csv/' + filename)

        # Convert it
        Converter.convert(from_type, to_type, filename)
        self.logger.info("Converted from: " + from_type + " to: " + to_type + " for: " + filename)

    def update_converter_dropdown(self):
        """Keep the drop-down menus of ConversionTools in sync with the filesystem."""
        from_type = self.CONVERT_FROM_MENU.currentText()
        all_files = []

        if from_type == "launch":
            # Get tracks from eufs_tracks package
            relevant_path = join(self.TRACKS, 'launch')
            launch_files = [f for f in listdir(relevant_path) if isfile(join(relevant_path, f))]

            # Remove "blacklisted" files (ones that don't define tracks)
            blacklist_filepath = join(self.TRACKS, 'launch/blacklist.txt')
            blacklist_ = open(blacklist_filepath, "r")
            blacklist = [f.strip() for f in blacklist_]
            all_files = [f for f in launch_files if f not in blacklist]
        elif from_type == "csv":
            relevant_path = join(self.TRACKS, 'csv')
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
