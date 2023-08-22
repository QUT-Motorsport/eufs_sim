from os import listdir, mkdir
from os.path import exists, isfile, join
from shutil import copyfile

from ament_index_python.packages import get_package_share_directory
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import (QApplication, QComboBox, QLabel,
                                         QLineEdit, QPushButton, QWidget)
from qt_gui.plugin import Plugin

from eufs_tracks.converter_tool import Converter


class EUFSConverterGUI(Plugin):
    def __init__(self, context):
        super(EUFSConverterGUI, self).__init__(context)

        # Give QObjects reasonable names
        self.setObjectName("EUFSConverterGUI")
        self.node = context.node
        self.logger = self.node.get_logger()

        # Create QWidget
        self._widget = QWidget()
        self._widget.setObjectName("EUFSConverterUI")

        # Store gazebo's path as it is used quite a lot:
        self.TRACKS = get_package_share_directory("eufs_tracks")

        # Extend the widget with all attributes and children from UI file
        # UI file which should be in the "ui" folder of this package
        main_ui_file = join(self.TRACKS, "ui", "conversion_tool.ui")
        loadUi(main_ui_file, self._widget)

        # Show _widget.windowTitle on left-top of each plugin (when
        # it's set in _widget). This is useful when you open multiple
        # plugins at once. Also if you open multiple instances of your
        # plugin at once, these lines add number to make it easy to
        # tell from pane to pane.
        if context.serial_number() > 1:
            the_title = self._widget.windowTitle() + (" (%d)" % context.serial_number())
            self._widget.setWindowTitle(the_title)

        # Give widget components permanent names
        self.CONVERT_BUTTON = self._widget.findChild(QPushButton, "ConvertButton")
        self.RENAME_BUTTON = self._widget.findChild(QPushButton, "RenameButton")
        self.CONVERT_FROM_MENU = self._widget.findChild(QComboBox, "ConvertFrom")
        self.CONVERT_TO_MENU = self._widget.findChild(QComboBox, "ConvertTo")
        self.FILE_FOR_CONVERSION_BOX = self._widget.findChild(
            QComboBox, "FileForConversion"
        )

        # Hook up buttons to onclick functions
        self.CONVERT_BUTTON.clicked.connect(self.convert_button_pressed)

        # Setup Conversion Tools dropdowns
        for f in ["csv"]:
            self.CONVERT_FROM_MENU.addItem(f)
        for f in ["world"]:
            self.CONVERT_TO_MENU.addItem(f)

        self.update_converter_dropdown()
        self.CONVERT_FROM_MENU.currentTextChanged.connect(
            self.update_converter_dropdown
        )

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
            if hasattr(widget, "geometry"):
                geom = widget.geometry()
                new_width = (
                    geom.width() * (scaler_multiplier)
                    if not isinstance(widget, QLabel)
                    else geom.width() * (scaler_multiplier) + 200
                )
                widget.setGeometry(
                    int(geom.x() * scaler_multiplier),
                    int(geom.y() * scaler_multiplier),
                    int(new_width),
                    int(geom.height() * (scaler_multiplier)),
                )

    ###################
    #    Converter    #
    ###################

    def convert_button_pressed(self):
        """Handles interfacing with ConversionTools."""

        # Get info from track generator gui
        from_type = self.CONVERT_FROM_MENU.currentText()
        to_type = self.CONVERT_TO_MENU.currentText()
        filename = self.FILE_FOR_CONVERSION_BOX.currentText()
        self.logger.info(
            "Converting from: " + from_type + " to: " + to_type + " for: " + filename
        )

        # Calculate correct full filepath for file to convert
        if from_type == "csv":
            filename = join(self.TRACKS, "csv/" + filename)

        # Convert it
        Converter.convert(from_type, to_type, filename)
        self.logger.info(
            "Converted from: " + from_type + " to: " + to_type + " for: " + filename
        )

    def update_converter_dropdown(self):
        """Keep the drop-down menus of ConversionTools in sync with the filesystem."""
        from_type = self.CONVERT_FROM_MENU.currentText()
        all_files = []

        if from_type == "csv":
            relevant_path = join(self.TRACKS, "csv")
            all_files = [
                f
                for f in listdir(relevant_path)
                if isfile(join(relevant_path, f)) and f[-3:] == "csv"
            ]

        # Remove old files from selector
        the_selector = self.FILE_FOR_CONVERSION_BOX
        the_selector.clear()

        # Add files to selector
        for f in all_files:
            the_selector.addItem(f)
