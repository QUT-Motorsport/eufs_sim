import random
import math

from qt_gui.plugin import Plugin
from python_qt_binding.QtCore import QPointF
from python_qt_binding.QtWidgets import QWidget, QVBoxLayout, QSpinBox, QDoubleSpinBox
from python_qt_binding.QtWidgets import QGroupBox, QFormLayout, QPushButton, QSizePolicy
from python_qt_binding.QtWidgets import QHBoxLayout, QLabel, QFileDialog, QSplitter
from python_qt_binding.QtGui import QBrush, QPainter, QPen, QColor

from eufs_tracks.track_generator import TrackGenerator


# ranges include both start and end values
constant_ranges = {
    'seed': {'min': 0, 'max': 10000},
    'min_corner_radius': {'min': 2, 'max': 50},
    'starting_straight_length': {'min': 0, 'max': 12},
    'cone_spacing_bias': {'min': 0, 'max': 2},

    'length': {'max': 10000},
    'margin': {'min': 0},
    'track_width': {'min': 0},
    'min_cone_spacing': {'min': 0.1},
    'max_cone_spacing': {'max': 10},
    'starting_cone_spacing': {'min': 0}
}

settings = {
    'seed': random.randint(constant_ranges['seed']['min'], constant_ranges['seed']['max']),
    'min_corner_radius': 3,
    'length': 500,
    'margin': 0,
    'starting_straight_length': 6,
    'min_cone_spacing': 3 * math.pi / 16,
    'max_cone_spacing': 5,
    'track_width': 3,
    'cone_spacing_bias': 0.5,
    'starting_cone_spacing': 0.5
}

ranges = {
    'min_corner_radius': {
        'length': {'min': 2 * math.pi},
        'track_width': {'max': "2*(settings['min_corner_radius'] - settings['margin'])"},
        'margin': {'max': "settings['min_corner_radius'] - settings['track_width']/2"},
    },
    'margin': {
        'track_width': {'max': "2*(settings['min_corner_radius'] - settings['margin'] - 0.01)"}
    },
    'track_width': {
        'margin': {'max': "settings['min_corner_radius'] - settings['track_width']/2 - 0.01"},
    },
    'min_cone_spacing': {
        'max_cone_spacing': {'min': 1}
    },
    'max_cone_spacing': {
        'starting_cone_spacing': {'max': 0.5},
        'min_cone_spacing': {'max': 1}
    }
}


def log_scaling(control, scaling):
    control.setSingleStep(control.value() * scaling)

    def adjust_step_size(value):
        control.setSingleStep(value * scaling)
    control.valueChanged.connect(adjust_step_size)
    return control


class TrackControls(QWidget):
    """docstring for TrackControls."""

    def __init__(self):
        super(TrackControls, self).__init__()
        self.setMinimumSize(100, 480)

        layout = QVBoxLayout(self)

        controls = {
            'seed': QSpinBox(),
            'min_corner_radius': log_scaling(QDoubleSpinBox(), 0.05),
            'length': log_scaling(QDoubleSpinBox(), 0.05),
            'margin': QDoubleSpinBox(),
            'starting_straight_length': QDoubleSpinBox(),
            'min_cone_spacing': log_scaling(QDoubleSpinBox(), 0.1),
            'max_cone_spacing': log_scaling(QDoubleSpinBox(), 0.1),
            'track_width': QDoubleSpinBox(),
            'cone_spacing_bias': QDoubleSpinBox(),
            'starting_cone_spacing': QDoubleSpinBox()
        }
        controls['starting_cone_spacing'].setSingleStep(0.1)
        controls['min_cone_spacing'].setSingleStep(0.2)
        controls['cone_spacing_bias'].setSingleStep(0.1)

        for ctrl in constant_ranges:
            if 'min' in constant_ranges[ctrl]:
                controls[ctrl].setMinimum(constant_ranges[ctrl]['min'])
            if 'max' in constant_ranges[ctrl]:
                controls[ctrl].setMaximum(constant_ranges[ctrl]['max'])

        def update_dependant_ranges(src):
            for dst in ranges[src]:
                if 'min' in ranges[src][dst]:
                    new_min = eval(ranges[src][dst]['min']) if isinstance(
                        ranges[src][dst]['min'], str) else settings[src] * ranges[src][dst]['min']
                    controls[dst].setMinimum(new_min)
                if 'max' in ranges[src][dst]:
                    new_max = eval(ranges[src][dst]['max']) if isinstance(
                        ranges[src][dst]['max'], str) else settings[src] * ranges[src][dst]['max']
                    controls[dst].setMaximum(new_max)

        for src in ranges:
            update_dependant_ranges(src)

        def on_value_changed(ctrl_name):
            def callback(value):
                settings[ctrl_name] = value
                if ctrl_name in ranges:
                    update_dependant_ranges(ctrl_name)
                self.parentWidget().redraw_track()
            return callback

        for ctrl in controls:
            controls[ctrl].setValue(settings[ctrl])
            controls[ctrl].valueChanged.connect(on_value_changed(ctrl))

        generation_group = QGroupBox()
        generation_group.setTitle("Track Generation")

        group = QFormLayout(generation_group)
        randomize_seed_btn = QPushButton("Randomize")
        randomize_seed_btn.setSizePolicy(QSizePolicy.Fixed, QSizePolicy.Fixed)

        def randomize_seed():
            settings['seed'] = random.randint(
                constant_ranges['seed']['min'], constant_ranges['seed']['max'])
            controls['seed'].setValue(settings['seed'])
            self.parentWidget().redraw_track()

        randomize_seed_btn.clicked.connect(randomize_seed)
        g = QWidget()
        seed_layout = QHBoxLayout(g)
        seed_layout.setContentsMargins(0, 0, 0, 0)
        seed_layout.addWidget(controls['seed'])
        seed_layout.addWidget(randomize_seed_btn)
        group.addRow(QLabel("Seed"), g)
        group.addRow(QLabel("Length"), controls['length'])
        group.addRow(QLabel("Min Turn Radius"), controls['min_corner_radius'])

        group.addRow(QLabel("Margin"), controls['margin'])

        cone_placement_group = QGroupBox()
        cone_placement_group.setTitle("Cone Placement")
        group = QFormLayout(cone_placement_group)

        group.addRow(QLabel("Starting Straight  Length"), controls['starting_straight_length'])

        group.addRow(QLabel("Starting Cone Spacing"), controls['starting_cone_spacing'])

        group.addRow(QWidget(), QWidget())

        group.addRow(QLabel("Cone Spacing Min"), controls['min_cone_spacing'])
        group.addRow(QLabel("Cone Spacing Max"), controls['max_cone_spacing'])

        group.addRow(QLabel("Cone Spacing Bias"), controls['cone_spacing_bias'])

        group.addRow(QWidget(), QWidget())

        group.addRow(QLabel("Track Width"), controls['track_width'])

        save_btn = QPushButton("Save")

        def save_track():
            filename = QFileDialog.getSaveFileName(self, "Save File", "track.csv", "CSV (*.csv)")[0]
            TrackGenerator.write_to_csv(filename, *TrackGenerator(settings)(), overwrite=True)
        save_btn.clicked.connect(save_track)

        layout.addWidget(generation_group)
        layout.addWidget(cone_placement_group)
        layout.addWidget(save_btn)


class TrackDisplay(QWidget):
    left_cone_fill = QBrush(QColor(49, 49, 226))
    right_cone_fill = QBrush(QColor(226, 220, 49))
    start_cone_fill = QBrush(QColor("#e28a31"))

    def __init__(self):
        super(TrackDisplay, self).__init__()
        self.resize(200, 200)
        self.setMinimumSize(480, 480)
        self.start_cones, self.left_cones, self.right_cones = TrackGenerator(settings)()

    def regenerate_path(self):
        self.start_cones, self.left_cones, self.right_cones = TrackGenerator(settings)()
        self.repaint(0, 0, -1, -1)

    def paintEvent(self, e):

        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        radius = 4
        stroke_width = 1
        margin = 0.1

        max_x = max(max(self.left_cones.real), max(self.right_cones.real),
                    max(self.start_cones.real)) + stroke_width / 2 + radius
        min_x = min(min(self.left_cones.real), min(self.right_cones.real),
                    min(self.start_cones.real)) - stroke_width / 2 - radius
        mid_x = (max_x + min_x) / 2

        max_y = max(max(self.left_cones.imag), max(self.right_cones.imag),
                    max(self.start_cones.imag)) + stroke_width / 2 + radius
        min_y = min(min(self.left_cones.imag), min(self.right_cones.imag),
                    min(self.start_cones.imag)) - stroke_width / 2 - radius
        mid_y = (max_y + min_y) / 2

        max_scale_x = (1 - 2 * margin) * self.width() / (max_x - min_x)
        max_scale_y = (1 - 2 * margin) * self.height() / (max_y - min_y)
        scale = min(max_scale_x, max_scale_y)

        painter.translate(self.width() / 2 - mid_x * scale, self.height() / 2 - mid_y * scale)

        # left cones
        painter.setPen(QPen(QColor(21, 21, 21), stroke_width))

        painter.setBrush(self.left_cone_fill)
        for cone in self.left_cones:
            painter.drawEllipse(scale * QPointF(cone.real, cone.imag), radius, radius)
        # right cones
        painter.setBrush(self.right_cone_fill)
        for cone in self.right_cones:
            painter.drawEllipse(scale * QPointF(cone.real, cone.imag), radius, radius)
        # starting cones
        painter.setBrush(self.start_cone_fill)
        for cone in self.start_cones:
            painter.drawEllipse(scale * QPointF(cone.real, cone.imag), radius, radius)

        painter.end()


class MainWindow(QSplitter):

    def __init__(self):
        super(MainWindow, self).__init__()
        self.setWindowTitle("EUFS Track Generator")
        self.setMinimumSize(720, 480)

        self.track_display = TrackDisplay()

        layout = QHBoxLayout(self)
        layout.addWidget(self.track_display)
        layout.addWidget(TrackControls())

    def redraw_track(self):
        self.track_display.regenerate_path()


class EUFSTracksGUI(Plugin):
    def __init__(self, context):
        super(EUFSTracksGUI, self).__init__(context)

        self.setObjectName('EUFSTracksGUI')
        self.node = context.node
        self.logger = self.node.get_logger()

        self._widget = MainWindow()
        context.add_widget(self._widget)
        self.logger.info("EUFSTracksGUI started!")
