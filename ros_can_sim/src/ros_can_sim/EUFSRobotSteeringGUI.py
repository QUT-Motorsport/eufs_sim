# Copyright (c) 2011, Dirk Thomas, TU Darmstadt
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#   * Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#   * Redistributions in binary form must reproduce the above
#     copyright notice, this list of conditions and the following
#     disclaimer in the documentation and/or other materials provided
#     with the distribution.
#   * Neither the name of the TU Darmstadt nor the names of its
#     contributors may be used to endorse or promote products derived
#     from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import os

# qt
from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtCore import Qt, QTimer, Slot
from python_qt_binding.QtGui import QKeySequence
from python_qt_binding.QtWidgets import QComboBox, QShortcut, QWidget

# ROS
import rospkg
import rospy
from ackermann_msgs.msg import AckermannDriveStamped


class EUFSRobotSteeringGUI(Plugin):

    slider_factor = 1000.0

    def __init__(self, context):
        super(EUFSRobotSteeringGUI, self).__init__(context)
        self.setObjectName('EUFSRobotSteeringGUI')
        rp = rospkg.RosPack()

        # Create QWidget
        self._widget = QWidget()
        # Get path to UI file which is a sibling of this file
        ui_file = os.path.join(rp.get_path('ros_can_sim'), 'resource',
                               'EUFSRobotSteeringGUI.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        self._widget.setObjectName('EUFSRobotSteeringGUI')

        # Show _widget.windowTitle on left-top of each plugin (when
        # it's set in _widget). This is useful when you open multiple
        # plugins at once. Also if you open multiple instances of your
        # plugin at once, these lines add number to make it easy to
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(
                self._widget.windowTitle() + (' (%d)' % context.serial_number()))

        # Add widget to the user interface
        context.add_widget(self._widget)

        self._publisher = None

        self.inputs = ["Speed", "Acceleration", "Jerk"]

        self._widget.input_select_menu.addItems(self.inputs)
        self._widget.input_select_menu.setCurrentIndex(1)

        self._widget.topic_line_edit.textChanged.connect(
            self._on_topic_changed)
        self._widget.stop_push_button.pressed.connect(self._on_stop_pressed)

        self._widget.linear_slider.valueChanged.connect(
            self._on_linear_slider_changed)
        self._widget.angular_slider.valueChanged.connect(
            self._on_angular_slider_changed)

        self._widget.increase_linear_push_button.pressed.connect(
            self._on_strong_increase_linear_pressed)
        self._widget.reset_linear_push_button.pressed.connect(
            self._on_reset_linear_pressed)
        self._widget.decrease_linear_push_button.pressed.connect(
            self._on_strong_decrease_linear_pressed)
        self._widget.increase_angular_push_button.pressed.connect(
            self._on_strong_increase_angular_pressed)
        self._widget.reset_angular_push_button.pressed.connect(
            self._on_reset_angular_pressed)
        self._widget.decrease_angular_push_button.pressed.connect(
            self._on_strong_decrease_angular_pressed)

        self._widget.max_linear_double_spin_box.valueChanged.connect(
            self._on_max_linear_changed)
        self._widget.min_linear_double_spin_box.valueChanged.connect(
            self._on_min_linear_changed)
        self._widget.max_angular_double_spin_box.valueChanged.connect(
            self._on_max_angular_changed)
        self._widget.min_angular_double_spin_box.valueChanged.connect(
            self._on_min_angular_changed)

        self.shortcut_w = QShortcut(QKeySequence(Qt.Key_W), self._widget)
        self.shortcut_w.setContext(Qt.ApplicationShortcut)
        self.shortcut_w.activated.connect(self._on_increase_linear_pressed)
        self.shortcut_x = QShortcut(QKeySequence(Qt.Key_X), self._widget)
        self.shortcut_x.setContext(Qt.ApplicationShortcut)
        self.shortcut_x.activated.connect(self._on_reset_linear_pressed)
        self.shortcut_s = QShortcut(QKeySequence(Qt.Key_S), self._widget)
        self.shortcut_s.setContext(Qt.ApplicationShortcut)
        self.shortcut_s.activated.connect(self._on_decrease_linear_pressed)
        self.shortcut_a = QShortcut(QKeySequence(Qt.Key_A), self._widget)
        self.shortcut_a.setContext(Qt.ApplicationShortcut)
        self.shortcut_a.activated.connect(self._on_increase_angular_pressed)
        self.shortcut_z = QShortcut(QKeySequence(Qt.Key_Z), self._widget)
        self.shortcut_z.setContext(Qt.ApplicationShortcut)
        self.shortcut_z.activated.connect(self._on_reset_angular_pressed)
        self.shortcut_d = QShortcut(QKeySequence(Qt.Key_D), self._widget)
        self.shortcut_d.setContext(Qt.ApplicationShortcut)
        self.shortcut_d.activated.connect(self._on_decrease_angular_pressed)

        self.shortcut_shift_w = QShortcut(
            QKeySequence(Qt.SHIFT + Qt.Key_W), self._widget)
        self.shortcut_shift_w.setContext(Qt.ApplicationShortcut)
        self.shortcut_shift_w.activated.connect(
            self._on_strong_increase_linear_pressed)
        self.shortcut_shift_x = QShortcut(
            QKeySequence(Qt.SHIFT + Qt.Key_X), self._widget)
        self.shortcut_shift_x.setContext(Qt.ApplicationShortcut)
        self.shortcut_shift_x.activated.connect(
            self._on_reset_linear_pressed)
        self.shortcut_shift_s = QShortcut(
            QKeySequence(Qt.SHIFT + Qt.Key_S), self._widget)
        self.shortcut_shift_s.setContext(Qt.ApplicationShortcut)
        self.shortcut_shift_s.activated.connect(
            self._on_strong_decrease_linear_pressed)
        self.shortcut_shift_a = QShortcut(
            QKeySequence(Qt.SHIFT + Qt.Key_A), self._widget)
        self.shortcut_shift_a.setContext(Qt.ApplicationShortcut)
        self.shortcut_shift_a.activated.connect(
            self._on_strong_increase_angular_pressed)
        self.shortcut_shift_z = QShortcut(
            QKeySequence(Qt.SHIFT + Qt.Key_Z), self._widget)
        self.shortcut_shift_z.setContext(Qt.ApplicationShortcut)
        self.shortcut_shift_z.activated.connect(
            self._on_reset_angular_pressed)
        self.shortcut_shift_d = QShortcut(
            QKeySequence(Qt.SHIFT + Qt.Key_D), self._widget)
        self.shortcut_shift_d.setContext(Qt.ApplicationShortcut)
        self.shortcut_shift_d.activated.connect(
            self._on_strong_decrease_angular_pressed)

        self.shortcut_space = QShortcut(
            QKeySequence(Qt.Key_Space), self._widget)
        self.shortcut_space.setContext(Qt.ApplicationShortcut)
        self.shortcut_space.activated.connect(self._on_stop_pressed)
        self.shortcut_space = QShortcut(
            QKeySequence(Qt.SHIFT + Qt.Key_Space), self._widget)
        self.shortcut_space.setContext(Qt.ApplicationShortcut)
        self.shortcut_space.activated.connect(self._on_stop_pressed)

        self._widget.stop_push_button.setToolTip(
            self._widget.stop_push_button.toolTip() + ' ' + self.tr('([Shift +] Space)'))
        self._widget.increase_linear_push_button.setToolTip(
            self._widget.increase_linear_push_button.toolTip() + ' ' + self.tr('([Shift +] W)'))
        self._widget.reset_linear_push_button.setToolTip(
            self._widget.reset_linear_push_button.toolTip() + ' ' + self.tr('([Shift +] X)'))
        self._widget.decrease_linear_push_button.setToolTip(
            self._widget.decrease_linear_push_button.toolTip() + ' ' + self.tr('([Shift +] S)'))
        self._widget.increase_angular_push_button.setToolTip(
            self._widget.increase_angular_push_button.toolTip() + ' ' + self.tr('([Shift +] A)'))
        self._widget.reset_angular_push_button.setToolTip(
            self._widget.reset_angular_push_button.toolTip() + ' ' + self.tr('([Shift +] Z)'))
        self._widget.decrease_angular_push_button.setToolTip(
            self._widget.decrease_angular_push_button.toolTip() + ' ' + self.tr('([Shift +] D)'))

        # timer to consecutively send AckermannDriveStamped messages
        self._update_parameter_timer = QTimer(self)
        self._update_parameter_timer.timeout.connect(
            self._on_parameter_changed)
        self._update_parameter_timer.start(100)
        self.zero_cmd_sent = False

    @Slot(str)
    def _on_topic_changed(self, topic):
        topic = str(topic)
        self._unregister_publisher()
        if topic == '':
            return
        try:
            self._publisher = rospy.Publisher(topic, AckermannDriveStamped, queue_size=10)
        except TypeError:
            self._publisher = rospy.Publisher(topic, AckermannDriveStamped)

    def _on_stop_pressed(self):
        # If the current value of sliders is zero directly send stop AckermannDriveStamped msg
        if self._widget.linear_slider.value() == 0 and \
                self._widget.angular_slider.value() == 0:
            self.zero_cmd_sent = False
            self._on_parameter_changed()
        else:
            self._widget.linear_slider.setValue(0)
            self._widget.angular_slider.setValue(0)

    def _on_linear_slider_changed(self):
        self._widget.current_linear_label.setText(
            '%0.2f m/s' % (self._widget.linear_slider.value() / EUFSRobotSteeringGUI.slider_factor))
        self._on_parameter_changed()

    def _on_angular_slider_changed(self):
        self._widget.current_angular_label.setText(
            '%0.2f rad/s' % (self._widget.angular_slider.value() / EUFSRobotSteeringGUI.slider_factor))
        self._on_parameter_changed()

    def _on_increase_linear_pressed(self):
        self._widget.linear_slider.setValue(
            self._widget.linear_slider.value() + self._widget.linear_slider.singleStep())

    def _on_reset_linear_pressed(self):
        self._widget.linear_slider.setValue(0)

    def _on_decrease_linear_pressed(self):
        self._widget.linear_slider.setValue(
            self._widget.linear_slider.value() - self._widget.linear_slider.singleStep())

    def _on_increase_angular_pressed(self):
        self._widget.angular_slider.setValue(
            self._widget.angular_slider.value() + self._widget.angular_slider.singleStep())

    def _on_reset_angular_pressed(self):
        self._widget.angular_slider.setValue(0)

    def _on_decrease_angular_pressed(self):
        self._widget.angular_slider.setValue(
            self._widget.angular_slider.value() - self._widget.angular_slider.singleStep())

    def _on_max_linear_changed(self, value):
        self._widget.linear_slider.setMaximum(
            value * EUFSRobotSteeringGUI.slider_factor)

    def _on_min_linear_changed(self, value):
        self._widget.linear_slider.setMinimum(
            value * EUFSRobotSteeringGUI.slider_factor)

    def _on_max_angular_changed(self, value):
        self._widget.angular_slider.setMaximum(
            value * EUFSRobotSteeringGUI.slider_factor)

    def _on_min_angular_changed(self, value):
        self._widget.angular_slider.setMinimum(
            value * EUFSRobotSteeringGUI.slider_factor)

    def _on_strong_increase_linear_pressed(self):
        self._widget.linear_slider.setValue(
            self._widget.linear_slider.value() + self._widget.linear_slider.pageStep())

    def _on_strong_decrease_linear_pressed(self):
        self._widget.linear_slider.setValue(
            self._widget.linear_slider.value() - self._widget.linear_slider.pageStep())

    def _on_strong_increase_angular_pressed(self):
        self._widget.angular_slider.setValue(
            self._widget.angular_slider.value() + self._widget.angular_slider.pageStep())

    def _on_strong_decrease_angular_pressed(self):
        self._widget.angular_slider.setValue(
            self._widget.angular_slider.value() - self._widget.angular_slider.pageStep())

    def _on_parameter_changed(self):
        self._send_ackermann_drive_stamped(
            self._widget.linear_slider.value() / EUFSRobotSteeringGUI.slider_factor,
            self._widget.angular_slider.value() / EUFSRobotSteeringGUI.slider_factor)

    def _send_ackermann_drive_stamped(self, linear, angular):
        if self._publisher is None:
            return

        drive = AckermannDriveStamped()
        drive.header.stamp = rospy.Time.now()

        drive.drive.acceleration = 0
        drive.drive.speed = 0
        drive.drive.jerk = 0
        input = self._widget.input_select_menu.currentIndex()
        if self.inputs[input] == "Speed":
            drive.drive.speed = linear
        elif self.inputs[input] == "Acceleration":
            drive.drive.acceleration = linear
        elif self.inputs[input] == "Jerk":
            drive.drive.jerk = linear

        drive.drive.steering_angle = angular
        drive.drive.steering_angle_velocity = 0

        # Only send the zero command once so other devices can take control
        if linear == 0 and angular == 0:
            if not self.zero_cmd_sent:
                self.zero_cmd_sent = True
                self._publisher.publish(drive)
        else:
            self.zero_cmd_sent = False
            self._publisher.publish(drive)

    def _unregister_publisher(self):
        if self._publisher is not None:
            self._publisher.unregister()
            self._publisher = None

    def shutdown_plugin(self):
        self._update_parameter_timer.stop()
        self._unregister_publisher()

    def save_settings(self, plugin_settings, instance_settings):
        instance_settings.set_value(
            'input', self.input)
        instance_settings.set_value(
            'topic', self._widget.topic_line_edit.text())
        instance_settings.set_value(
            'vx_max', self._widget.max_linear_double_spin_box.value())
        instance_settings.set_value(
            'vx_min', self._widget.min_linear_double_spin_box.value())
        instance_settings.set_value(
            'vw_max', self._widget.max_angular_double_spin_box.value())
        instance_settings.set_value(
            'vw_min', self._widget.min_angular_double_spin_box.value())

    def restore_settings(self, plugin_settings, instance_settings):
        value = instance_settings.value('input', 1)
        value = rospy.get_param('~default_input', value)
        self.input = value

        value = instance_settings.value('topic', '/rqt/command')
        value = rospy.get_param('~default_topic', value)
        self._widget.topic_line_edit.setText(value)

        value = self._widget.max_linear_double_spin_box.value()
        value = instance_settings.value('vx_max', value)
        value = rospy.get_param('~default_vx_max', value)
        self._widget.max_linear_double_spin_box.setValue(float(value))

        value = self._widget.min_linear_double_spin_box.value()
        value = instance_settings.value('vx_min', value)
        value = rospy.get_param('~default_vx_min', value)
        self._widget.min_linear_double_spin_box.setValue(float(value))

        value = self._widget.max_angular_double_spin_box.value()
        value = instance_settings.value('vw_max', value)
        value = rospy.get_param('~default_vw_max', value)
        self._widget.max_angular_double_spin_box.setValue(float(value))

        value = self._widget.min_angular_double_spin_box.value()
        value = instance_settings.value('vw_min', value)
        value = rospy.get_param('~default_vw_min', value)
        self._widget.min_angular_double_spin_box.setValue(float(value))
