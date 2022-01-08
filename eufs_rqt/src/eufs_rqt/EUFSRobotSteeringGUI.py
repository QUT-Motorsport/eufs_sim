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
import time

# qt
from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtCore import Qt, QTimer
from python_qt_binding.QtGui import QKeySequence
from python_qt_binding.QtWidgets import QShortcut, QWidget

# ROS
from ament_index_python.packages import get_package_share_directory
import rclpy
from std_srvs.srv import Trigger
from ackermann_msgs.msg import AckermannDriveStamped


class EUFSRobotSteeringGUI(Plugin):
    slider_factor = 1000.0

    def __init__(self, context):
        super(EUFSRobotSteeringGUI, self).__init__(context)
        self.setObjectName('EUFSRobotSteeringGUI')

        self.node = context.node
        self.logger = self.node.get_logger()

        # Create QWidget
        self._widget = QWidget()
        # Get path to UI file which is a sibling of this file
        ui_file = os.path.join(get_package_share_directory('eufs_rqt'),
                               'resource',
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
                self._widget.windowTitle() + (
                    ' (%d)' % context.serial_number()))

        # Add widget to the user interface
        context.add_widget(self._widget)

        self._publisher = None

        self._widget.topic_line_edit.textChanged.connect(
            self._on_topic_changed)
        self._widget.topic_line_edit.editingFinished.connect(
            self._on_topic_set)
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
            self._widget.stop_push_button.toolTip() + ' ' + self.tr(
                '([Shift +] Space)'))
        self._widget.increase_linear_push_button.setToolTip(
            self._widget.increase_linear_push_button.toolTip() + ' ' + self.tr(
                '([Shift +] W)'))
        self._widget.reset_linear_push_button.setToolTip(
            self._widget.reset_linear_push_button.toolTip() + ' ' + self.tr(
                '([Shift +] X)'))
        self._widget.decrease_linear_push_button.setToolTip(
            self._widget.decrease_linear_push_button.toolTip() + ' ' + self.tr(
                '([Shift +] S)'))
        self._widget.increase_angular_push_button.setToolTip(
            self._widget.increase_angular_push_button.toolTip() + ' '
            + self.tr(
                '([Shift +] A)'))
        self._widget.reset_angular_push_button.setToolTip(
            self._widget.reset_angular_push_button.toolTip() + ' ' + self.tr(
                '([Shift +] Z)'))
        self._widget.decrease_angular_push_button.setToolTip(
            self._widget.decrease_angular_push_button.toolTip() + ' '
            + self.tr(
                '([Shift +] D)'))

        # Service to query the race car model for command mode on startup
        self.command_mode_srv = self.node.create_client(
            Trigger, "/race_car_model/command_mode")
        self.command_mode = self.request_command_mode()

        # Configure default maximum slider value
        default_vel_range = 5.00
        default_acc_range = 1.00

        if self.command_mode == "acceleration":
            self._widget.max_linear_double_spin_box.setValue(default_acc_range)
            self._widget.min_linear_double_spin_box.setValue(
                -default_acc_range)
            self.slider_units = "m/s^2"
        elif self.command_mode == "velocity":
            self._widget.max_linear_double_spin_box.setValue(default_vel_range)
            self._widget.min_linear_double_spin_box.setValue(
                -default_vel_range)
            self.slider_units = "m/s"
        else:
            self.logger.error(
                "Invalid command mode: '{}', must be 'acceleration' or "
                "'velocity'".format(
                    self.command_mode))

        # Update sliders to use configurable units
        self._widget.current_linear_label.setText(
            ('%0.2f ' + self.slider_units) % (
                self._widget.linear_slider.value()
                / EUFSRobotSteeringGUI.slider_factor))

        self._widget.max_linear_double_spin_box.setToolTip(
            "Maximum linear " + self.command_mode)
        self._widget.min_linear_double_spin_box.setToolTip(
            "Minimum linear " + self.command_mode)
        self._widget.reset_linear_push_button.setToolTip(
            "Reset linear " + self.command_mode)
        self._widget.increase_linear_push_button.setToolTip(
            "Increase linear " + self.command_mode)
        self._widget.decrease_linear_push_button.setToolTip(
            "Decrease linear " + self.command_mode)

        # timer to consecutively send AckermannDriveStamped messages
        self._update_parameter_timer = QTimer(self)
        self._update_parameter_timer.timeout.connect(
            self._on_parameter_changed)
        self._update_parameter_timer.start(100)
        self.zero_cmd_sent = False

    def request_command_mode(self):
        """Requests command mode from race_car_model"""
        while not self.command_mode_srv.wait_for_service(timeout_sec=4.0):
            self.logger.debug(
                'command mode service not available, waiting again...')

        req = Trigger.Request()
        future = self.command_mode_srv.call_async(req)

        while not future.done():
            self.logger.debug(
                "Waiting for command mode request to complete...")
            time.sleep(0.1)

        self.logger.debug("command mode request completed.")
        result = future.result()
        return result.message

    def _on_topic_changed(self, topic):
        self._unregister_publisher()
        self.topic = str(topic)

    def _on_topic_set(self, log=True):
        # Publisher is unregistered when the topic is changed
        # If the publisher has not been unregistered, do not need to set it
        if self._publisher is not None:
            return

        if self.topic == '':
            self.logger.error(
                "Could NOT set EUFS Robot Steering GUI publisher's topic to "
                "be empty")
            return

        # Catches "topics can't end in backslash" error
        try:
            self._publisher = self.node.create_publisher(
                AckermannDriveStamped, self.topic, 10)
            self._update_parameter_timer.start(100)
            if log:
                self.logger.info(
                    "Set EUFS Robot Steering GUI publisher's topic to: "
                    + self.topic)
            else:
                self.logger.debug(
                    "Set EUFS Robot Steering GUI publisher's topic to: "
                    + self.topic)
        except rclpy.exceptions.InvalidTopicNameException:
            self.logger.error(
                "Could NOT set EUFS Robot Steering GUI publisher's topic to: "
                "" + self.topic)
            return

    def _on_stop_pressed(self):
        # If the current value of sliders is zero directly send stop
        # AckermannDriveStamped msg
        if self._widget.linear_slider.value() == 0 and \
                self._widget.angular_slider.value() == 0:
            self.zero_cmd_sent = False
            self._on_parameter_changed()
        else:
            self._widget.linear_slider.setValue(0)
            self._widget.angular_slider.setValue(0)

    def _on_linear_slider_changed(self):
        self._widget.current_linear_label.setText(
            ('%0.2f ' + self.slider_units) % (
                self._widget.linear_slider.value()
                / EUFSRobotSteeringGUI.slider_factor))
        self._on_parameter_changed()

    def _on_angular_slider_changed(self):
        self._widget.current_angular_label.setText(
            '%0.2f rad' % (
                self._widget.angular_slider.value()
                / EUFSRobotSteeringGUI.slider_factor))
        self._on_parameter_changed()

    def _on_increase_linear_pressed(self):
        self._widget.linear_slider.setValue(
            self._widget.linear_slider.value()
            + self._widget.linear_slider.singleStep())

    def _on_reset_linear_pressed(self):
        self._widget.linear_slider.setValue(0)

    def _on_decrease_linear_pressed(self):
        self._widget.linear_slider.setValue(
            self._widget.linear_slider.value()
            - self._widget.linear_slider.singleStep())

    def _on_increase_angular_pressed(self):
        self._widget.angular_slider.setValue(
            self._widget.angular_slider.value()
            + self._widget.angular_slider.singleStep())

    def _on_reset_angular_pressed(self):
        self._widget.angular_slider.setValue(0)

    def _on_decrease_angular_pressed(self):
        self._widget.angular_slider.setValue(
            self._widget.angular_slider.value()
            - self._widget.angular_slider.singleStep())

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
            self._widget.linear_slider.value()
            + self._widget.linear_slider.pageStep())

    def _on_strong_decrease_linear_pressed(self):
        self._widget.linear_slider.setValue(
            self._widget.linear_slider.value()
            - self._widget.linear_slider.pageStep())

    def _on_strong_increase_angular_pressed(self):
        self._widget.angular_slider.setValue(
            self._widget.angular_slider.value()
            + self._widget.angular_slider.pageStep())

    def _on_strong_decrease_angular_pressed(self):
        self._widget.angular_slider.setValue(
            self._widget.angular_slider.value()
            - self._widget.angular_slider.pageStep())

    def _on_parameter_changed(self):
        self._send_ackermann_drive_stamped(
            self._widget.linear_slider.value()
            / EUFSRobotSteeringGUI.slider_factor,
            self._widget.angular_slider.value()
            / EUFSRobotSteeringGUI.slider_factor)

    def _send_ackermann_drive_stamped(self, linear, angular):
        if self._publisher is None:
            return

        drive = AckermannDriveStamped()
        drive.header.stamp = self.node.get_clock().now().to_msg()

        drive.drive.acceleration = 0.0
        drive.drive.speed = 0.0
        if self.command_mode == "velocity":
            drive.drive.speed = linear
        elif self.command_mode == "acceleration":
            drive.drive.acceleration = linear

        drive.drive.steering_angle = angular
        drive.drive.steering_angle_velocity = 0.0

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
            self._update_parameter_timer.stop()
            assert (self.node.destroy_publisher(
                self._publisher)), 'Publisher could not be destroyed.'
            self._publisher = None

    def shutdown_plugin(self):
        self._update_parameter_timer.stop()
        self._unregister_publisher()
        # Note: do not destroy the node here as it could cause errors for
        # the Mission Control GUI
        # Let ROS 2 clean up nodes

    def save_settings(self, plugin_settings, instance_settings):
        instance_settings.set_value(
            'ackermann_topic', self._widget.topic_line_edit.text())

        if self.command_mode == "velocity":
            instance_settings.set_value(
                'velx_max', self._widget.max_linear_double_spin_box.value())
            instance_settings.set_value(
                'velx_min', self._widget.min_linear_double_spin_box.value())
        elif self.command_mode == "acceleration":
            instance_settings.set_value(
                'accx_max', self._widget.max_linear_double_spin_box.value())
            instance_settings.set_value(
                'accx_min', self._widget.min_linear_double_spin_box.value())

        instance_settings.set_value(
            'w_max', self._widget.max_angular_double_spin_box.value())
        instance_settings.set_value(
            'w_min', self._widget.min_angular_double_spin_box.value())

    def get_param(self, instance_settings, param_name, value):
        value = instance_settings.value(param_name, value)
        if not self.node.has_parameter('default_' + param_name):
            self.node.declare_parameter('default_' + param_name, value)
        value = self.node.get_parameter('default_' + param_name).value
        return value

    def restore_settings(self, plugin_settings, instance_settings):
        value = self.get_param(instance_settings, 'ackermann_topic',
                               '/cmd')
        self._widget.topic_line_edit.setText(str(value))

        # In order to make the topic be set
        self._on_topic_set(log=False)

        if self.command_mode == 'acceleration':
            value = self.get_param(
                instance_settings, 'acc_max',
                self._widget.max_linear_double_spin_box.value())
            self._widget.max_linear_double_spin_box.setValue(float(value))
            value = self.get_param(
                instance_settings, 'acc_min',
                self._widget.min_linear_double_spin_box.value())
            self._widget.min_linear_double_spin_box.setValue(float(value))

        elif self.command_mode == 'velocity':
            value = self.get_param(
                instance_settings, 'velx_max',
                self._widget.max_linear_double_spin_box.value())
            self._widget.max_linear_double_spin_box.setValue(float(value))
            value = self.get_param(
                instance_settings, 'velx_min',
                self._widget.min_linear_double_spin_box.value())
            self._widget.min_linear_double_spin_box.setValue(float(value))

        value = self.get_param(
            instance_settings, 'w_max',
            self._widget.max_angular_double_spin_box.value())
        self._widget.max_angular_double_spin_box.setValue(float(value))
        value = self.get_param(
            instance_settings, 'w_min',
            self._widget.min_angular_double_spin_box.value())
        self._widget.min_angular_double_spin_box.setValue(float(value))
