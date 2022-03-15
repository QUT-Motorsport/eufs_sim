import os
import threading

# qt
from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget, QComboBox, QPushButton, QLabel

# ROS
from ament_index_python.packages import get_package_share_directory
import rclpy
from eufs_msgs.msg import CanState
from std_srvs.srv import Trigger
from eufs_msgs.srv import SetCanState


class MissionControlGUI(Plugin):

    def __init__(self, context):
        super(MissionControlGUI, self).__init__(context)
        self.setObjectName('MissionControlGUI')

        self.node = context.node

        # Create QWidget
        self._widget = QWidget()
        # Get path to UI file which is a sibling of this file
        ui_file = os.path.join(get_package_share_directory('eufs_rqt'),
                               'resource',
                               'MissionControlGUI.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        self._widget.setObjectName('MissionControlUi')

        # Show _widget.windowTitle on left-top of each plugin (when
        # it's set in _widget). This is useful when you open multiple
        # plugins at once. Also if you open multiple instances of your
        # plugin at once, these lines add number to make it easy to
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(
                self._widget.windowTitle() + (
                    ' (%d)' % context.serial_number()))

        # Enumrations taken from CanState.msg
        self.states = {CanState.AS_OFF: "OFF",
                       CanState.AS_READY: "READY",
                       CanState.AS_DRIVING: "DRIVING",
                       CanState.AS_EMERGENCY_BRAKE: "EMERGENCY",
                       CanState.AS_FINISHED: "FINISHED"}

        # Autonomous missions
        self.missions = {CanState.AMI_NOT_SELECTED: "NOT_SELECTED",
                         CanState.AMI_ACCELERATION: "ACCELERATION",
                         CanState.AMI_SKIDPAD: "SKIDPAD",
                         CanState.AMI_AUTOCROSS: "AUTOCROSS",
                         CanState.AMI_TRACK_DRIVE: "TRACK_DRIVE",
                         CanState.AMI_AUTONOMOUS_DEMO: "AUTONOMOUS_DEMO",
                         CanState.AMI_ADS_INSPECTION: "ADS_INSPECTION",
                         CanState.AMI_ADS_EBS: "ADS_EBS",
                         CanState.AMI_DDT_INSPECTION_A: "DDT_INSPECTION_A",
                         CanState.AMI_DDT_INSPECTION_B: "DDT_INSPECTION_B",
                         CanState.AMI_JOYSTICK: "JOYSTICK",
                         }

        for mission in self.missions.values():
            self._widget.findChild(
                QComboBox, "MissionSelectMenu").addItem(mission)

        # hook up buttons to callbacks
        self._widget.findChild(
            QPushButton, "SetMissionButton").clicked.connect(self.setMission)
        self._widget.findChild(
            QPushButton, "ResetButton").clicked.connect(self.resetState)
        self._widget.findChild(
            QPushButton, "ResetSimButton").clicked.connect(self.resetSim)
        self._widget.findChild(
            QPushButton, "RequestEBS").clicked.connect(self.requestEBS)
        self._widget.findChild(
            QPushButton, "DriveButton").clicked.connect(self.setManualDriving)

        # Subscribers
        self.state_sub = self.node.create_subscription(CanState,
                                                       "/ros_can/state",
                                                       self.stateCallback, 10)

        # Publishers

        # Services
        self.ebs_srv = self.node.create_client(Trigger, "/ros_can/ebs")
        self.reset_srv = self.node.create_client(Trigger, "/ros_can/reset")
        self.set_mission_cli = self.node.create_client(SetCanState, "/ros_can/set_mission")
        self.reset_vehicle_pos_srv = self.node.create_client(Trigger,
                                                             "/ros_can/reset_vehicle_pos")
        self.reset_cone_pos_srv = self.node.create_client(Trigger,
                                                          "/ros_can/reset_cone_pos")

        # Add widget to the user interface
        context.add_widget(self._widget)

        thread = threading.Thread(target=self.ros_spin, daemon=True)
        thread.start()

    def ros_spin(self):
        rclpy.spin(self.node)

    def sendRequest(self, mission_ami_state):
        """Sends a mission request to the simulated ros_can
        The mission request is of message type eufs_msgs/srv/SetCanState
        where only the ami_state field is used.
        """
        if self.set_mission_cli.wait_for_service(timeout_sec=1):
            request = SetCanState.Request()
            request.ami_state = mission_ami_state
            result = self.set_mission_cli.call_async(request)
            self.node.get_logger().debug("Mission request sent successfully")
            self.node.get_logger().debug(result)
        else:
            self.node.get_logger().warn(
                "/ros_can/set_mission service is not available")

    def setMission(self):
        """Requests ros_can to set mission"""
        mission = self._widget.findChild(
            QComboBox, "MissionSelectMenu").currentText()

        self.node.get_logger().debug(
            "Sending mission request for " + str(mission))

        # create message to be sent
        mission_msg = CanState()

        # find enumerated mission and set
        for enum, mission_name in self.missions.items():
            if mission_name == mission:
                mission_msg.ami_state = enum
                break

        self.sendRequest(mission_msg.ami_state)

    def setManualDriving(self):
        self.node.get_logger().debug("Sending manual mission request")
        mission_msg = CanState()
        mission_msg.ami_state = CanState.AMI_MANUAL
        self.sendRequest(mission_msg.ami_state)

    def resetState(self):
        """Requests state_machine reset"""
        self.node.get_logger().debug("Requesting state_machine reset")

        if self.reset_srv.wait_for_service(timeout_sec=1):
            request = Trigger.Request()
            result = self.reset_srv.call_async(request)
            self.node.get_logger().debug("state reset successful")
            self.node.get_logger().debug(result)
        else:
            self.node.get_logger().warn(
                "/ros_can/reset service is not available")

    def resetVehiclePos(self):
        """Requests race car model position reset"""
        self.node.get_logger().debug(
            "Requesting race_car_model position reset")

        if self.reset_vehicle_pos_srv.wait_for_service(timeout_sec=1):
            request = Trigger.Request()
            result = self.reset_vehicle_pos_srv.call_async(request)
            self.node.get_logger().debug("Vehicle position reset successful")
            self.node.get_logger().debug(result)
        else:
            self.node.get_logger().warn(
                "/ros_can/reset_vehicle_pos service is not available")

    def resetConePos(self):
        """Requests gazebo_cone_ground_truth to reset cone position"""
        self.node.get_logger().debug(
            "Requesting gazebo_cone_ground_truth cone position reset")

        if self.reset_cone_pos_srv.wait_for_service(timeout_sec=1):
            request = Trigger.Request()
            result = self.reset_cone_pos_srv.call_async(request)
            self.node.get_logger().debug("Cone position reset successful")
            self.node.get_logger().debug(result)
        else:
            self.node.get_logger().warn(
                "/ros_can/reset_cone_pos service is not available")

    def resetSim(self):
        """Requests state machine, vehicle position and cone position reset"""
        self.node.get_logger().debug("Requesting Simulation Reset")

        # Reset State Machine
        self.resetState()

        # Reset Vehicle Position
        self.resetVehiclePos()

        # Reset Cone Position
        self.resetConePos()

    def requestEBS(self):
        """Requests ros_can to go into EMERGENCY_BRAKE state"""
        self.node.get_logger().debug("Requesting EBS")

        if self.ebs_srv.wait_for_service(timeout_sec=1):
            request = Trigger.Request()
            result = self.ebs_srv.call_async(request)
            self.node.get_logger().debug("EBS successful")
            self.node.get_logger().debug(result)
        else:
            self.node.get_logger().warn(
                "/ros_can/ebs service is not available")

    def stateCallback(self, msg):
        """Reads the robot state from the message
        and displays it within the GUI

        Args:
            msg (eufs_msgs/CanState): state of race car
        """
        if msg.ami_state == CanState.AMI_MANUAL:
            self._widget.findChild(QLabel, "StateDisplay").setText(
                "Manual Driving")
            self._widget.findChild(QLabel, "MissionDisplay").setText(
                "MANUAL")
        else:
            self._widget.findChild(QLabel, "StateDisplay").setText(
                self.states[msg.as_state])
            self._widget.findChild(QLabel, "MissionDisplay").setText(
                self.missions[msg.ami_state])

    def shutdown_plugin(self):
        """stop all publisher, subscriber and services
        necessary for clean shutdown"""
        assert (self.node.destroy_client(
            self.set_mission_cli)), "Mission client could not be destroyed"
        assert (self.node.destroy_subscription(
            self.state_sub)), "State subscriber could not be destroyed"
        assert (self.node.destroy_client(
            self.ebs_srv)), "EBS client could not be destroyed"
        assert (self.node.destroy_client(
            self.reset_srv)), "State reset client could not be destroyed"
        # Note: do not destroy the node in shutdown_plugin as this could
        # cause errors for the Robot Steering GUI. Let ROS 2 clean up nodes

    def save_settings(self, plugin_settings, instance_settings):
        # don't know how to use
        # don't delete function as it breaks rqt
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # don't know how to use
        # don't delete function as it breaks rqt
        pass
