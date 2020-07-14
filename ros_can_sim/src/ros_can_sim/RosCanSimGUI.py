import os

# qt
from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget, QComboBox, QPushButton, QLabel

# ROS
import rospkg
import rospy
from eufs_msgs.msg import CanState
from std_srvs.srv import Trigger


class RosCanGUI(Plugin):

    def __init__(self, context):
        super(RosCanGUI, self).__init__(context)
        self.setObjectName('RosCanSimGUI')
        rp = rospkg.RosPack()

        # Create QWidget
        self._widget = QWidget()
        # Get path to UI file which is a sibling of this file
        ui_file = os.path.join(rp.get_path('ros_can_sim'), 'resource',
                               'RosCanSimGUI.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        self._widget.setObjectName('RosCanSimUi')

        # Show _widget.windowTitle on left-top of each plugin (when
        # it's set in _widget). This is useful when you open multiple
        # plugins at once. Also if you open multiple instances of your
        # plugin at once, these lines add number to make it easy to
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(
                self._widget.windowTitle() + (' (%d)' % context.serial_number()))

        # setup states and missions
        # enumrations taken from CanState.msg
        self.states = {CanState.AS_OFF: "OFF",
                       CanState.AS_READY: "READY",
                       CanState.AS_DRIVING: "DRIVING",
                       CanState.AS_EMERGENCY_BRAKE: "EMERGENCY",
                       CanState.AS_FINISHED: "FINISHED"}

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
                         CanState.AMI_MANUAL: "MANUAL"}

        for mission in self.missions.values():
            self._widget.findChild(
                QComboBox, "MissionSelectMenu").addItem(mission)

        # hook up buttons to callbacks
        self._widget.findChild(
            QPushButton, "SetMissionButton").clicked.connect(self.setMission)
        self._widget.findChild(
            QPushButton, "ResetButton").clicked.connect(self.resetState)
        self._widget.findChild(
            QPushButton, "RequestEBS").clicked.connect(self.requestEBS)
        self._widget.findChild(
            QPushButton, "DriveButton").clicked.connect(self.justDrive)

        # Subscribers
        self.state_sub = rospy.Subscriber(
            "/ros_can/state", CanState, self.stateCallback)

        # Publishers
        self.set_mission_pub = rospy.Publisher(
            "/ros_can/set_mission", CanState, queue_size=1)

        # Services
        self.ebs_srv = rospy.ServiceProxy("/ros_can/ebs", Trigger)
        self.reset_srv = rospy.ServiceProxy("/ros_can/reset", Trigger)

        # Add widget to the user interface
        context.add_widget(self._widget)

    def setMission(self):
        """Sends a mission request to the simulated ros_can
        The mission request is of message type eufs_msgs/CanState
        where only the ami_mission field is used"""
        mission = self._widget.findChild(
            QComboBox, "MissionSelectMenu").currentText()

        rospy.logdebug("Sending mission request for " + str(mission))

        # create message to be sent
        mission_msg = CanState()

        # find enumerated mission and set
        for enum, mission_name in self.missions.items():
            if mission_name == mission:
                mission_msg.ami_state = enum

        self.set_mission_pub.publish(mission_msg)
        rospy.logdebug("Mission request sent successfully")

    def resetState(self):
        """Requests ros_can to reset it's state machine"""
        rospy.logdebug("Requesting ros_can_sim reset")
        try:
            self.reset_srv.wait_for_service(timeout=1)  # 1 second
            result = self.reset_srv()
            rospy.logdebug("ros_can_sim reset successful")
            rospy.logdebug(result)
        except:
            rospy.logwarn(
                "Requested ros_can_sim reset but /ros_can/reset service is not available")

    def requestEBS(self):
        """Requests ros_can to go into EMERGENCY_BRAKE state"""
        rospy.logdebug("Requesting ros_can_sim reset")
        try:
            self.ebs_srv.wait_for_service(timeout=1)  # 1 second
            result = self.ebs_srv()
            rospy.logdebug("ros_can_sim reset successful")
            rospy.logdebug(result)
        except:
            rospy.logwarn(
                "Requested ros_can_sim EBS but /ros_can/ebs service is not available")

    def stateCallback(self, msg):
        """Reads the ros_can state from the message
        and displays it within the GUI

        Args:
            msg (eufs_msgs/CanState): state of ros_can
        """
        self._widget.findChild(QLabel, "StateDisplay").setText(
            self.states[msg.as_state])
        self._widget.findChild(QLabel, "MissionDisplay").setText(
            self.missions[msg.ami_state])

    def justDrive(self):
        """overrides the state machine of the car and just makes it drive"""

        # create message to be sent
        state_msg = CanState()
        state_msg.as_state = CanState.AS_DRIVING
        state_msg.ami_state = CanState.AMI_MANUAL

        self.set_mission_pub.publish(state_msg)

    def shutdown_plugin(self):
        """stop all publisher, subscriber and services
        necessary for clean shutdown"""
        self.set_mission_pub.unregister()
        self.state_sub.unregister()
        self.ebs_srv.close()
        self.reset_srv.close()

    def save_settings(self, plugin_settings, instance_settings):
        # don't know how to use
        # don't delete function as it breaks rqt
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # don't know how to use
        # don't delete function as it breaks rqt
        pass
