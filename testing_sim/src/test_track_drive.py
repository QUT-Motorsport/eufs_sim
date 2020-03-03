#!/usr/bin/env python

import rospy
import unittest
import rostest
from time import sleep

from std_msgs.msg import Int16
from eufs_msgs.msg import CanState

class SimulationTestClass(unittest.TestCase):

    lap_count = -1
    as_state = 0

    def laps_callback(self, msg):
        self.lap_count = msg.data

    def state_callback(self, msg):
        self.as_state = msg.as_state

    def test_lapcount(self):
        rospy.Subscriber('/finish_line_detector/completed_laps', Int16, self.laps_callback)
        rospy.Subscriber('/ros_can/state', CanState, self.state_callback)

        # Wait until the mission is completed or ros is shutdown
        # There is a timeout in the launch file
        while not rospy.is_shutdown() and not (self.lap_count == 10 and self.as_state == 4):
            rospy.sleep(0.1)

        self.assertEqual(self.lap_count, 10)
        self.assertEqual(self.as_state, 4)


if __name__ == '__main__':

    rospy.init_node('test_track_drive')

    pub = rospy.Publisher('/ros_can/set_mission', CanState, queue_size=1)

    # Waits for a message which signifies that the car is ready to recieve a message to select the mission
    msg = rospy.wait_for_message('/ros_can/state', CanState)

    # Publish a message to the car to signify the mission
    mission_msg = CanState()
    mission_msg.as_state = 0
    mission_msg.ami_state = 14 # Track drive
    mission_msg.mission_flag = False
    pub.publish(mission_msg)

    rostest.rosrun('testing_sim', 'test_simulation', SimulationTestClass)
