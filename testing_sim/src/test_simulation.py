#!/usr/bin/env python

import rospy
import unittest
import rostest
from time import sleep

from std_msgs.msg import Int16
from eufs_msgs.msg import CanState

class SimulationTestClass(unittest.TestCase):

    lap_count = -1

    def callback(self, msg):
        self.lap_count = msg.data

    def test_lapcount(self):
        rospy.Subscriber('/finish_line_detector/completed_laps', Int16, self.callback)

        count = 0
        while not rospy.is_shutdown() and self.lap_count == -1:
            count += 1

        self.assertNotEqual(self.lap_count, -1)

if __name__ == '__main__':

    rospy.init_node('test_lapcount')

    pub = rospy.Publisher('/ros_can/set_mission', CanState, queue_size=1)
    mission_msg = CanState()
    mission_msg.as_state = 0
    mission_msg.ami_state = 13
    mission_msg.mission_flag = False
    pub.publish(mission_msg)

    rostest.rosrun('testing_sim', 'test_simulation', SimulationTestClass)
