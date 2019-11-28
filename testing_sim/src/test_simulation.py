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
        # rospy.Subscriber('/ros_can/state', CanState, self.state_callback)

        while not rospy.is_shutdown() and self.lap_count != 0: # or self.as_state != 4):
            continue

        self.assertEqual(self.lap_count, 0)
        #self.assertEqual(self.as_state, 4)


if __name__ == '__main__':

    rospy.init_node('test_lapcount')

    pub = rospy.Publisher('/ros_can/set_mission', CanState, queue_size=1)

    msg = rospy.wait_for_message('/ros_can/state', CanState)

    mission_msg = CanState()
    mission_msg.as_state = 0
    mission_msg.ami_state = 13
    mission_msg.mission_flag = False
    pub.publish(mission_msg)


    rostest.rosrun('testing_sim', 'test_simulation', SimulationTestClass)
