#!/usr/bin/env python

# Publishes to rqt/command
# Subscribes to cmd_vel and rqt/cmd_vel

import rospy
import math
from eufs_msgs.msg import driveCommand2
from geometry_msgs.msg import Twist

class ConvertTwistToDriveCommand2:
    def __init__(self):
        self.publisher = rospy.Publisher('/rqt/command', driveCommand2, queue_size=10)
        self.max_steering = 1
        self.min_steering = -1
        self.epsilon_steering = math.radians(0.001)

    def callback(self, data):
        out_cmd = driveCommand2()

        out_cmd.header.stamp = rospy.Time.now()

        out_cmd.front_throttle = data.linear.x
        out_cmd.rear_throttle = data.linear.x
        out_cmd.steering = data.angular.z

        out_cmd.sender = 'rqt'

        # the torque car's steering is flipped relative to the speed car
        # so to keep things consistent we flip the steering here:
	out_cmd.steering = -out_cmd.steering

        # impose limits on commanded angle
        if out_cmd.steering > self.max_steering:
            out_cmd.steering = self.max_steering
        if out_cmd.steering < self.min_steering:
            out_cmd.steering = self.min_steering

        # clean up angle if it is very close to zero
        if math.fabs(out_cmd.steering) < self.epsilon_steering:
            out_cmd.steering = 0.0

        self.publisher.publish(out_cmd)

    def listener(self):
	#Sometimes it's weird and the controller wants it to subscribe to /rqt/cmd_vel ???  It flipflops
        rospy.Subscriber("/cmd_vel", Twist, self.callback)
        rospy.Subscriber("/rqt/cmd_vel", Twist, self.callback)
        rospy.spin()

if __name__ == '__main__':
    try:
        rospy.init_node("twistToDriveCommand2Node", anonymous=True)
        cnv = ConvertTwistToDriveCommand2()
        cnv.listener()
    except rospy.ROSInterruptException: pass
