#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

# Import your custom message type
from driverless_msgs.msg import ConeDetectionStamped
# Import standard visualization message types
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA

class ConeDetectionConverter(Node):
    def __init__(self):
        super().__init__('cone_detection_converter')
        # Subscribe to the custom ground truth track topic
        self.subscription = self.create_subscription(
            ConeDetectionStamped,
            '/ground_truth/global_map',  # Topic where your plugin publishes its custom message
            self.custom_msg_callback,
            10)
        # Publisher for the standard MarkerArray message
        self.publisher = self.create_publisher(MarkerArray, '/converted/cone_markers', 10)
        self.get_logger().info("Cone Detection Converter node has been started.")

    def custom_msg_callback(self, msg):
        """
        Callback for processing ConeDetectionStamped messages.
        Converts the custom message into a MarkerArray.
        """
        marker_array = MarkerArray()
        marker = Marker()
        marker.header = msg.header
        marker.ns = "cones"
        marker.id = 0
        marker.type = Marker.SPHERE_LIST  # Use a sphere list for simplicity
        marker.action = Marker.ADD
        # Set the scale (size) of each sphere (cone)
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        # Set a color (here an orange color for cones)
        marker.color = ColorRGBA(r=1.0, g=0.5, b=0.0, a=1.0)

        # Assuming msg.cones is a list of Cone messages that each contain a field 'location'
        # of type geometry_msgs/msg/Point, add each cone's position to the marker.
        for cone in msg.cones:
            marker.points.append(cone.location)

        marker_array.markers.append(marker)
        # Publish the MarkerArray
        self.publisher.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    node = ConeDetectionConverter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
