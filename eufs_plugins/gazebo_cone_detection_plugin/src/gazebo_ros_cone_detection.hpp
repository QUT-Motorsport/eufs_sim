#pragma once

#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
#include <gazebo_ros/node.hpp>

// ROS  srvs
#include <std_srvs/srv/trigger.hpp>

// QUTMS messages
#include <driverless_msgs/msg/cone.hpp>
#include <driverless_msgs/msg/cone_detection_stamped.hpp>

#include "helpers_gazebo.hpp"
#include "helpers_track.hpp"
#include "helpers_ros.hpp"
#include "helpers_detection.hpp"

namespace gazebo_plugins {
namespace eufs_plugins {

class ConeDetectionPlugin : public gazebo::ModelPlugin {
   public:
    ConeDetectionPlugin();

    // Gazebo plugin functions
    void Load(gazebo::physics::ModelPtr parent, sdf::ElementPtr sdf);
    void UpdateChild();

  private:
    gazebo::physics::WorldPtr world;
    gazebo_ros::Node::SharedPtr ros_node;
    gazebo::physics::ModelPtr track_model;
    gazebo::event::ConnectionPtr update_connection;

    double update_rate;
    gazebo::common::Time last_update;

    rclcpp::Publisher<driverless_msgs::msg::ConeDetectionStamped>::SharedPtr ground_truth_pub;
    rclcpp::Publisher<driverless_msgs::msg::ConeDetectionStamped>::SharedPtr lidar_detection_pub;
    rclcpp::Publisher<driverless_msgs::msg::ConeDetectionStamped>::SharedPtr vision_detection_pub;
    rclcpp::Publisher<driverless_msgs::msg::ConeDetectionStamped>::SharedPtr slam_global_pub;
    rclcpp::Publisher<driverless_msgs::msg::ConeDetectionStamped>::SharedPtr slam_local_pub;

    SensorConfig_t lidar_config;
    SensorConfig_t camera_config;
};

}  // namespace eufs_plugins
}  // namespace gazebo_plugins
