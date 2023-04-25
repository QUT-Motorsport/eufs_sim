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

#include "helpers_detection.hpp"
#include "helpers_gazebo.hpp"
#include "helpers_ros.hpp"
#include "helpers_track.hpp"

namespace gazebo_plugins {
namespace eufs_plugins {

class ConeDetectionPlugin : public gazebo::ModelPlugin {
   public:
    ConeDetectionPlugin();

    // Gazebo plugin functions
    void Load(gazebo::physics::ModelPtr parent, sdf::ElementPtr sdf);
    void update();
    bool resetConePosition(std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                           std::shared_ptr<std_srvs::srv::Trigger::Response> response);

   private:
    gazebo::physics::WorldPtr world;
    gazebo_ros::Node::SharedPtr _ros_node;
    gazebo::event::ConnectionPtr update_connection;

    gazebo::physics::ModelPtr track_model;
    gazebo::physics::LinkPtr car_link;
    ignition::math::Pose3d car_inital_pose;

    double update_rate;
    bool publish_ground_truth;
    bool simulate_perception;
    bool simulate_SLAM;
    gazebo::common::Time last_update;
    driverless_msgs::msg::ConeDetectionStamped initial_track;

    rclcpp::Publisher<driverless_msgs::msg::ConeDetectionStamped>::SharedPtr ground_truth_pub;
    rclcpp::Publisher<driverless_msgs::msg::ConeDetectionStamped>::SharedPtr lidar_detection_pub;
    rclcpp::Publisher<driverless_msgs::msg::ConeDetectionStamped>::SharedPtr vision_detection_pub;
    rclcpp::Publisher<driverless_msgs::msg::ConeDetectionStamped>::SharedPtr slam_global_pub;
    rclcpp::Publisher<driverless_msgs::msg::ConeDetectionStamped>::SharedPtr slam_local_pub;
    // ROS Service
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_cone_pos_srv;

    SensorConfig_t lidar_config;
    SensorConfig_t camera_config;
    SLAMConfig_t slam_config;
};

}  // namespace eufs_plugins
}  // namespace gazebo_plugins
