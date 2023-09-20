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
    void initParams();
    void publishGTTrack();
    void publishLiDARDetection();
    void publishCameraDetection();
    void publishSLAM();
    bool resetConePosition(std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                           std::shared_ptr<std_srvs::srv::Trigger::Response> response);

   private:
    gazebo::physics::WorldPtr _world;
    gazebo::physics::ModelPtr _model;
    gazebo_ros::Node::SharedPtr _ros_node;
    gazebo::event::ConnectionPtr _update_connection;

    std::string _map_frame;
    std::string _base_frame;

    bool _pub_gt;
    bool _simulate_perception;
    bool _simulate_slam;

    gazebo::physics::ModelPtr _track_model;
    gazebo::physics::LinkPtr _car_link;
    ignition::math::Pose3d _car_inital_pose;

    double _gt_update_rate;
    double _lidar_update_rate;
    double _camera_update_rate;
    double _slam_update_rate;
    gazebo::common::Time _last_gt_update;
    gazebo::common::Time _last_lidar_update;
    gazebo::common::Time _last_camera_update;
    gazebo::common::Time _last_slam_update;
    driverless_msgs::msg::ConeDetectionStamped _initial_track;
    driverless_msgs::msg::ConeDetectionStamped _initial_slam;

    rclcpp::Publisher<driverless_msgs::msg::ConeDetectionStamped>::SharedPtr _ground_truth_pub;
    rclcpp::Publisher<driverless_msgs::msg::ConeDetectionStamped>::SharedPtr _lidar_detection_pub;
    rclcpp::Publisher<driverless_msgs::msg::ConeDetectionStamped>::SharedPtr _vision_detection_pub;
    rclcpp::Publisher<driverless_msgs::msg::ConeDetectionStamped>::SharedPtr _slam_global_pub;
    rclcpp::Publisher<driverless_msgs::msg::ConeDetectionStamped>::SharedPtr _slam_local_pub;
    // ROS Service
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_cone_pos_srv;

    SensorConfig_t _lidar_config;
    SensorConfig_t _camera_config;
    SLAMConfig_t _slam_config;
};

}  // namespace eufs_plugins
}  // namespace gazebo_plugins
