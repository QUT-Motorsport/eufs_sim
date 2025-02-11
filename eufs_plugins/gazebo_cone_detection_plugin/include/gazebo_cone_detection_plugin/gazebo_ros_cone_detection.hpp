#pragma once

// Gazebo includes
// #include <gz/common/Plugin.hh>
// #include <gazebo/common/Time.hh>
// #include <gazebo/gazebo.hh>
// #include <gazebo/physics/Link.hh>
// #include <gazebo/physics/Model.hh>
// #include <gazebo/physics/World.hh>
// #include <gazebo_ros/node.hpp>
#include <gz/sim/System.hh>
#include <gz/sim/Entity.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/World.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/Link.hh>
#include <gz/transport/Node.hh>
// #include <gz/sim/UpdateInfo.hh>
#include <gz/sim/EventManager.hh>

// ROS Includes
#include <std_srvs/srv/trigger.hpp>
#include <rclcpp/rclcpp.hpp>

// QUTMS Driverless Includes
#include <driverless_msgs/msg/cone.hpp>
#include <driverless_msgs/msg/cone_detection_stamped.hpp>

// Eufs Sim includes
#include "helpers_detection.hpp"
#include "helpers_gazebo.hpp"
#include "helpers_ros.hpp"
#include "helpers_track.hpp"

namespace gazebo_plugins {
namespace eufs_plugins {

class ConeDetectionPlugin : public gz::sim::System,
                            public gz::sim::ISystemConfigure,
                            public gz::sim::ISystemPreUpdate
{
   public:
    ConeDetectionPlugin();
    ~ConeDetectionPlugin();

    // System cobfiguration (replaced Load)
    void Configure(const gz::sim::Entity &entity,
                   const std::shared_ptr<const sdf::Element> &sdf,
                   gz::sim::EntityComponentManager &ecm,
                   gz::sim::EventManager &eventMgr) override;

    // PreUpdate (replaced update)
    void PreUpdate(const gz::sim::UpdateInfo &info,
                   gz::sim::EntityComponentManager &ecm) override;

   private:
    void initParams(gz::sim::EntityComponentManager &ecm);
    void publishGTTrack(gz::sim::EntityComponentManager &ecm);
    void publishLiDARDetection(gz::sim::EntityComponentManager &ecm);
    void publishCameraDetection(gz::sim::EntityComponentManager &ecm);
    void publishSLAM(gz::sim::EntityComponentManager &ecm);
    bool resetConePosition(std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                           std::shared_ptr<std_srvs::srv::Trigger::Response> response,
                           gz::sim::EntityComponentManager &ecm);

   private:
    gz::sim::World _world;
    gz::sim::Model _model;
    gz::sim::Entity _entity;
    gz::sim::EntityComponentManager* _ecm;  // store a pointer so that helper functions can use it
    gz::sim::Model _track_model;
    gz::sim::Link _car_link;
    gz::math::Pose3d _car_inital_pose;  

    std::shared_ptr<rclcpp::Node> _ros_node;

    std::string _map_frame;
    std::string _base_frame;

    bool _pub_gt;
    bool _simulate_perception;
    bool _simulate_slam;

    double _gt_update_rate;
    double _lidar_update_rate;
    double _camera_update_rate;
    double _slam_update_rate;

    std::chrono::time_point<std::chrono::steady_clock> _last_gt_update;
    std::chrono::time_point<std::chrono::steady_clock> _last_lidar_update;
    std::chrono::time_point<std::chrono::steady_clock> _last_camera_update;
    std::chrono::time_point<std::chrono::steady_clock> _last_slam_update;

    driverless_msgs::msg::ConeDetectionStamped _initial_track;
    driverless_msgs::msg::ConeDetectionStamped _initial_slam;

    rclcpp::Publisher<driverless_msgs::msg::ConeDetectionStamped>::SharedPtr _ground_truth_pub;
    rclcpp::Publisher<driverless_msgs::msg::ConeDetectionStamped>::SharedPtr _lidar_detection_pub;
    rclcpp::Publisher<driverless_msgs::msg::ConeDetectionStamped>::SharedPtr _vision_detection_pub;
    rclcpp::Publisher<driverless_msgs::msg::ConeDetectionStamped>::SharedPtr _slam_global_pub;
    rclcpp::Publisher<driverless_msgs::msg::ConeDetectionStamped>::SharedPtr _slam_local_pub;

    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_cone_pos_srv;

    // gz::sim::Model _track_model;
    // gz::sim::Link _car_link;
    // ignition::math::Pose3d _car_inital_pose;
    
    SensorConfig_t _lidar_config;
    SensorConfig_t _camera_config;
    SLAMConfig_t _slam_config;
};

}  // namespace eufs_plugins
}  // namespace gazebo_plugins
