#ifndef EUFS_PLUGINS_GAZEBO_SBG_PLUGIN_INCLUDE_GAZEBO_SBG_PLUGIN_GAZEBO_ROS_SBG_HPP_
#define EUFS_PLUGINS_GAZEBO_SBG_PLUGIN_INCLUDE_GAZEBO_SBG_PLUGIN_GAZEBO_ROS_SBG_HPP_

#include <memory>
#include <queue>
#include <string>
#include <vector>
#include <map>
#include <utility>
// ROS Includes
#include "rclcpp/rclcpp.hpp"

// ROS msgs
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"

// SBG ROS msgs
#include "sbg_driver/msg/sbg_ekf_euler.hpp"
#include "sbg_driver/msg/sbg_gps_pos.hpp"

// Gazebo Includes
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo_ros/node.hpp>

// Local Includes
#include "eufs_models/eufs_models.hpp"
#include "helpers_ros.hpp"
#include "helpers_gazebo.hpp"

namespace gazebo_plugins {
namespace eufs_plugins {

class SBGPlugin : public gazebo::ModelPlugin
{
   public:
    SBGPlugin();

    void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override;

   private:   
    // Gazebo functions
    void update();
    void navSatFixCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
    void publishVelocity();
    void publishEuler();
    // void publishGps();
    void publishGTOdom();
    std::vector<double> ToQuaternion(std::vector<double> &euler);

    gazebo_ros::Node::SharedPtr _ros_node;

    // Gazebo
    gazebo::physics::WorldPtr _world;
    gazebo::physics::ModelPtr _model;
    gazebo::event::ConnectionPtr _update_connection;

    // States
    ignition::math::Pose3d _offset;
    ignition::math::Pose3d _pose;
    ignition::math::Vector3d _vel;
    ignition::math::Vector3d _angular_vel;
    std::unique_ptr<eufs::models::Noise> _noise;

    // ROS variables
    std::string _reference_frame;
    std::string _robot_frame;
    sensor_msgs::msg::NavSatFix _last_nav_sat_msg;
    bool _pub_gt;

    // Rate to publish ros messages
    double _vel_update_rate;
    double _ekf_update_rate;
    double _gps_update_rate;
    gazebo::common::Time _last_vel_time;
    gazebo::common::Time _last_ekf_time;
    gazebo::common::Time _last_gps_time;
    gazebo::common::Time _last_odom_time;

    // ROS Publishers
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr _pub_velocity;
    rclcpp::Publisher<sbg_driver::msg::SbgEkfEuler>::SharedPtr _pub_euler;
    rclcpp::Publisher<sbg_driver::msg::SbgGpsPos>::SharedPtr _pub_gps;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr _pub_gt_odom;

    // ROS Subscribers
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr _sub_nav_sat_fix;

};

}  // namespace eufs_plugins
}  // namespace gazebo_plugins

#endif  // EUFS_PLUGINS_GAZEBO_SBG_PLUGIN_INCLUDE_GAZEBO_SBG_PLUGIN_GAZEBO_ROS_SBG_HPP_