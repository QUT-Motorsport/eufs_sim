// Main Include
#include "gazebo_sbg_plugin/gazebo_ros_sbg_driver.hpp"

// STD Include
#include <algorithm>
#include <fstream>
#include <mutex>   // NOLINT(build/c++11)
#include <thread>  // NOLINT(build/c++11)

namespace gazebo_plugins {
namespace eufs_plugins {

SBGPlugin::SBGPlugin() {}

void SBGPlugin::Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) {
    _ros_node = gazebo_ros::Node::Get(sdf);

    RCLCPP_DEBUG(_ros_node->get_logger(), "Loading SBGPlugin");

    _model = model;
    _world = _model->GetWorld();

    // Initialize parameters
    initParams();

    // ROS publishers
    _pub_velocity = _ros_node->create_publisher<geometry_msgs::msg::TwistStamped>("imu/velocity", 1);
    _pub_euler = _ros_node->create_publisher<sbg_driver::msg::SbgEkfEuler>("sbg/ekf_euler", 1);
    _pub_gps = _ros_node->create_publisher<sbg_driver::msg::SbgGpsPos>("sbg/gps_pos", 1);

    // ROS subscribers
    _sub_nav_sat_fix = _ros_node->create_subscription<sensor_msgs::msg::NavSatFix>(
        "imu/nav_sat_fix", 1, std::bind(&SBGPlugin::navSatFixCallback, this, std::placeholders::_1));

    // Connect to Gazebo
    _update_connection = gazebo::event::Events::ConnectWorldUpdateBegin(std::bind(&SBGPlugin::update, this));
    _last_vel_time = _world->SimTime();
    _last_ekf_time = _world->SimTime();
    _last_gps_time = _world->SimTime();

    _offset = _model->WorldPose();

    RCLCPP_INFO(_ros_node->get_logger(), "SBGPlugin Loaded");
}

void SBGPlugin::initParams() {
    // Get parameters
    _ekf_update_rate = _ros_node->declare_parameter("ekf_update_rate", 1.0);
    _vel_update_rate = _ros_node->declare_parameter("vel_update_rate", 1.0);
    _gps_update_rate = _ros_node->declare_parameter("gps_update_rate", 1.0);
    _map_frame = _ros_node->declare_parameter("map_frame", "map");
    _odom_frame = _ros_node->declare_parameter("odom_frame", "odom");
    _base_frame = _ros_node->declare_parameter("base_frame", "base_link");

    // Noise
    std::string noise_yaml_name = _ros_node->declare_parameter("noise_config", "null");
    if (noise_yaml_name == "null") {
        RCLCPP_FATAL(_ros_node->get_logger(), "gazebo_ros_race_car plugin missing <noise_config>, cannot proceed");
        return;
    }
    _noise = std::make_unique<eufs::models::Noise>(noise_yaml_name);
}

void SBGPlugin::navSatFixCallback(const sensor_msgs::msg::NavSatFix::SharedPtr nav_sat_msg) {
    _last_nav_sat_msg = *nav_sat_msg;

    sbg_driver::msg::SbgGpsPos msg;
    msg.header.stamp.sec = _last_nav_sat_msg.header.stamp.sec;
    msg.header.stamp.nanosec = _last_nav_sat_msg.header.stamp.nanosec;
    msg.header.frame_id = _odom_frame;

    msg.latitude = _last_nav_sat_msg.latitude;
    msg.longitude = _last_nav_sat_msg.longitude;
    msg.altitude = _last_nav_sat_msg.altitude;

    if (has_subscribers(_pub_gps)) {
        _pub_gps->publish(msg);
    }
}

void SBGPlugin::update() {
    _pose = _model->WorldPose();
    _vel = _model->WorldLinearVel();
    _angular_vel = _model->WorldAngularVel();

    publishVelocity();
    publishEuler();
}

void SBGPlugin::publishVelocity() {
    auto curr_time = _world->SimTime();
    if (calc_dt(_last_vel_time, curr_time) < (1.0 / _vel_update_rate)) {
        return;
    }
    _last_vel_time = curr_time;

    geometry_msgs::msg::TwistStamped msg;
    msg.header.stamp.sec = curr_time.sec;
    msg.header.stamp.nanosec = curr_time.nsec;
    msg.header.frame_id = _map_frame;

    geometry_msgs::msg::Twist twist_vals;
    twist_vals.linear.x = _vel.X();
    twist_vals.linear.y = _vel.Y();
    twist_vals.linear.z = _vel.Z();

    twist_vals.angular.x = _angular_vel.X();
    twist_vals.angular.y = _angular_vel.Y();
    twist_vals.angular.z = _angular_vel.Z();

    msg.twist = _noise->applyNoiseToTwist(twist_vals);

    if (has_subscribers(_pub_velocity)) {
        _pub_velocity->publish(msg);
    }
}

void SBGPlugin::publishEuler() {
    auto curr_time = _world->SimTime();
    if (calc_dt(_last_ekf_time, curr_time) < (1.0 / _ekf_update_rate)) {
        return;
    }
    _last_ekf_time = curr_time;

    sbg_driver::msg::SbgEkfEuler msg;
    msg.header.stamp.sec = curr_time.sec;
    msg.header.stamp.nanosec = curr_time.nsec;
    msg.header.frame_id = _map_frame;

    geometry_msgs::msg::Vector3 euler_vals;
    euler_vals.x = _pose.Roll() - _offset.Roll();
    euler_vals.y = _pose.Pitch() - _offset.Pitch();
    euler_vals.z = _pose.Yaw() - _offset.Yaw();

    msg.angle = _noise->applyNoiseToVector(euler_vals);

    if (has_subscribers(_pub_euler)) {
        _pub_euler->publish(msg);
    }
}
}  // namespace eufs_plugins
}  // namespace gazebo_plugins
