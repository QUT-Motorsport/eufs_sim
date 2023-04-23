// Main Include
#include "gazebo_sbg_plugin/gazebo_ros_sbg_driver.hpp"

// STD Include
#include <algorithm>
#include <fstream>
#include <mutex>   // NOLINT(build/c++11)
#include <thread>  // NOLINT(build/c++11)

namespace gazebo_plugins {
namespace eufs_plugins {

GZ_REGISTER_MODEL_PLUGIN(SBGPlugin)

SBGPlugin::SBGPlugin() {}

void SBGPlugin::Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf)
{
    _ros_node = gazebo_ros::Node::Get(sdf);

    RCLCPP_DEBUG(_ros_node->get_logger(), "Loading SBGPlugin");

    _model = model;
    _world = _model->GetWorld();

    _ekf_update_rate = get_double_parameter(sdf, "ekfUpdateRate", 0, "0.0 (as fast as possible)", _ros_node->get_logger());
    _vel_update_rate = get_double_parameter(sdf, "velUpdateRate", 0, "0.0 (as fast as possible)", _ros_node->get_logger());
    _gps_update_rate = get_double_parameter(sdf, "gpsUpdateRate", 0, "0.0 (as fast as possible)", _ros_node->get_logger());
    _pub_gt = get_bool_parameter(sdf, "publishGroundTruth", false, "false", _ros_node->get_logger());

    _reference_frame = get_string_parameter(sdf, "referenceFrame", "", "empty", _ros_node->get_logger());
    _robot_frame = get_string_parameter(sdf, "robotFrame", "", "empty", _ros_node->get_logger());

    // Create noise object
    std::string yaml_name = get_string_parameter(sdf, "noise_config", "", "empty", _ros_node->get_logger());
    _noise = std::make_unique<eufs::models::Noise>(yaml_name);

    // ROS publishers
    _pub_velocity = _ros_node->create_publisher<geometry_msgs::msg::TwistStamped>("imu/velocity", 1);
    _pub_euler = _ros_node->create_publisher<sbg_driver::msg::SbgEkfEuler>("sbg/ekf_euler", 1);
    _pub_gps = _ros_node->create_publisher<sbg_driver::msg::SbgGpsPos>("sbg/gps_pos", 1);
    _pub_odom = _ros_node->create_publisher<nav_msgs::msg::Odometry>("ground_truth/odom", 1);

    // ROS subscribers
    _sub_nav_sat_fix = _ros_node->create_subscription<sensor_msgs::msg::NavSatFix>(
        "imu/nav_sat_fix", 1, std::bind(&SBGPlugin::navSatFixCallback, this, std::placeholders::_1));

    // Connect to Gazebo
    _update_connection = gazebo::event::Events::ConnectWorldUpdateBegin(std::bind(&SBGPlugin::update, this));
    _last_vel_time = _world->SimTime();
    _last_ekf_time = _world->SimTime();
    _last_gps_time = _world->SimTime();
    _last_odom_time = _world->SimTime();

    _offset = _model->WorldPose();

    RCLCPP_INFO(_ros_node->get_logger(), "SBGPlugin Loaded");
}

void SBGPlugin::navSatFixCallback(const sensor_msgs::msg::NavSatFix::SharedPtr nav_sat_msg)
{
    _last_nav_sat_msg = *nav_sat_msg;
    
    sbg_driver::msg::SbgGpsPos msg;
    msg.header.stamp.sec = _last_nav_sat_msg.header.stamp.sec;
    msg.header.stamp.nanosec = _last_nav_sat_msg.header.stamp.nanosec;
    msg.header.frame_id = _reference_frame;

    msg.latitude = _last_nav_sat_msg.latitude;
    msg.longitude = _last_nav_sat_msg.longitude;
    msg.altitude = _last_nav_sat_msg.altitude;

    _pub_gps->publish(msg);
}

void SBGPlugin::update()
{
    _pose = _model->WorldPose();
    _vel = _model->WorldLinearVel();
    _angular_vel = _model->WorldAngularVel();

    publishVelocity();
    publishEuler();
    // publishGps();
    publishOdom();
}


void SBGPlugin::publishVelocity()
{
    auto curr_time = _world->SimTime();
    if (calc_dt(_last_vel_time, curr_time) < (1.0 / _vel_update_rate)) {
        return;
    }
    _last_vel_time = curr_time;

    geometry_msgs::msg::TwistStamped msg;
    msg.header.stamp.sec = curr_time.sec;
    msg.header.stamp.nanosec = curr_time.nsec;
    msg.header.frame_id = _reference_frame;

    msg.twist.linear.x = _vel.X();
    msg.twist.linear.y = _vel.Y();
    msg.twist.linear.z = _vel.Z();

    msg.twist.angular.x = _angular_vel.X();
    msg.twist.angular.y = _angular_vel.Y();
    msg.twist.angular.z = _angular_vel.Z();

    _pub_velocity->publish(msg);
}

void SBGPlugin::publishEuler()
{
    auto curr_time = _world->SimTime();
    if (calc_dt(_last_ekf_time, curr_time) < (1.0 / _ekf_update_rate)) {
        return;
    }
    _last_ekf_time = curr_time;

    sbg_driver::msg::SbgEkfEuler msg;
    msg.header.stamp.sec = curr_time.sec;
    msg.header.stamp.nanosec = curr_time.nsec;
    msg.header.frame_id = _reference_frame;

    msg.angle.x = _pose.Roll() - _offset.Roll();
    msg.angle.y = _pose.Pitch() - _offset.Pitch();
    msg.angle.z = _pose.Yaw() - _offset.Yaw();

    _pub_euler->publish(msg);
}

// void SBGPlugin::publishGps()
// {
//     auto curr_time = _world->SimTime();
//     if (calc_dt(_last_gps_time, curr_time) < (1.0 / _gps_update_rate)) {
//         return;
//     }
//     _last_gps_time = curr_time;

//     sbg_driver::msg::SbgGpsPos msg;
//     msg.header.stamp.sec = _last_nav_sat_msg.header.stamp.sec;
//     msg.header.stamp.nanosec = _last_nav_sat_msg.header.stamp.nanosec;
//     msg.header.frame_id = _reference_frame;

//     msg.latitude = _last_nav_sat_msg.latitude;
//     msg.longitude = _last_nav_sat_msg.longitude;
//     msg.altitude = _last_nav_sat_msg.altitude;

//     _pub_gps->publish(msg);
// }

void SBGPlugin::publishOdom()
{
    if (_pub_odom->get_subscription_count() == 0 || !_pub_gt) {
        return;
    }

    auto curr_time = _world->SimTime();
    if (calc_dt(_last_odom_time, curr_time) < (1.0 / _ekf_update_rate)) {
        return;
    }
    _last_odom_time = curr_time;

    nav_msgs::msg::Odometry msg;
    msg.header.stamp.sec = curr_time.sec;
    msg.header.stamp.nanosec = curr_time.nsec;
    msg.header.frame_id = _reference_frame;
    msg.child_frame_id = _robot_frame;

    msg.pose.pose.position.x = _pose.Pos().X() - _offset.Pos().X();
    msg.pose.pose.position.y = _pose.Pos().Y() - _offset.Pos().Y();
    msg.pose.pose.position.z = _pose.Pos().Z() - _offset.Pos().Z();

    std::vector<double> orientation = {_pose.Yaw() - _offset.Yaw(), 0.0, 0.0};
    orientation = ToQuaternion(orientation);

    msg.pose.pose.orientation.x = orientation[0];
    msg.pose.pose.orientation.y = orientation[1];
    msg.pose.pose.orientation.z = orientation[2];
    msg.pose.pose.orientation.w = orientation[3];

    msg.twist.twist.linear.x = _vel.X();
    msg.twist.twist.linear.y = _vel.Y();
    msg.twist.twist.linear.z = _vel.Z();

    msg.twist.twist.angular.x = _angular_vel.X();
    msg.twist.twist.angular.y = _angular_vel.Y();
    msg.twist.twist.angular.z = _angular_vel.Z();

    _pub_odom->publish(msg);
}

std::vector<double> SBGPlugin::ToQuaternion(std::vector<double> &euler) {
    // Abbreviations for the various angular functions
    double cy = cos(euler[0] * 0.5);
    double sy = sin(euler[0] * 0.5);
    double cp = cos(euler[1] * 0.5);
    double sp = sin(euler[1] * 0.5);
    double cr = cos(euler[2] * 0.5);
    double sr = sin(euler[2] * 0.5);

    std::vector<double> q;
    q.push_back(cy * cp * sr - sy * sp * cr);  // x
    q.push_back(sy * cp * sr + cy * sp * cr);  // y
    q.push_back(sy * cp * cr - cy * sp * sr);  // z
    q.push_back(cy * cp * cr + sy * sp * sr);  // w

    return q;
}
}
}