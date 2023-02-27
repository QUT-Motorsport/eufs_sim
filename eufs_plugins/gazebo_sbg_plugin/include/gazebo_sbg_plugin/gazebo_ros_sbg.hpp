#ifndef EUFS_PLUGINS_GAZEBO_RACE_CAR_PLUGIN_INCLUDE_GAZEBO_RACE_CAR_PLUGIN_GAZEBO_ROS_RACE_CAR_HPP_
#define EUFS_PLUGINS_GAZEBO_RACE_CAR_PLUGIN_INCLUDE_GAZEBO_RACE_CAR_PLUGIN_GAZEBO_ROS_RACE_CAR_HPP_

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
#include "geometry_msgs/msg/pose_with_covariance.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/twist_with_covariance.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "nav_msgs/msg/odometry.hpp"

// SBG ROS msgs
#include "sbg_driver/msg/sbg_ekf_euler.hpp"

// ROS TF2
#include <tf2/transform_datatypes.h>
#include <tf2/utils.h>
#include <tf2_ros/transform_broadcaster.h>

#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

// Gazebo Includes
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo_ros/node.hpp>


namespace gazebo_plugins {
namespace eufs_plugins {

class SBGPlugin : public gazebo::SensorPlugin
{
   public:
    SBGPlugin();

    void Load(gazebo::sensors::SensorPtr sensor, sdf::ElementPtr sdf) override;

    void getIMUData();

    void getGPSData();
    
    void update();

    gazebo_ros::Node::SharedPtr _ros_node;

    // Gazebo
    gazebo::physics::WorldPtr _world;
    gazebo::sensors::ImuSensorPtr _imu_sensor;
    gazebo::sensors::GpsSensorPtr _gps_sensor;

    // Rate to publish ros messages
    double _update_rate;
    double _publish_rate;
    gazebo::common::Time _time_last_published;

    // ROS Publishers
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr _pose_pub;


}
}  // namespace eufs_plugins
}  // namespace gazebo_plugins

#endif  // EUFS_PLUGINS_GAZEBO_SBG_PLUGIN_INCLUDE_GAZEBO_RACE_CAR_PLUGIN_GAZEBO_ROS_RACE_CAR_HPP_