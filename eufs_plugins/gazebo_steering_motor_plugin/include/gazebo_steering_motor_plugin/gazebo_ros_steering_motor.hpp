#ifndef EUFS_PLUGINS_GAZEBO_STEERING_MOTOR_PLUGIN_INCLUDE_GAZEBO_STEERING_MOTOR_PLUGIN_GAZEBO_ROS_STEERING_MOTOR_HPP_
#define EUFS_PLUGINS_GAZEBO_STEERING_MOTOR_PLUGIN_INCLUDE_GAZEBO_STEERING_MOTOR_PLUGIN_GAZEBO_ROS_STEERING_MOTOR_HPP_

#include <memory>
#include <queue>
#include <string>
#include <vector>
// ROS Includes
#include "rclcpp/rclcpp.hpp"

// ROS msgs
#include "driverless_msgs/msg/can.hpp"
#include "std_msgs/msg/float32.hpp"

// Gazebo Includes
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo_ros/node.hpp>

// Local includes
#include "helpers_gazebo.hpp"
#include "helpers_ros.hpp"
#include "gazebo_steering_motor_plugin/helpers_steering.hpp"

namespace gazebo_plugins {
namespace eufs_plugins {

class SteeringMotorPlugin : public gazebo::ModelPlugin {
   public:
    SteeringMotorPlugin();

    ~SteeringMotorPlugin() override;

    void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override;

   private:
    void update();
    void can_rx_callback(const driverless_msgs::msg::Can::SharedPtr msg);
    void update_state();

    std::shared_ptr<rclcpp::Node> _rosnode;

    // Gazebo
    gazebo::physics::WorldPtr _world;
    gazebo::physics::ModelPtr _model;
    gazebo::event::ConnectionPtr _update_connection;
    gazebo::common::Time _last_sim_time;

    gazebo::physics::JointPtr _left_steering_joint;
    gazebo::physics::JointPtr _right_steering_joint;

    // Rate to publish ros messages
    double _update_rate = 100;

    // states
    c5e_state _desired_state = states[SOD];
    c5e_state _current_state = states[NRTSO];
    uint32_t _home_offset = 0;
    uint16_t _motion_profile_type = 0;
    uint32_t _profile_velocity = 200;
    uint32_t _end_velocity = 0;
    uint32_t _profile_acceleration = 50;
    uint32_t _profile_deceleration = 50;
    uint32_t _quick_stop_deceleration = 50;
    uint32_t _max_acceleration = 50;
    uint32_t _max_deceleration = 50;
    uint16_t _mode_of_operation = 1;
    uint32_t _target_position = 0;
    uint32_t _position_actual_value = 0;
    uint32_t _last_position_actual_value;
    bool _moving = false;

    // ROS Subscribers
    rclcpp::Subscription<driverless_msgs::msg::Can>::SharedPtr _sub_can;
    // ROS Publishers
    rclcpp::Publisher<driverless_msgs::msg::Can>::SharedPtr _pub_can;

    int counter = 0;
};

}  // namespace eufs_plugins
}  // namespace gazebo_plugins

#endif  // EUFS_PLUGINS_GAZEBO_STEERING_MOTOR_PLUGIN_INCLUDE_GAZEBO_STEERING_MOTOR_PLUGIN_GAZEBO_ROS_STEERING_MOTOR_HPP_
