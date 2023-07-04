/*
 * AMZ-Driverless
 * Copyright (c) 2018 Authors:
 *   - Juraj Kabzan <kabzanj@gmail.com>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#ifndef EUFS_PLUGINS_GAZEBO_RACE_CAR_PLUGIN_INCLUDE_GAZEBO_RACE_CAR_PLUGIN_GAZEBO_ROS_RACE_CAR_HPP_
#define EUFS_PLUGINS_GAZEBO_RACE_CAR_PLUGIN_INCLUDE_GAZEBO_RACE_CAR_PLUGIN_GAZEBO_ROS_RACE_CAR_HPP_

#include <memory>
#include <queue>
#include <string>
#include <vector>
// ROS Includes
#include "rclcpp/rclcpp.hpp"

// ROS msgs
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/float32.hpp"

// ROS TF2
#include <tf2/transform_datatypes.h>
#include <tf2/utils.h>
#include <tf2_ros/transform_broadcaster.h>

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

// ROS  srvs
#include <std_srvs/srv/trigger.hpp>

// Gazebo Includes
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo_ros/node.hpp>

// Local includes
#include "eufs_models/eufs_models.hpp"
#include "gazebo_race_car_plugin/state_machine.hpp"
#include "helpers_gazebo.hpp"
#include "helpers_ros.hpp"

namespace gazebo_plugins {
namespace eufs_plugins {

class RaceCarPlugin : public gazebo::ModelPlugin {
   public:
    RaceCarPlugin();

    ~RaceCarPlugin() override;

    void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override;

    eufs::models::State &getState() { return _state; }
    eufs::models::Input &getInput() { return _des_input; }

   private:
    void update();

    void setPositionFromWorld();
    bool resetVehiclePosition(std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                              std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    void returnCommandMode(std::shared_ptr<std_srvs::srv::Trigger::Request>,
                           std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    void setModelState();

    void initParams(const sdf::ElementPtr &sdf);

    geometry_msgs::msg::PoseWithCovarianceStamped stateToPoseMsg(const eufs::models::State &state);
    nav_msgs::msg::Odometry getWheelOdometry(const std::vector<double> &speeds, const double &input);

    void publishCarPose();
    void publishVehicleOdom();
    void publishTf();

    void onCmd(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg);

    std::vector<double> ToQuaternion(std::vector<double> &euler);

    std::shared_ptr<rclcpp::Node> _rosnode;
    eufs::models::VehicleModelPtr _vehicle;

    // States
    std::unique_ptr<StateMachine> _state_machine;
    eufs::models::State _state;
    eufs::models::Input _des_input, _act_input;
    std::unique_ptr<eufs::models::Noise> _noise;
    ignition::math::Pose3d _offset;

    // Gazebo
    gazebo::physics::WorldPtr _world;
    gazebo::physics::ModelPtr _model;
    gazebo::event::ConnectionPtr _update_connection;
    gazebo::common::Time _last_sim_time, _last_cmd_time;

    // Rate to publish ros messages
    double _update_rate;
    double _publish_rate;
    gazebo::common::Time _time_last_published;

    // ROS TF
    bool _pub_tf;
    std::string _reference_frame;
    std::string _robot_frame;
    std::unique_ptr<tf2_ros::TransformBroadcaster> _tf_br;

    // ROS Publishers
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr _pub_wheel_odom;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr _pub_gt_wheel_odom;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr _pub_pose;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr _pub_gt_pose;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr _pub_steering_angle;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr _pub_gt_steering_angle;

    // ROS Subscriptions
    rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr _sub_cmd;

    // ROS Services
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr _reset_vehicle_pos_srv;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr _command_mode_srv;

    // Steering joints state
    gazebo::physics::JointPtr _left_steering_joint;
    gazebo::physics::JointPtr _right_steering_joint;

    // Ground truth
    bool _pub_gt;

    // SLAM Pose
    bool _simulate_slam;

    enum CommandMode { acceleration, velocity };
    CommandMode _command_mode;

    // Command queue for control delays
    ackermann_msgs::msg::AckermannDriveStamped _last_cmd;
    double _control_delay;
    // Steering rate limit variables
    double _max_steering_rate, _steering_lock_time;
    int counter = 0;
};

}  // namespace eufs_plugins
}  // namespace gazebo_plugins

#endif  // EUFS_PLUGINS_GAZEBO_RACE_CAR_PLUGIN_INCLUDE_GAZEBO_RACE_CAR_PLUGIN_GAZEBO_ROS_RACE_CAR_HPP_
