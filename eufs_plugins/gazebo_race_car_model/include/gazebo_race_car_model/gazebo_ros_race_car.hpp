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

#ifndef EUFS_PLUGINS_GAZEBO_RACE_CAR_MODEL_INCLUDE_GAZEBO_RACE_CAR_MODEL_GAZEBO_ROS_RACE_CAR_HPP_
#define EUFS_PLUGINS_GAZEBO_RACE_CAR_MODEL_INCLUDE_GAZEBO_RACE_CAR_MODEL_GAZEBO_ROS_RACE_CAR_HPP_

#include <memory>
#include <queue>
#include <string>
#include <vector>
// ROS Includes
#include "rclcpp/rclcpp.hpp"

// ROS msgs
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "eufs_msgs/msg/car_state.hpp"
#include "eufs_msgs/msg/wheel_speeds_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/twist_with_covariance.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "nav_msgs/msg/odometry.hpp"

// ROS TF2
#include <tf2/transform_datatypes.h>
#include <tf2/utils.h>
#include <tf2_ros/transform_broadcaster.h>

// ROS  srvs
#include <std_srvs/srv/trigger.hpp>

// Gazebo Includes
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo_ros/node.hpp>

// EUFS includes
#include "eufs_models/eufs_models.hpp"
#include "gazebo_race_car_model/state_machine.hpp"

namespace gazebo_plugins {
namespace eufs_plugins {

class RaceCarModelPlugin : public gazebo::ModelPlugin {
 public:
  RaceCarModelPlugin();

  ~RaceCarModelPlugin() override;

  void Reset() override;
  void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override;

  eufs::models::State &getState() { return _state; }
  eufs::models::Input &getInput() { return _des_input; }

 private:
  void update();
  void updateState(double dt);

  void setPositionFromWorld();
  bool resetVehiclePosition(std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                            std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void returnCommandMode(std::shared_ptr<std_srvs::srv::Trigger::Request>,
                         std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void setModelState();

  void initVehicleModel(const sdf::ElementPtr &sdf);
  void initParams(const sdf::ElementPtr &sdf);
  void initModel(const sdf::ElementPtr &sdf);
  void initNoise(const sdf::ElementPtr &sdf);

  eufs_msgs::msg::CarState stateToCarStateMsg(const eufs::models::State &state);

  void publishCarState();
  void publishWheelSpeeds();
  void publishOdom();
  void publishTf();

  void onCmd(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg);

  /// @brief Converts an euler orientation to quaternion
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
  bool _publish_tf;
  std::string _reference_frame;
  std::string _robot_frame;
  std::unique_ptr<tf2_ros::TransformBroadcaster> _tf_br;

  // ROS topic parameters
  std::string _ground_truth_car_state_topic;
  std::string _localisation_car_state_topic;
  std::string _wheel_speeds_topic_name;
  std::string _ground_truth_wheel_speeds_topic_name;
  std::string _odom_topic_name;

  // ROS Publishers
  rclcpp::Publisher<eufs_msgs::msg::CarState>::SharedPtr _pub_ground_truth_car_state;
  rclcpp::Publisher<eufs_msgs::msg::CarState>::SharedPtr _pub_localisation_car_state;
  rclcpp::Publisher<eufs_msgs::msg::WheelSpeedsStamped>::SharedPtr _pub_wheel_speeds;
  rclcpp::Publisher<eufs_msgs::msg::WheelSpeedsStamped>::SharedPtr _pub_ground_truth_wheel_speeds;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr _pub_odom;

  // ROS Subscriptions
  rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr _sub_cmd;

  // ROS Services
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr _reset_vehicle_pos_srv;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr _command_mode_srv;

  // Steering joints state
  gazebo::physics::JointPtr _left_steering_joint;
  gazebo::physics::JointPtr _right_steering_joint;

  // Stop Ground truth
  bool _pub_ground_truth;

  enum CommandMode { acceleration, velocity };
  CommandMode _command_mode;

  // Command queue for control delays
  std::queue<std::shared_ptr<ackermann_msgs::msg::AckermannDriveStamped>> _command_Q;
  std::queue<gazebo::common::Time> _cmd_time_Q;
  double _control_delay;
  // Steering rate limit variables
  double _max_steering_rate, _steering_lock_time;
};

}  // namespace eufs_plugins
}  // namespace gazebo_plugins

#endif  // EUFS_PLUGINS_GAZEBO_RACE_CAR_MODEL_INCLUDE_GAZEBO_RACE_CAR_MODEL_GAZEBO_ROS_RACE_CAR_HPP_
