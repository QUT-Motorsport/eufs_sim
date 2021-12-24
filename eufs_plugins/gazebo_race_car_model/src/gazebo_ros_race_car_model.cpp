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

// Main Include
#include "gazebo_race_car_model/gazebo_ros_race_car.hpp"

// STD Include
#include <algorithm>
#include <fstream>
#include <mutex>  // NOLINT(build/c++11)
#include <thread>  // NOLINT(build/c++11)


namespace gazebo_plugins {
namespace eufs_plugins {

RaceCarModelPlugin::RaceCarModelPlugin() {}

RaceCarModelPlugin::~RaceCarModelPlugin() { _update_connection.reset(); }

void RaceCarModelPlugin::Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) {
  _rosnode = gazebo_ros::Node::Get(sdf);

  RCLCPP_DEBUG(_rosnode->get_logger(), "Loading RaceCarModelPlugin");

  _model = model;
  _world = _model->GetWorld();

  _tf_br = std::make_unique<tf2_ros::TransformBroadcaster>(_rosnode);
  _state_machine = std::make_unique<StateMachine>(_rosnode);

  // Initialize parameters
  initParams(sdf);

  // Initialize vehicle model
  initVehicleModel(sdf);

  // Initialize handles to Gazebo vehicle components
  initModel(sdf);

  // Initialize noise object
  initNoise(sdf);

  // ROS Publishers
  _pub_ground_truth_car_state =
      _rosnode->create_publisher<eufs_msgs::msg::CarState>(_ground_truth_car_state_topic, 1);
  _pub_localisation_car_state =
      _rosnode->create_publisher<eufs_msgs::msg::CarState>(_localisation_car_state_topic, 1);
  _pub_wheel_speeds =
      _rosnode->create_publisher<eufs_msgs::msg::WheelSpeedsStamped>(_wheel_speeds_topic_name, 1);
  _pub_ground_truth_wheel_speeds = _rosnode->create_publisher<eufs_msgs::msg::WheelSpeedsStamped>(
      _ground_truth_wheel_speeds_topic_name, 1);
  _pub_odom = _rosnode->create_publisher<nav_msgs::msg::Odometry>(_odom_topic_name, 1);

  // ROS Services
  _reset_vehicle_pos_srv = _rosnode->create_service<std_srvs::srv::Trigger>(
      "/ros_can/reset_vehicle_pos", std::bind(&RaceCarModelPlugin::resetVehiclePosition, this,
                                              std::placeholders::_1, std::placeholders::_2));
  _command_mode_srv = _rosnode->create_service<std_srvs::srv::Trigger>(
      "/race_car_model/command_mode", std::bind(&RaceCarModelPlugin::returnCommandMode, this,
                                                std::placeholders::_1, std::placeholders::_2));

  // ROS Subscriptions
  _sub_cmd = _rosnode->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
      "/cmd", 1, std::bind(&RaceCarModelPlugin::onCmd, this, std::placeholders::_1));

  // Connect to Gazebo
  _update_connection =
      gazebo::event::Events::ConnectWorldUpdateBegin(std::bind(&RaceCarModelPlugin::update, this));
  _last_sim_time = _world->SimTime();

  _max_steering_rate =
      (_vehicle->getParam().input_ranges.delta.max - _vehicle->getParam().input_ranges.delta.min) /
      _steering_lock_time;

  // Set offset
  setPositionFromWorld();

  RCLCPP_INFO(_rosnode->get_logger(), "RaceCarModelPlugin Loaded");
}

void RaceCarModelPlugin::initParams(const sdf::ElementPtr &sdf) {
  if (!sdf->HasElement("update_rate")) {
    _update_rate = 1000.0;
  } else {
    _update_rate = sdf->GetElement("update_rate")->Get<double>();
  }

  if (!sdf->HasElement("publish_rate")) {
    _publish_rate = 200.0;
  } else {
    _publish_rate = sdf->GetElement("publish_rate")->Get<double>();
  }

  if (!sdf->HasElement("referenceFrame")) {
    RCLCPP_DEBUG(_rosnode->get_logger(),
                 "gazebo_ros_race_car_model plugin missing <referenceFrame>, defaults to map");
    _reference_frame = "map";
  } else {
    _reference_frame = sdf->GetElement("referenceFrame")->Get<std::string>();
  }

  if (!sdf->HasElement("robotFrame")) {
    RCLCPP_DEBUG(
        _rosnode->get_logger(),
        "gazebo_ros_race_car_model plugin missing <robotFrame>, defaults to base_footprint");
    _robot_frame = "base_footprint";
  } else {
    _robot_frame = sdf->GetElement("robotFrame")->Get<std::string>();
  }

  if (!sdf->HasElement("publishTransform")) {
    RCLCPP_DEBUG(_rosnode->get_logger(),
                 "gazebo_ros_race_car_model plugin missing <publishTransform>, defaults to false");
    _publish_tf = false;
  } else {
    _publish_tf = sdf->GetElement("publishTransform")->Get<bool>();
  }

  if (!sdf->HasElement("wheelSpeedsTopicName")) {
    RCLCPP_FATAL(_rosnode->get_logger(),
                 "gazebo_ros_race_car_model plugin missing <wheelSpeedsTopicName>, cannot proceed");
    return;
  } else {
    _wheel_speeds_topic_name = sdf->GetElement("wheelSpeedsTopicName")->Get<std::string>();
  }

  if (!sdf->HasElement("groundTruthWheelSpeedsTopicName")) {
    RCLCPP_FATAL(_rosnode->get_logger(),
                 "gazebo_ros_race_car_model plugin missing <groundTruthWheelSpeedsTopicName>, "
                 "cannot proceed");
    return;
  } else {
    _ground_truth_wheel_speeds_topic_name =
        sdf->GetElement("groundTruthWheelSpeedsTopicName")->Get<std::string>();
  }

  if (!sdf->HasElement("groundTruthCarStateTopic")) {
    RCLCPP_FATAL(
        _rosnode->get_logger(),
        "gazebo_ros_race_car_model plugin missing <groundTruthCarStateTopic>, cannot proceed");
    return;
  } else {
    _ground_truth_car_state_topic = sdf->GetElement("groundTruthCarStateTopic")->Get<std::string>();
  }

  if (!sdf->HasElement("localisationCarStateTopic")) {
    RCLCPP_FATAL(
        _rosnode->get_logger(),
        "gazebo_ros_race_car_model plugin missing <localisationCarStateTopic>, cannot proceed");
    return;
  } else {
    _localisation_car_state_topic =
        sdf->GetElement("localisationCarStateTopic")->Get<std::string>();
  }

  if (!sdf->HasElement("odometryTopicName")) {
    RCLCPP_FATAL(_rosnode->get_logger(),
                 "gazebo_ros_race_car_model plugin missing <odometryTopicName>, cannot proceed");
    return;
  } else {
    _odom_topic_name = sdf->GetElement("odometryTopicName")->Get<std::string>();
  }

  if (!sdf->HasElement("commandMode")) {
    RCLCPP_DEBUG(
        _rosnode->get_logger(),
        "gazebo_ros_race_car_model plugin missing <commandMode>, defaults to acceleration");
    _command_mode = acceleration;
  } else {
    auto temp = sdf->GetElement("commandMode")->Get<std::string>();
    if (temp.compare("acceleration") == 0) {
      _command_mode = acceleration;
    } else if (temp.compare("velocity") == 0) {
      _command_mode = velocity;
    } else {
      RCLCPP_WARN(_rosnode->get_logger(),
                  "commandMode parameter string is invalid, defaults to acceleration");
      _command_mode = acceleration;
    }
  }

  if (!sdf->HasElement("controlDelay")) {
    RCLCPP_FATAL(_rosnode->get_logger(),
                 "gazebo_ros_race_car_model plugin missing <controlDelay>, cannot proceed");
    return;
  } else {
    _control_delay = sdf->GetElement("controlDelay")->Get<double>();
  }

  if (!sdf->HasElement("steeringLockTime")) {
    RCLCPP_FATAL(_rosnode->get_logger(),
                 "gazebo_ros_race_car_model plugin missing <steeringLockTime>, cannot proceed");
    return;
  } else {
    _steering_lock_time = sdf->GetElement("steeringLockTime")->Get<double>();
  }

  if (!sdf->HasElement("pubGroundTruth")) {
    RCLCPP_FATAL(_rosnode->get_logger(),
                 "gazebo_ros_race_car_model plugin missing <pubGroundTruth>, cannot proceed");
    return;
  } else {
    _pub_ground_truth = sdf->GetElement("pubGroundTruth")->Get<bool>();
  }
}

void RaceCarModelPlugin::initVehicleModel(const sdf::ElementPtr &sdf) {
  // Get the vehicle model from the sdf
  std::string vehicle_model_ = "";
  if (!sdf->HasElement("vehicle_model")) {
    vehicle_model_ = "DynamicBicycle";
  } else {
    vehicle_model_ = sdf->GetElement("vehicle_model")->Get<std::string>();
  }

  std::string yaml_name = "";
  if (!sdf->HasElement("yaml_config")) {
    RCLCPP_FATAL(_rosnode->get_logger(),
                 "gazebo_ros_race_car_model plugin missing <yaml_config>, cannot proceed");
    return;
  } else {
    yaml_name = sdf->GetElement("yaml_config")->Get<std::string>();
  }

  RCLCPP_DEBUG(_rosnode->get_logger(), "RaceCarModelPlugin finished loading params");

  if (vehicle_model_ == "PointMass") {
    _vehicle = std::unique_ptr<eufs::models::VehicleModel>(new eufs::models::PointMass(yaml_name));
  } else if (vehicle_model_ == "DynamicBicycle") {
    _vehicle =
        std::unique_ptr<eufs::models::VehicleModel>(new eufs::models::DynamicBicycle(yaml_name));
  } else {
    RCLCPP_FATAL(_rosnode->get_logger(),
                 "gazebo_ros_race_car_model plugin invalid vehicle model, cannot proceed");
    return;
  }
}

void RaceCarModelPlugin::initModel(const sdf::ElementPtr &sdf) {
  // Steering joints
  std::string leftSteeringJointName =
      _model->GetName() + "::" + sdf->Get<std::string>("front_left_wheel_steering");
  _left_steering_joint = _model->GetJoint(leftSteeringJointName);
  std::string rightSteeringJointName =
      _model->GetName() + "::" + sdf->Get<std::string>("front_right_wheel_steering");
  _right_steering_joint = _model->GetJoint(rightSteeringJointName);
}

void RaceCarModelPlugin::initNoise(const sdf::ElementPtr &sdf) {
  std::string yaml_name = "";
  if (!sdf->HasElement("noise_config")) {
    RCLCPP_FATAL(_rosnode->get_logger(),
                 "gazebo_ros_race_car_model plugin missing <noise_config>, cannot proceed");
    return;
  } else {
    yaml_name = sdf->GetElement("noise_config")->Get<std::string>();
  }

  // Create noise object
  _noise = std::make_unique<eufs::models::Noise>(yaml_name);
}

void RaceCarModelPlugin::setPositionFromWorld() {
  _offset = _model->WorldPose();

  RCLCPP_DEBUG(_rosnode->get_logger(), "Got starting offset %f %f %f", _offset.Pos()[0],
               _offset.Pos()[1], _offset.Pos()[2]);

  _state.x = 0.0;
  _state.y = 0.0;
  _state.z = 0.0;
  _state.yaw = 0.0;
  _state.v_x = 0.0;
  _state.v_y = 0.0;
  _state.v_z = 0.0;
  _state.r_x = 0.0;
  _state.r_y = 0.0;
  _state.r_z = 0.0;
  _state.a_x = 0.0;
  _state.a_y = 0.0;
  _state.a_z = 0.0;
}

bool RaceCarModelPlugin::resetVehiclePosition(
    std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
  _state.x = 0.0;
  _state.y = 0.0;
  _state.z = 0.0;
  _state.yaw = 0.0;
  _state.v_x = 0.0;
  _state.v_y = 0.0;
  _state.v_z = 0.0;
  _state.r_x = 0.0;
  _state.r_y = 0.0;
  _state.r_z = 0.0;
  _state.a_x = 0.0;
  _state.a_y = 0.0;
  _state.a_z = 0.0;

  const ignition::math::Vector3d vel(0.0, 0.0, 0.0);
  const ignition::math::Vector3d angular(0.0, 0.0, 0.0);

  _model->SetWorldPose(_offset);
  _model->SetAngularVel(angular);
  _model->SetLinearVel(vel);

  return response->success;
}

void RaceCarModelPlugin::returnCommandMode(
    std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
  std::string command_mode_str;
  if (_command_mode == acceleration) {
    command_mode_str = "acceleration";
  } else {
    command_mode_str = "velocity";
  }

  response->success = true;
  response->message = command_mode_str;
}

void RaceCarModelPlugin::setModelState() {
  double yaw = _state.yaw + _offset.Rot().Yaw();

  double x =
      _offset.Pos().X() + _state.x * cos(_offset.Rot().Yaw()) - _state.y * sin(_offset.Rot().Yaw());
  double y =
      _offset.Pos().Y() + _state.x * sin(_offset.Rot().Yaw()) + _state.y * cos(_offset.Rot().Yaw());
  double z = _state.z;

  double vx = _state.v_x * cos(yaw) - _state.v_y * sin(yaw);
  double vy = _state.v_x * sin(yaw) + _state.v_y * cos(yaw);

  const ignition::math::Pose3d pose(x, y, z, 0, 0.0, yaw);
  const ignition::math::Vector3d vel(vx, vy, 0.0);
  const ignition::math::Vector3d angular(0.0, 0.0, _state.r_z);

  _model->SetWorldPose(pose);
  _model->SetAngularVel(angular);
  _model->SetLinearVel(vel);
}

eufs_msgs::msg::CarState RaceCarModelPlugin::stateToCarStateMsg(const eufs::models::State &state) {
  // Publish Car Info
  eufs_msgs::msg::CarState car_state;

  car_state.header.stamp.sec = _last_sim_time.sec;
  car_state.header.stamp.nanosec = _last_sim_time.nsec;
  car_state.header.frame_id = _reference_frame;
  car_state.child_frame_id = _robot_frame;

  car_state.pose.pose.position.x = state.x;
  car_state.pose.pose.position.y = state.y;
  car_state.pose.pose.position.z = state.z;

  std::vector<double> orientation = {state.yaw, 0.0, 0.0};

  orientation = ToQuaternion(orientation);

  car_state.pose.pose.orientation.x = orientation[0];
  car_state.pose.pose.orientation.y = orientation[1];
  car_state.pose.pose.orientation.z = orientation[2];
  car_state.pose.pose.orientation.w = orientation[3];

  car_state.twist.twist.linear.x = state.v_x;
  car_state.twist.twist.linear.y = state.v_y;
  car_state.twist.twist.linear.z = state.v_z;

  car_state.twist.twist.angular.x = state.r_x;
  car_state.twist.twist.angular.y = state.r_y;
  car_state.twist.twist.angular.z = state.r_z;

  car_state.linear_acceleration.x = state.a_x;
  car_state.linear_acceleration.y = state.a_y;
  car_state.linear_acceleration.z = state.a_z;

  car_state.slip_angle = _vehicle->getSlipAngle(_state, _act_input, true);

  car_state.state_of_charge = 999;

  return car_state;
}

void RaceCarModelPlugin::publishCarState() {
  eufs_msgs::msg::CarState car_state = stateToCarStateMsg(_state);

  // Publish the ground truth car state if it has subscribers and is allowed to publish
  if (_pub_ground_truth_car_state->get_subscription_count() > 0 && _pub_ground_truth) {
    _pub_ground_truth_car_state->publish(car_state);
  }

  // Add noise
  eufs::models::State state_noisy = _noise->applyNoise(_state);
  eufs_msgs::msg::CarState car_state_noisy = stateToCarStateMsg(state_noisy);

  // Fill in covariance matrix
  const eufs::models::NoiseParam &noise_param = _noise->getNoiseParam();
  car_state_noisy.pose.covariance[0] = pow(noise_param.position[0], 2);
  car_state_noisy.pose.covariance[7] = pow(noise_param.position[1], 2);
  car_state_noisy.pose.covariance[14] = pow(noise_param.position[2], 2);

  car_state_noisy.pose.covariance[21] = pow(noise_param.orientation[0], 2);
  car_state_noisy.pose.covariance[28] = pow(noise_param.orientation[1], 2);
  car_state_noisy.pose.covariance[35] = pow(noise_param.orientation[2], 2);

  car_state_noisy.twist.covariance[0] = pow(noise_param.linear_velocity[0], 2);
  car_state_noisy.twist.covariance[7] = pow(noise_param.linear_velocity[1], 2);
  car_state_noisy.twist.covariance[14] = pow(noise_param.linear_velocity[2], 2);

  car_state_noisy.twist.covariance[21] = pow(noise_param.angular_velocity[0], 2);
  car_state_noisy.twist.covariance[28] = pow(noise_param.angular_velocity[1], 2);
  car_state_noisy.twist.covariance[35] = pow(noise_param.angular_velocity[2], 2);

  car_state_noisy.linear_acceleration_covariance[0] = pow(noise_param.linear_acceleration[0], 2);
  car_state_noisy.linear_acceleration_covariance[4] = pow(noise_param.linear_acceleration[1], 2);
  car_state_noisy.linear_acceleration_covariance[8] = pow(noise_param.linear_acceleration[2], 2);

  // Publish with noise
  if (_pub_localisation_car_state->get_subscription_count() > 0) {
    _pub_localisation_car_state->publish(car_state_noisy);
  }
}

void RaceCarModelPlugin::publishWheelSpeeds() {
  eufs_msgs::msg::WheelSpeedsStamped wheel_speeds_stamped;
  eufs_msgs::msg::WheelSpeeds wheel_speeds;

  wheel_speeds_stamped.header.stamp.sec = _last_sim_time.sec;
  wheel_speeds_stamped.header.stamp.nanosec = _last_sim_time.nsec;
  wheel_speeds_stamped.header.frame_id = _robot_frame;

  wheel_speeds = _vehicle->getWheelSpeeds(_state, _act_input);
  wheel_speeds_stamped.speeds = wheel_speeds;

  // Publish the ground truth wheel speeds if it has subscribers and is allowed to publish
  if (_pub_ground_truth_wheel_speeds->get_subscription_count() > 0 && _pub_ground_truth) {
    _pub_ground_truth_wheel_speeds->publish(wheel_speeds_stamped);
  }

  wheel_speeds = _noise->applyNoiseToWheelSpeeds(wheel_speeds);
  wheel_speeds_stamped.speeds = wheel_speeds;

  // Publish with Noise
  if (_pub_wheel_speeds->get_subscription_count() > 0) {
    _pub_wheel_speeds->publish(wheel_speeds_stamped);
  }
}

void RaceCarModelPlugin::publishOdom() {
  nav_msgs::msg::Odometry odom;

  odom.header.stamp.sec = _last_sim_time.sec;
  odom.header.stamp.nanosec = _last_sim_time.nsec;

  odom.header.frame_id = _reference_frame;
  odom.child_frame_id = _robot_frame;

  eufs::models::State state_noisy = _noise->applyNoise(_state);

  odom.pose.pose.position.x = state_noisy.x;
  odom.pose.pose.position.y = state_noisy.y;
  odom.pose.pose.position.z = state_noisy.z;

  std::vector<double> orientation = {state_noisy.yaw, 0.0, 0.0};
  orientation = ToQuaternion(orientation);
  odom.pose.pose.orientation.x = orientation[0];
  odom.pose.pose.orientation.y = orientation[1];
  odom.pose.pose.orientation.z = orientation[2];
  odom.pose.pose.orientation.w = orientation[3];

  odom.twist.twist.linear.x = state_noisy.v_x;
  odom.twist.twist.linear.y = state_noisy.v_y;
  odom.twist.twist.linear.z = state_noisy.v_z;

  odom.twist.twist.angular.x = state_noisy.r_x;
  odom.twist.twist.angular.y = state_noisy.r_y;
  odom.twist.twist.angular.z = state_noisy.r_z;

  // fill in covariance matrix
  const eufs::models::NoiseParam &noise_param = _noise->getNoiseParam();
  odom.pose.covariance[0] = pow(noise_param.position[0], 2);
  odom.pose.covariance[7] = pow(noise_param.position[1], 2);
  odom.pose.covariance[14] = pow(noise_param.position[2], 2);

  odom.pose.covariance[21] = pow(noise_param.orientation[0], 2);
  odom.pose.covariance[28] = pow(noise_param.orientation[1], 2);
  odom.pose.covariance[35] = pow(noise_param.orientation[2], 2);

  odom.twist.covariance[0] = pow(noise_param.linear_velocity[0], 2);
  odom.twist.covariance[7] = pow(noise_param.linear_velocity[1], 2);
  odom.twist.covariance[14] = pow(noise_param.linear_velocity[2], 2);

  odom.twist.covariance[21] = pow(noise_param.angular_velocity[0], 2);
  odom.twist.covariance[28] = pow(noise_param.angular_velocity[1], 2);
  odom.twist.covariance[35] = pow(noise_param.angular_velocity[2], 2);

  // Publish the ground truth odom if it has subscribers and is allowed to publish
  if (_pub_odom->get_subscription_count() > 0 && _pub_ground_truth) {
    _pub_odom->publish(odom);
  }
}

void RaceCarModelPlugin::publishTf() {
  eufs::models::State state_noisy = _noise->applyNoise(_state);

  // Position
  tf2::Transform transform;
  transform.setOrigin(tf2::Vector3(state_noisy.x, state_noisy.y, 0.0));

  // Orientation
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, state_noisy.yaw);
  transform.setRotation(q);

  // Send TF
  geometry_msgs::msg::TransformStamped transform_stamped;

  transform_stamped.header.stamp.sec = _last_sim_time.sec;
  transform_stamped.header.stamp.nanosec = _last_sim_time.nsec;
  transform_stamped.header.frame_id = _reference_frame;
  transform_stamped.child_frame_id = _robot_frame;
  tf2::convert(transform, transform_stamped.transform);

  _tf_br->sendTransform(transform_stamped);
}

void RaceCarModelPlugin::Reset() { _last_sim_time = 0; }

void RaceCarModelPlugin::update() {
  gazebo::common::Time curTime = _world->SimTime();
  double dt = (curTime - _last_sim_time).Double();
  if (dt < (1 / _update_rate)) {
    return;
  }

  _last_sim_time = curTime;
  updateState(dt);
}

void RaceCarModelPlugin::updateState(const double dt) {
  if (!_command_Q.empty()) {
    gazebo::common::Time cmd_time = _cmd_time_Q.front();
    if ((_last_sim_time - cmd_time).Double() >= _control_delay) {
      std::shared_ptr<ackermann_msgs::msg::AckermannDriveStamped> cmd = _command_Q.front();
      _des_input.acc = cmd->drive.acceleration;
      _des_input.vel = cmd->drive.speed;
      _des_input.delta = cmd->drive.steering_angle;

      _command_Q.pop();
      _cmd_time_Q.pop();
    }
  }

  if (_command_mode == velocity) {
    double current_speed = std::sqrt(std::pow(_state.v_x, 2) + std::pow(_state.v_y, 2));
    _des_input.acc = (_des_input.vel - current_speed) / dt;
  }

  // If last command was more than 1s ago, then slow down car
  _act_input.acc = (_last_sim_time - _last_cmd_time) < 1.0 ? _des_input.acc : -1.0;
  // Make sure steering rate is within limits
  _act_input.delta +=
      (_des_input.delta - _act_input.delta >= 0 ? 1 : -1) *
      std::min(_max_steering_rate * dt, std::abs(_des_input.delta - _act_input.delta));

  // Update z value from simulation
  // This allows the state to have the most up to date value of z. Without this
  // the vehicle in simulation has problems interacting with the ground plane.
  // This may cause problems if the vehicle models start to take into account z
  // but because this simulation isn't for flying cars we should be ok (at least for now).
  _state.z = _model->WorldPose().Pos().Z();

  _vehicle->updateState(_state, _act_input, dt);

  _left_steering_joint->SetPosition(0, _act_input.delta);
  _right_steering_joint->SetPosition(0, _act_input.delta);
  setModelState();

  double time_since_last_published = (_last_sim_time - _time_last_published).Double();
  if (time_since_last_published < (1 / _publish_rate)) {
    return;
  }
  _time_last_published = _last_sim_time;

  // Publish Everything
  publishCarState();
  publishWheelSpeeds();
  publishOdom();

  if (_publish_tf) {
    publishTf();
  }

  _state_machine->spinOnce(_last_sim_time);
}

void RaceCarModelPlugin::onCmd(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg) {
  // Override commands if we're not in canDrive state
  if (!_state_machine->canDrive()) {
    msg->drive.steering_angle = 0;
    msg->drive.acceleration = -100;
    msg->drive.speed = 0;
  }
  _command_Q.push(msg);
  _cmd_time_Q.push(_world->SimTime());
  _last_cmd_time = _world->SimTime();
}

std::vector<double> RaceCarModelPlugin::ToQuaternion(std::vector<double> &euler) {
  // Abbreviations for the various angular functions
  double cy = cos(euler[0] * 0.5);
  double sy = sin(euler[0] * 0.5);
  double cp = cos(euler[1] * 0.5);
  double sp = sin(euler[1] * 0.5);
  double cr = cos(euler[2] * 0.5);
  double sr = sin(euler[2] * 0.5);

  std::vector<double> q;
  q.reserve(4);
  q[0] = cy * cp * sr - sy * sp * cr;  // x
  q[1] = sy * cp * sr + cy * sp * cr;  // y
  q[2] = sy * cp * cr - cy * sp * sr;  // z
  q[3] = cy * cp * cr + sy * sp * sr;  // w

  return q;
}

GZ_REGISTER_MODEL_PLUGIN(RaceCarModelPlugin)

}  // namespace eufs_plugins
}  // namespace gazebo_plugins
