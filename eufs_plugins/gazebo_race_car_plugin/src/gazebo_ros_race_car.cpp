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
#include "gazebo_race_car_plugin/gazebo_ros_race_car.hpp"

// STD Include
#include <algorithm>
#include <fstream>
#include <mutex>   // NOLINT(build/c++11)
#include <thread>  // NOLINT(build/c++11)

namespace gazebo_plugins {
namespace eufs_plugins {

RaceCarPlugin::RaceCarPlugin() {}

RaceCarPlugin::~RaceCarPlugin() { _update_connection.reset(); }

void RaceCarPlugin::Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) {
    _rosnode = gazebo_ros::Node::Get(sdf);

    RCLCPP_DEBUG(_rosnode->get_logger(), "Loading RaceCarPlugin");

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
    // Wheel odom
    _pub_wheel_odom = _rosnode->create_publisher<nav_msgs::msg::Odometry>("/vehicle/wheel_odom", 1);
    _pub_ground_truth_wheel_odom =
        _rosnode->create_publisher<nav_msgs::msg::Odometry>("/ground_truth/wheel_odom", 1);
    // Pose
    _pub_pose = _rosnode->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("slam/car_pose", 1);
    _pub_ground_truth_pose = _rosnode->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("ground_truth/car_pose", 1);

    // ROS Services
    _reset_vehicle_pos_srv = _rosnode->create_service<std_srvs::srv::Trigger>(
        "/ros_can/reset_vehicle_pos",
        std::bind(&RaceCarPlugin::resetVehiclePosition, this, std::placeholders::_1, std::placeholders::_2));
    _command_mode_srv = _rosnode->create_service<std_srvs::srv::Trigger>(
        "/race_car_model/command_mode",
        std::bind(&RaceCarPlugin::returnCommandMode, this, std::placeholders::_1, std::placeholders::_2));

    // ROS Subscriptions
    _sub_cmd = _rosnode->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
        "/control/driving_command", 1, std::bind(&RaceCarPlugin::onCmd, this, std::placeholders::_1));

    // Connect to Gazebo
    _update_connection = gazebo::event::Events::ConnectWorldUpdateBegin(std::bind(&RaceCarPlugin::update, this));
    _last_sim_time = _world->SimTime();

    _max_steering_rate = (_vehicle->getParam().input_ranges.delta.max - _vehicle->getParam().input_ranges.delta.min) /
                         _steering_lock_time;

    // Set offset
    setPositionFromWorld();

    RCLCPP_INFO(_rosnode->get_logger(), "RaceCarPlugin Loaded");
}

void RaceCarPlugin::initParams(const sdf::ElementPtr &sdf) {
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
        _reference_frame = "track";
    } else {
        _reference_frame = sdf->GetElement("referenceFrame")->Get<std::string>();
    }

    if (!sdf->HasElement("robotFrame")) {
        RCLCPP_DEBUG(_rosnode->get_logger(),
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

    if (!sdf->HasElement("commandMode")) {
        RCLCPP_DEBUG(_rosnode->get_logger(),
                     "gazebo_ros_race_car_model plugin missing <commandMode>, defaults to acceleration");
        _command_mode = acceleration;
    } else {
        auto temp = sdf->GetElement("commandMode")->Get<std::string>();
        if (temp.compare("acceleration") == 0) {
            _command_mode = acceleration;
        } else if (temp.compare("velocity") == 0) {
            _command_mode = velocity;
        } else {
            RCLCPP_WARN(_rosnode->get_logger(), "commandMode parameter string is invalid, defaults to acceleration");
            _command_mode = acceleration;
        }
    }

    if (!sdf->HasElement("controlDelay")) {
        RCLCPP_FATAL(_rosnode->get_logger(), "gazebo_ros_race_car_model plugin missing <controlDelay>, cannot proceed");
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

    if (!sdf->HasElement("simulateSLAM")) {
        RCLCPP_FATAL(_rosnode->get_logger(),
                     "gazebo_ros_race_car_model plugin missing <simulateSLAM>, cannot proceed");
        return;
    } else {
        _simulate_slam = sdf->GetElement("simulateSLAM")->Get<bool>();
    }
}

void RaceCarPlugin::initVehicleModel(const sdf::ElementPtr &sdf) {
    // Get the vehicle model from the sdf
    std::string vehicle_model_ = "";
    if (!sdf->HasElement("vehicle_model")) {
        vehicle_model_ = "DynamicBicycle";
    } else {
        vehicle_model_ = sdf->GetElement("vehicle_model")->Get<std::string>();
    }

    std::string yaml_name = "";
    if (!sdf->HasElement("yaml_config")) {
        RCLCPP_FATAL(_rosnode->get_logger(), "gazebo_ros_race_car_model plugin missing <yaml_config>, cannot proceed");
        return;
    } else {
        yaml_name = sdf->GetElement("yaml_config")->Get<std::string>();
    }

    RCLCPP_DEBUG(_rosnode->get_logger(), "RaceCarPlugin finished loading params");

    if (vehicle_model_ == "PointMass") {
        _vehicle = std::unique_ptr<eufs::models::VehicleModel>(new eufs::models::PointMass(yaml_name));
    } else if (vehicle_model_ == "DynamicBicycle") {
        _vehicle = std::unique_ptr<eufs::models::VehicleModel>(new eufs::models::DynamicBicycle(yaml_name));
    } else {
        RCLCPP_FATAL(_rosnode->get_logger(), "gazebo_ros_race_car_model plugin invalid vehicle model, cannot proceed");
        return;
    }
}

void RaceCarPlugin::initModel(const sdf::ElementPtr &sdf) {
    // Steering joints
    std::string leftSteeringJointName = _model->GetName() + "::" + sdf->Get<std::string>("front_left_wheel_steering");
    _left_steering_joint = _model->GetJoint(leftSteeringJointName);
    std::string rightSteeringJointName = _model->GetName() + "::" + sdf->Get<std::string>("front_right_wheel_steering");
    _right_steering_joint = _model->GetJoint(rightSteeringJointName);
}

void RaceCarPlugin::initNoise(const sdf::ElementPtr &sdf) {
    std::string yaml_name = "";
    if (!sdf->HasElement("noise_config")) {
        RCLCPP_FATAL(_rosnode->get_logger(), "gazebo_ros_race_car_model plugin missing <noise_config>, cannot proceed");
        return;
    } else {
        yaml_name = sdf->GetElement("noise_config")->Get<std::string>();
    }

    // Create noise object
    _noise = std::make_unique<eufs::models::Noise>(yaml_name);
}

void RaceCarPlugin::setPositionFromWorld() {
    _offset = _model->WorldPose();

    RCLCPP_DEBUG(_rosnode->get_logger(), "Got starting offset %f %f %f", _offset.Pos()[0], _offset.Pos()[1],
                 _offset.Pos()[2]);

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

    _last_cmd.drive.steering_angle = 0;
    _last_cmd.drive.acceleration = -100;
    _last_cmd.drive.speed = 0;
}

bool RaceCarPlugin::resetVehiclePosition(std::shared_ptr<std_srvs::srv::Trigger::Request>,
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

void RaceCarPlugin::returnCommandMode(std::shared_ptr<std_srvs::srv::Trigger::Request>,
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

void RaceCarPlugin::setModelState() {
    double yaw = _state.yaw + _offset.Rot().Yaw();

    double x = _offset.Pos().X() + _state.x * cos(_offset.Rot().Yaw()) - _state.y * sin(_offset.Rot().Yaw());
    double y = _offset.Pos().Y() + _state.x * sin(_offset.Rot().Yaw()) + _state.y * cos(_offset.Rot().Yaw());
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

geometry_msgs::msg::PoseWithCovarianceStamped RaceCarPlugin::stateToPoseMsg(const eufs::models::State &state) {
    geometry_msgs::msg::PoseWithCovarianceStamped pose_msg;

    pose_msg.header.stamp.sec = _last_sim_time.sec;
    pose_msg.header.stamp.nanosec = _last_sim_time.nsec;
    pose_msg.header.frame_id = _reference_frame;

    pose_msg.pose.pose.position.x = state.x;
    pose_msg.pose.pose.position.y = state.y;
    pose_msg.pose.pose.position.z = state.z;

    std::vector<double> orientation = {state.yaw, 0.0, 0.0};

    orientation = ToQuaternion(orientation);

    pose_msg.pose.pose.orientation.x = orientation[0];
    pose_msg.pose.pose.orientation.y = orientation[1];
    pose_msg.pose.pose.orientation.z = orientation[2];
    pose_msg.pose.pose.orientation.w = orientation[3];

    return pose_msg;
}

void RaceCarPlugin::publishCarPose() {
    geometry_msgs::msg::PoseWithCovarianceStamped pose = stateToPoseMsg(_state);

    // Add noise
    geometry_msgs::msg::PoseWithCovarianceStamped pose_noisy = stateToPoseMsg(_noise->applyNoise(_state));

    // Fill in covariance matrix
    const eufs::models::NoiseParam &noise_param = _noise->getNoiseParam();
    pose_noisy.pose.covariance[0] = pow(noise_param.position[0], 2);
    pose_noisy.pose.covariance[7] = pow(noise_param.position[1], 2);
    pose_noisy.pose.covariance[14] = pow(noise_param.position[2], 2);

    pose_noisy.pose.covariance[21] = pow(noise_param.orientation[0], 2);
    pose_noisy.pose.covariance[28] = pow(noise_param.orientation[1], 2);
    pose_noisy.pose.covariance[35] = pow(noise_param.orientation[2], 2);

    if (_pub_ground_truth_pose->get_subscription_count() > 0 && _pub_ground_truth) {
        _pub_ground_truth_pose->publish(pose);
    }

    if (_pub_pose->get_subscription_count() > 0 && _simulate_slam) {
        _pub_pose->publish(pose_noisy);
    }
}

nav_msgs::msg::Odometry RaceCarPlugin::getWheelOdometry(const eufs_msgs::msg::WheelSpeeds &wheel_speeds_noisy,
                                                        const eufs::models::Input &input) {
    nav_msgs::msg::Odometry wheel_odom;

    // Calculate avg wheel speeds
    float rf = wheel_speeds_noisy.rf_speed;
    float lf = wheel_speeds_noisy.lf_speed;
    float rb = wheel_speeds_noisy.rb_speed;
    float lb = wheel_speeds_noisy.lb_speed;
    float avg_wheel_speed = (rf + lf + rb + lb) / 4.0;

    // Calculate odom with wheel speed and steering angle
    wheel_odom.twist.twist.linear.x = avg_wheel_speed;
    wheel_odom.twist.twist.linear.y = 0.0;

    wheel_odom.twist.twist.angular.z = avg_wheel_speed * tan(input.delta) / _vehicle->getParam().kinematic.axle_width;

    wheel_odom.header.stamp.sec = _last_sim_time.sec;
    wheel_odom.header.stamp.nanosec = _last_sim_time.nsec;
    wheel_odom.header.frame_id = _reference_frame;
    wheel_odom.child_frame_id = _robot_frame;

    return wheel_odom;
}

void RaceCarPlugin::publishWheelOdom() {
    eufs_msgs::msg::WheelSpeeds wheel_speeds;

    // Calculate Wheel speeds
    wheel_speeds.lf_speed = _state.v_x;
    wheel_speeds.rf_speed = _state.v_x;
    wheel_speeds.lb_speed = _state.v_x;
    wheel_speeds.rb_speed = _state.v_x;

    nav_msgs::msg::Odometry wheel_odom = getWheelOdometry(wheel_speeds, _act_input);

    // Publish wheel odometry
    if (_pub_ground_truth_wheel_odom->get_subscription_count() > 0 && _pub_ground_truth) {
        _pub_ground_truth_wheel_odom->publish(wheel_odom);
    }

    // Add noise
    eufs_msgs::msg::WheelSpeeds wheel_speeds_noisy = _noise->applyNoiseToWheelSpeeds(wheel_speeds);
    nav_msgs::msg::Odometry wheel_odom_noisy = getWheelOdometry(wheel_speeds_noisy, _act_input);

    if (_pub_wheel_odom->get_subscription_count() > 0) {
        _pub_wheel_odom->publish(wheel_odom_noisy);
    }
}

void RaceCarPlugin::publishTf() {
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

void RaceCarPlugin::update() {
    gazebo::common::Time curTime = _world->SimTime();
    double dt = (curTime - _last_sim_time).Double();
    if (dt < (1 / _update_rate)) {
        return;
    }

    _last_sim_time = curTime;
    updateState(dt);
}

void RaceCarPlugin::updateState(const double dt) {
    _des_input.acc = _last_cmd.drive.acceleration;
    _des_input.vel = _last_cmd.drive.speed;
    _des_input.delta = _last_cmd.drive.steering_angle * 3.1415 / 180 /
                       (180 / 26);  // scales from steering wheel 90* to front wheels 26*

    if (_command_mode == velocity) {
        double current_speed = std::sqrt(std::pow(_state.v_x, 2) + std::pow(_state.v_y, 2));
        _act_input.acc = (_des_input.vel - current_speed) / dt;
    }

    // Make sure steering rate is within limits
    _act_input.delta += (_des_input.delta - _act_input.delta >= 0 ? 1 : -1) *
                        std::min(_max_steering_rate * dt, std::abs(_des_input.delta - _act_input.delta));

    // Ensure vehicle can drive
    if (!_state_machine->canDrive() || (_world->SimTime() - _last_cmd_time).Double() > 0.5) {
        _act_input.acc = -100.0;
        _act_input.vel = 0.0;
        _act_input.delta = 0.0;
    }

    counter++;
    if (counter == 100) {
        RCLCPP_DEBUG(_rosnode->get_logger(), "steering %f", _act_input.delta);
        counter = 0;
    }

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
    publishCarPose();
    publishWheelOdom();

    if (_publish_tf) {
        publishTf();
    }

    _state_machine->spinOnce(_last_sim_time);
}

void RaceCarPlugin::onCmd(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg) {
    RCLCPP_INFO(_rosnode->get_logger(), "Last time: %f", (_world->SimTime() - _last_cmd_time).Double());
    while ((_world->SimTime() - _last_cmd_time).Double() < _control_delay) {
        RCLCPP_DEBUG(_rosnode->get_logger(), "Waiting until control delay is over");
    }
    _last_cmd.drive.acceleration = msg->drive.acceleration;
    _last_cmd.drive.speed = msg->drive.speed;
    _last_cmd.drive.steering_angle = msg->drive.steering_angle;
    _last_cmd_time = _world->SimTime();
}

std::vector<double> RaceCarPlugin::ToQuaternion(std::vector<double> &euler) {
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

GZ_REGISTER_MODEL_PLUGIN(RaceCarPlugin)

}  // namespace eufs_plugins
}  // namespace gazebo_plugins
