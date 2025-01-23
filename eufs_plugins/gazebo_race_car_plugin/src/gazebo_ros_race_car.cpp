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

    // Initialize parameters
    initParams();

    // ROS Publishers
    // Wheel speeds
    _pub_wheel_twist =
        _rosnode->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>("/vehicle/wheel_twist", 1);
    // Steering angle
    _pub_steering_angle =
        _rosnode->create_publisher<std_msgs::msg::Float32>("/vehicle/steering_angle", 1);
    // Steering angle
    _pub_velocity = _rosnode->create_publisher<std_msgs::msg::Float32>("/vehicle/velocity", 1);
    // Visual odom
    _pub_vis_odom = _rosnode->create_publisher<nav_msgs::msg::Odometry>("/zed2i/zed_node/odom", 1);
    // SBG odometry
    _pub_odom = _rosnode->create_publisher<nav_msgs::msg::Odometry>("/odometry/sbg_ekf", 1);
    // Pose (from slam output)
    if (_simulate_slam) {
        _pub_pose = _rosnode->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("slam/car_pose", 1);
    }
    // Ground truth
    if (_pub_gt) {
        _pub_gt_wheel_twist =
            _rosnode->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>("/ground_truth/wheel_twist", 1);
        _pub_gt_odom = _rosnode->create_publisher<nav_msgs::msg::Odometry>("/ground_truth/odom", 1);
        _pub_gt_steering_angle =
            _rosnode->create_publisher<std_msgs::msg::Float32>("/ground_truth/steering_angle", 1);
        _pub_gt_velocity =
            _rosnode->create_publisher<std_msgs::msg::Float32>("/ground_truth/velocity", 1);
    }

    // RVIZ joint visuals
    _pub_joint_state = _rosnode->create_publisher<sensor_msgs::msg::JointState>("/joint_states/steering", 1);

    // Driverless state
    // _sub_state = _rosnode->create_subscription<driverless_msgs::msg::State>(
    //     "/system/as_status", 1, std::bind(&RaceCarPlugin::updateState, this, std::placeholders::_1));

    // ROS Subscriptions
    _sub_cmd = _rosnode->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
        "/control/driving_command", 1, std::bind(&RaceCarPlugin::onCmd, this, std::placeholders::_1));

    // ROS Services
    _reset_vehicle_pos_srv = _rosnode->create_service<std_srvs::srv::Trigger>(
        "/system/reset_car_pos",
        std::bind(&RaceCarPlugin::resetVehiclePosition, this, std::placeholders::_1, std::placeholders::_2));
    _command_mode_srv = _rosnode->create_service<std_srvs::srv::Trigger>(
        "/race_car_model/command_mode",
        std::bind(&RaceCarPlugin::returnCommandMode, this, std::placeholders::_1, std::placeholders::_2));

    // Connect to Gazebo
    _update_connection = gazebo::event::Events::ConnectWorldUpdateBegin(std::bind(&RaceCarPlugin::update, this));
    _last_sim_time = _world->SimTime();

    _max_steering_rate = (_vehicle->getParam().input_ranges.delta.max - _vehicle->getParam().input_ranges.delta.min) /
                         _steering_lock_time;

    // Set offset
    setPositionFromWorld();

    RCLCPP_INFO(_rosnode->get_logger(), "RaceCarPlugin Loaded");
}

void RaceCarPlugin::initParams() {
    // State
    // _as_state.state = driverless_msgs::msg::State::START;
    // _as_state.mission = driverless_msgs::msg::State::MISSION_NONE;

    // Get ROS parameters
    _update_rate = _rosnode->declare_parameter("update_rate", 2.0);
    _publish_rate = _rosnode->declare_parameter("publish_rate", 200.0);
    _map_frame = _rosnode->declare_parameter("map_frame", "map");
    _odom_frame = _rosnode->declare_parameter("odom_frame", "odom");
    _base_frame = _rosnode->declare_parameter("base_frame", "base_link");
    _control_delay = _rosnode->declare_parameter("control_delay", 0.5);
    _pub_tf = _rosnode->declare_parameter("simulate_transform", false);
    _pub_gt = _rosnode->declare_parameter("publish_ground_truth", false);
    _simulate_slam = _rosnode->declare_parameter("simulate_slam", false);
    /// SHOULD BE IN VEHICLE PARAMS FILE
    _steering_lock_time = _rosnode->declare_parameter("steering_lock_time", 1.0);

    std::string command_str = _rosnode->declare_parameter("command_mode", "acceleration");
    if (command_str.compare("acceleration") == 0) {
        _command_mode = acceleration;
    } else if (command_str.compare("velocity") == 0) {
        _command_mode = velocity;
    } else {
        RCLCPP_FATAL(_rosnode->get_logger(), "gazebo_ros_race_car plugin invalid command mode, cannot proceed");
        return;
    }

    // Vehicle model
    std::string vehicle_model_ = _rosnode->declare_parameter("vehicle_model", "DynamicBicycle");
    std::string vehicle_yaml_name = _rosnode->declare_parameter("vehicle_config", "null");
    if (vehicle_yaml_name == "null") {
        RCLCPP_FATAL(_rosnode->get_logger(), "gazebo_ros_race_car plugin missing <yamlConfig>, cannot proceed");
        return;
    }

    if (vehicle_model_ == "PointMass") {
        _vehicle = std::unique_ptr<eufs::models::VehicleModel>(new eufs::models::PointMass(vehicle_yaml_name));
    } else if (vehicle_model_ == "DynamicBicycle") {
        _vehicle = std::unique_ptr<eufs::models::VehicleModel>(new eufs::models::DynamicBicycle(vehicle_yaml_name));
    } else {
        RCLCPP_FATAL(_rosnode->get_logger(), "gazebo_ros_race_car plugin invalid vehicle model, cannot proceed");
        return;
    }

    // Steering joints
    std::string leftSteeringJointName = _model->GetName() + "::left_steering_hinge_joint";
    _left_steering_joint = _model->GetJoint(leftSteeringJointName);
    std::string rightSteeringJointName = _model->GetName() + "::right_steering_hinge_joint";
    _right_steering_joint = _model->GetJoint(rightSteeringJointName);

    // Noise
    std::string noise_yaml_name = _rosnode->declare_parameter("noise_config", "null");
    if (noise_yaml_name == "null") {
        RCLCPP_FATAL(_rosnode->get_logger(), "gazebo_ros_race_car plugin missing <noise_config>, cannot proceed");
        return;
    }
    _noise = std::make_unique<eufs::models::Noise>(noise_yaml_name);
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

geometry_msgs::msg::PoseWithCovarianceStamped RaceCarPlugin::odomToPoseMsg(const nav_msgs::msg::Odometry &odom_msg) {
    geometry_msgs::msg::PoseWithCovarianceStamped pose_msg;

    pose_msg.header.stamp = odom_msg.header.stamp;
    pose_msg.header.frame_id = _map_frame;
    pose_msg.pose = odom_msg.pose;

    return pose_msg;
}

geometry_msgs::msg::TwistWithCovarianceStamped RaceCarPlugin::getWheelTwist(const std::vector<double> &speeds,
                                                                            const double &angle) {
    geometry_msgs::msg::TwistWithCovarianceStamped wheel_twist;

    // Calculate avg wheel speeds
    double rf = speeds[0];
    double lf = speeds[1];
    double rb = speeds[2];
    double lb = speeds[3];
    double avg_wheel_speed = (rf + lf + rb + lb) / 4.0;

    // Calculate odom with wheel speed and steering angle
    wheel_twist.twist.twist.linear.x = avg_wheel_speed;

    wheel_twist.twist.twist.angular.z = avg_wheel_speed * tan(angle) / _vehicle->getParam().kinematic.axle_width;

    wheel_twist.header.stamp.sec = _last_sim_time.sec;
    wheel_twist.header.stamp.nanosec = _last_sim_time.nsec;
    wheel_twist.header.frame_id = _base_frame;

    return wheel_twist;
}

nav_msgs::msg::Odometry RaceCarPlugin::stateToOdom(const eufs::models::State &state) {
    // convert all state field into respective odometry fields
    nav_msgs::msg::Odometry msg;
    msg.header.stamp.sec = _last_sim_time.sec;
    msg.header.stamp.nanosec = _last_sim_time.nsec;
    msg.header.frame_id = _odom_frame;
    msg.child_frame_id = _base_frame;

    msg.pose.pose.position.x = state.x;
    msg.pose.pose.position.y = state.y;

    std::vector<double> orientation = {state.yaw, 0.0, 0.0};
    orientation = to_quaternion(orientation);

    msg.pose.pose.orientation.x = orientation[0];
    msg.pose.pose.orientation.y = orientation[1];
    msg.pose.pose.orientation.z = orientation[2];
    msg.pose.pose.orientation.w = orientation[3];

    msg.twist.twist.linear.x = state.v_x;
    msg.twist.twist.linear.y = state.v_y;

    msg.twist.twist.angular.z = state.yaw;

    return msg;
}

nav_msgs::msg::Odometry RaceCarPlugin::getVisualOdom(const nav_msgs::msg::Odometry &odom) {
    nav_msgs::msg::Odometry msg;
    msg.header = odom.header;
    msg.pose = odom.pose;
    msg.child_frame_id = _base_frame;

    return msg;
}

void RaceCarPlugin::publishVehicleMotion() {
    // Get odometry msg from state
    nav_msgs::msg::Odometry odom = stateToOdom(_state);
    if (has_subscribers(_pub_gt_odom)) {
        _pub_gt_odom->publish(odom);
    }

    // Publish pose
    nav_msgs::msg::Odometry odom_noisy = stateToOdom(_noise->applyNoise(_state));
    geometry_msgs::msg::PoseWithCovarianceStamped pose_noisy = odomToPoseMsg(odom_noisy);
    if (has_subscribers(_pub_pose)) {
        _pub_pose->publish(pose_noisy);
    }

    // Publish visual odom (THIS CAN BE IN A DIFFERENT PLUGIN)
    nav_msgs::msg::Odometry visual_odom = getVisualOdom(odom_noisy);
    if (has_subscribers(_pub_vis_odom)) {
        // Publish at 30Hz
        if (_last_sim_time - _time_last_vis_odom_published > (1 / 30)) {
            _pub_vis_odom->publish(visual_odom);
            _time_last_vis_odom_published = _last_sim_time;
        }
    }

    odom_noisy = stateToOdom(_noise->applyNoise(_state));
    if (has_subscribers(_pub_odom)) {
        // Publish at 50Hz
        if (_last_sim_time - _time_last_sbg_odom_published > (1 / 50)) {
            _pub_odom->publish(odom_noisy);
            _time_last_sbg_odom_published = _last_sim_time;
        }
    }

    // Publish steering angle
    std_msgs::msg::Float32 steering_angle;
    // un-convert steering angle from radians to degrees and from linear to angular
    steering_angle.data = _act_input.delta * 90.0 / 16.0;
    steering_angle.data = steering_angle.data * 180.0 / 3.1415;

    if (has_subscribers(_pub_gt_steering_angle)) {
        _pub_gt_steering_angle->publish(steering_angle);
    }

    // Add noise
    std_msgs::msg::Float32 steering_angle_noisy;
    steering_angle_noisy.data = _noise->applyNoiseToSteering(steering_angle.data);

    if (has_subscribers(_pub_steering_angle)) {
        _pub_steering_angle->publish(steering_angle_noisy);
    }

    // Publish velocity
    std_msgs::msg::Float32 velocity;
    velocity.data = _state.v_x;

    if (has_subscribers(_pub_gt_velocity)) {
        _pub_gt_velocity->publish(velocity);
    }

    // Add noise
    std_msgs::msg::Float32 velocity_noisy;
    velocity_noisy.data = _noise->applyNoiseToSteering(velocity.data);

    if (has_subscribers(_pub_velocity)) {
        _pub_velocity->publish(velocity_noisy);
    }

    // Publish wheel twist
    std::vector<double> wheel_speeds = {_state.v_x, _state.v_x, _state.v_x, _state.v_x};
    geometry_msgs::msg::TwistWithCovarianceStamped wheel_twist = getWheelTwist(wheel_speeds, steering_angle.data);

    if (has_subscribers(_pub_gt_wheel_twist)) {
        _pub_gt_wheel_twist->publish(wheel_twist);
    }

    std::vector<double> wheel_speeds_noisy = _noise->applyNoiseToWheels(wheel_speeds);
    geometry_msgs::msg::TwistWithCovarianceStamped wheel_twist_noisy =
        getWheelTwist(wheel_speeds_noisy, steering_angle_noisy.data);

    if (has_subscribers(_pub_wheel_twist)) {
        _pub_wheel_twist->publish(wheel_twist_noisy);
    }
}

void RaceCarPlugin::publishTf() {
    // Base->Odom
    // Position
    tf2::Transform base_to_odom;
    eufs::models::State noise_tf_state = _noise->applyNoise(_state);
    base_to_odom.setOrigin(tf2::Vector3(noise_tf_state.x, noise_tf_state.y, 0.0));

    // Orientation
    tf2::Quaternion base_odom_q;
    base_odom_q.setRPY(0.0, 0.0, noise_tf_state.yaw);
    base_to_odom.setRotation(base_odom_q);

    // Send TF
    geometry_msgs::msg::TransformStamped transform_stamped;

    transform_stamped.header.stamp.sec = _last_sim_time.sec;
    transform_stamped.header.stamp.nanosec = _last_sim_time.nsec;
    transform_stamped.header.frame_id = _odom_frame;
    transform_stamped.child_frame_id = _base_frame;
    tf2::convert(base_to_odom, transform_stamped.transform);

    _tf_br->sendTransform(transform_stamped);

    // Odom->Map
    // Position
    tf2::Transform odom_to_map;
    odom_to_map.setOrigin(tf2::Vector3(0.0, 0.0, 0.0));

    // Orientation
    tf2::Quaternion odom_map_q;
    odom_map_q.setRPY(0.0, 0.0, 0.0);
    odom_to_map.setRotation(odom_map_q);

    // Send TF
    transform_stamped.header.stamp.sec = _last_sim_time.sec;
    transform_stamped.header.stamp.nanosec = _last_sim_time.nsec;
    transform_stamped.header.frame_id = _map_frame;
    transform_stamped.child_frame_id = _odom_frame;
    tf2::convert(odom_to_map, transform_stamped.transform);

    _tf_br->sendTransform(transform_stamped);
}

void RaceCarPlugin::update() {
    // Check against update rate
    gazebo::common::Time curTime = _world->SimTime();
    double dt = calc_dt(_last_sim_time, curTime);
    if (dt < (1 / _update_rate)) {
        return;
    }

    _last_sim_time = curTime;

    _des_input.acc = _last_cmd.drive.acceleration;
    _des_input.vel = _last_cmd.drive.speed;
    _des_input.delta = _last_cmd.drive.steering_angle * 3.1415 / 180;
    // 90* (max steering angle) = 16* (max wheel angle)
    _des_input.delta *= (16.0 / 90.0);  // maybe use params not hardcoded?

    if (_command_mode == velocity) {
        double current_speed = std::sqrt(std::pow(_state.v_x, 2) + std::pow(_state.v_y, 2));
        _act_input.acc = (_des_input.vel - current_speed) / dt;
    }

    // Make sure steering rate is within limits
    _act_input.delta += (_des_input.delta - _act_input.delta >= 0 ? 1 : -1) *
                        std::min(_max_steering_rate * dt, std::abs(_des_input.delta - _act_input.delta));

    // Ensure vehicle can drive
    // if (_as_state.state != driverless_msgs::msg::State::DRIVING ||
    //     (_world->SimTime() - _last_cmd_time).Double() > 0.5) {
    //     _act_input.acc = -100.0;
    //     _act_input.vel = 0.0;
    //     _act_input.delta = 0.0;
    // }

    counter++;
    if (counter == 100) {
        RCLCPP_DEBUG(_rosnode->get_logger(), "steering desired: %.2f, desired: %.2f", _des_input.delta,
                     _act_input.delta);
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
    // joint states
    sensor_msgs::msg::JointState joint_state;
    joint_state.header.stamp.sec = _last_sim_time.sec;
    joint_state.header.stamp.nanosec = _last_sim_time.nsec;
    joint_state.name.push_back(_left_steering_joint->GetName());
    joint_state.name.push_back(_right_steering_joint->GetName());
    joint_state.position.push_back(_left_steering_joint->Position());
    joint_state.position.push_back(_right_steering_joint->Position());

    _pub_joint_state->publish(joint_state);

    setModelState();

    double time_since_last_published = (_last_sim_time - _time_last_published).Double();
    if (time_since_last_published < (1 / _publish_rate)) {
        return;
    }
    _time_last_published = _last_sim_time;

    // Publish car states
    publishVehicleMotion();

    if (_pub_tf) {
        publishTf();
    }
}

// void RaceCarPlugin::updateState(const driverless_msgs::msg::State::SharedPtr msg) { _as_state = *msg; }

void RaceCarPlugin::onCmd(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg) {
    RCLCPP_DEBUG(_rosnode->get_logger(), "Last time: %f", (_world->SimTime() - _last_cmd_time).Double());
    while ((_world->SimTime() - _last_cmd_time).Double() < _control_delay) {
        RCLCPP_DEBUG(_rosnode->get_logger(), "Waiting until control delay is over");
    }
    _last_cmd.drive.acceleration = msg->drive.acceleration;
    _last_cmd.drive.speed = msg->drive.speed;
    _last_cmd.drive.steering_angle = msg->drive.steering_angle;
    _last_cmd_time = _world->SimTime();
}

GZ_REGISTER_MODEL_PLUGIN(RaceCarPlugin)

}  // namespace eufs_plugins
}  // namespace gazebo_plugins
