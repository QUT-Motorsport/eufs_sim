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

// GZ Includes
#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/World.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/transport/Node.hh>
#include <gz/sim/Types.hh>
#include <gz/sim/Joint.hh>

// GZ Math (for Pose3d, Vector3d, etc.)
#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>

// TF2 Includes
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "helpers_ros.hpp"

namespace gazebo_plugins {
namespace eufs_plugins {

RaceCarPlugin::RaceCarPlugin() {}

RaceCarPlugin::~RaceCarPlugin() {}

void RaceCarPlugin::Configure(
    const gz::sim::Entity &entity,
    const std::shared_ptr<const sdf::Element> &sdf,
    gz::sim::EntityComponentManager &ecm,
    gz::sim::EventManager &eventMgr)
    {
    // Currently unused
    (void)sdf;
    (void)eventMgr;

    // Storage for later
    this->_entity = entity;
    this->_model = gz::sim::Model(entity);
    // error checking
    if (!this->_model.Valid(ecm)) {
        RCLCPP_ERROR(rclcpp::get_logger("race_car_plugin"), "Invalid model entity. Plugin won't run.");
        return;
    }

    // Ensure that ROS2 is initialized; if not, initialize it.
    if (!rclcpp::ok()) {
        rclcpp::init(0, nullptr);
    }

    this->_rosnode = std::make_shared<rclcpp::Node>("race_car_plugin_node");
    RCLCPP_INFO(this->_rosnode->get_logger(), "Created ROS node inside RaceCarPlugin.");

    // Initialize parameters
    this->initParams();

    RCLCPP_DEBUG(this->_rosnode->get_logger(), "Initializing publishers, subscribers, services...");

    // ROS Publishers
    // Wheel speeds
    this->_pub_wheel_twist =
      this->_rosnode->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>("/vehicle/wheel_twist", 1);
    // Steering angle
    this->_pub_steering_angle =
      this->_rosnode->create_publisher<std_msgs::msg::Float32>("/vehicle/steering_angle", 1);
    // Vehicle Velocity
    this->_pub_velocity =
      this->_rosnode->create_publisher<std_msgs::msg::Float32>("/vehicle/velocity", 1);
    // Visual odom
    this->_pub_vis_odom =
      this->_rosnode->create_publisher<nav_msgs::msg::Odometry>("/zed2i/zed_node/odom", 1);
    // SBG odometry
    this->_pub_odom =
      this->_rosnode->create_publisher<nav_msgs::msg::Odometry>("/odometry/sbg_ekf", 1);
    // Pose (from slam output)
    if (_simulate_slam) {
        _pub_pose = _rosnode->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("slam/car_pose", 1);
    }
    // Ground truth
    if (_pub_gt) {
        this->_pub_gt_wheel_twist =
          this->_rosnode->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>("/ground_truth/wheel_twist", 1);
        this->_pub_gt_odom =
          this->_rosnode->create_publisher<nav_msgs::msg::Odometry>("/ground_truth/odom", 1);
        this->_pub_gt_steering_angle =
          this->_rosnode->create_publisher<std_msgs::msg::Float32>("/ground_truth/steering_angle", 1);
        this->_pub_gt_velocity =
          this->_rosnode->create_publisher<std_msgs::msg::Float32>("/ground_truth/velocity", 1);
    }

    // Example names as in your URDF/Xacro
    this->_left_steering_joint =
        this->_model.JointByName(ecm, "left_steering_hinge_joint");
    this->_right_steering_joint =
        this->_model.JointByName(ecm, "right_steering_hinge_joint");

    // Drive-wheel joints
    this->_left_rear_wheel_joint =
        this->_model.JointByName(ecm, "rear_left_wheel_joint");
    this->_right_rear_wheel_joint =
        this->_model.JointByName(ecm, "rear_right_wheel_joint");

    // RVIZ joint visuals
    this->_pub_joint_state =
      this->_rosnode->create_publisher<sensor_msgs::msg::JointState>("/joint_states/steering", 1);

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

    // Create a TF broadcaster if needed
    this->_tf_br = std::make_unique<tf2_ros::TransformBroadcaster>(this->_rosnode);

    // locate steering joints
    auto leftJointName  = this->_model.Name(ecm) + "::left_steering_hinge_joint";
    auto rightJointName = this->_model.Name(ecm) + "::right_steering_hinge_joint";

    this->_left_steering_joint = this->_model.JointByName(ecm, "left_steering_hinge_joint");
    this->_right_steering_joint = this->_model.JointByName(ecm, "right_steering_hinge_joint");
    if (this->_left_steering_joint == gz::sim::kNullEntity ||
        this->_right_steering_joint == gz::sim::kNullEntity)
    {
        RCLCPP_WARN(this->_rosnode->get_logger(),
        "Could not find steering joints by name. Steering commands will not work. :(");
    }


    // Set offset
    this->setPositionFromWorld();

    // store the last-sim-time so we can find diff in PostUpdate (dt)
    // (info.simTime is in PostUpdate)
    this->_last_sim_time = 0.0;

    RCLCPP_INFO(_rosnode->get_logger(), "RaceCarPlugin Loaded");
}

void RaceCarPlugin::PreUpdate(
    const gz::sim::UpdateInfo &info,
    gz::sim::EntityComponentManager &ecm)
{
    // If the simulation is paused, do nothing
    if (info.paused)
      return;

    // Current sim time in seconds
    double currentSimTime = std::chrono::duration<double>(info.simTime).count();

    // Calculate dt
    double dt = currentSimTime - this->_last_sim_time;
    this->_last_sim_time = currentSimTime;

    // 100hz Rate chack
    if (dt < 0.01)
      return;

    // Now call your main update logic, passing in dt, the ECM, and anything else you need.
    this->update(ecm, info, dt);
}

void RaceCarPlugin::initParams() {
    // State
    // _as_state.state = driverless_msgs::msg::State::START;
    // _as_state.mission = driverless_msgs::msg::State::MISSION_NONE;

    // Get ROS parameters
    _update_rate   = _rosnode->declare_parameter("update_rate", 2.0);
    _publish_rate  = _rosnode->declare_parameter("publish_rate", 200.0);
    _map_frame     = _rosnode->declare_parameter("map_frame", "map");
    _odom_frame    = _rosnode->declare_parameter("odom_frame", "odom");
    _base_frame    = _rosnode->declare_parameter("base_frame", "base_link");
    _control_delay = _rosnode->declare_parameter("control_delay", 0.5);
    _pub_tf        = _rosnode->declare_parameter("simulate_transform", false);
    _pub_gt        = _rosnode->declare_parameter("publish_ground_truth", false);
    _simulate_slam = _rosnode->declare_parameter("simulate_slam", false);
    /// SHOULD BE IN VEHICLE PARAMS FILE
    _steering_lock_time = _rosnode->declare_parameter("steering_lock_time", 1.0);

    std::string command_str = _rosnode->declare_parameter("command_mode", "acceleration");
    if (command_str == "acceleration") {
        _command_mode = acceleration;
    } else if (command_str == "velocity") {
        _command_mode = velocity;
    } else {
        RCLCPP_FATAL(_rosnode->get_logger(), "RaceCarPlugin invalid command mode, cannot proceed");
        return;
    }

    // Vehicle model
    std::string vehicle_model_    = _rosnode->declare_parameter("vehicle_model", "DynamicBicycle");
    std::string vehicle_yaml_name = _rosnode->declare_parameter("vehicle_config", "/home/liam/QUTMS/install/eufs_config/share/eufs_config/config/configDry.yaml");
    if (vehicle_yaml_name == "null") {
        RCLCPP_FATAL(_rosnode->get_logger(), "gazebo_ros_race_car plugin missing <yamlConfig>, cannot proceed");
        return;
    }

    if (vehicle_model_ == "PointMass") {
        _vehicle = std::make_unique<eufs::models::PointMass>(vehicle_yaml_name);
    } else if (vehicle_model_ == "DynamicBicycle") {
        _vehicle = std::make_unique<eufs::models::DynamicBicycle>(vehicle_yaml_name);
    } else {
        RCLCPP_FATAL(_rosnode->get_logger(), "gazebo_ros_race_car plugin invalid vehicle model, cannot proceed");
        return;
    }

    // Noise
    std::string noise_yaml_name = _rosnode->declare_parameter("noise_config", "/home/liam/QUTMS/install/eufs_config/share/eufs_config/config/motionNoise.yaml");
    if (noise_yaml_name == "null") {
        RCLCPP_FATAL(_rosnode->get_logger(), "gazebo_ros_race_car plugin missing <noise_config>, cannot proceed");
        return;
    }
    _noise = std::make_unique<eufs::models::Noise>(noise_yaml_name);

    // Steering rate limit
    auto &ranges = _vehicle->getParam().input_ranges;
    _max_steering_rate = (ranges.delta.max - ranges.delta.min) / _steering_lock_time;
}

void RaceCarPlugin::setPositionFromWorld() {
    // will probably need to change this.
    _offset = gz::math::Pose3d(0,0,0, 0,0,0);

    _state.x   = 0.0;
    _state.y   = 0.0;
    _state.z   = 0.0;
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
    _last_cmd.drive.acceleration   = -100;
    _last_cmd.drive.speed          = 0;
}

bool RaceCarPlugin::resetVehiclePosition(std::shared_ptr<std_srvs::srv::Trigger::Request>,
                                         std::shared_ptr<std_srvs::srv::Trigger::Response> response) {

    // Reset internal state
    _state = eufs::models::State();
    return response->success;
}

void RaceCarPlugin::returnCommandMode(std::shared_ptr<std_srvs::srv::Trigger::Request>,
                                      std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    std::string command_mode_str = (_command_mode == acceleration) ? "acceleration" : "velocity";

    response->success = true;
    response->message = command_mode_str;
}

void RaceCarPlugin::setModelState(gz::sim::EntityComponentManager &ecm) {
    //===============================================================================
    double mass = 150; // Hard code, store variables in car information and read here
    double wheelRadius = 0.2032;                    

    // The desired total longitudinal force for the car
    double f_desired = mass * this->_act_input.acc;

    // Split load between 2 wheels
    double f_per_wheel = f_desired / 2.0;

    // Convert linear force to torque about the wheel’s spin axis
    double torque_per_wheel = f_per_wheel * wheelRadius;

    std::vector<double> force_per_wheel{torque_per_wheel};

    // Grab the joint objects
    gz::sim::Joint leftRearJoint(this->_left_rear_wheel_joint);
    gz::sim::Joint rightRearJoint(this->_right_rear_wheel_joint);

    if (leftRearJoint.Valid(ecm) && rightRearJoint.Valid(ecm))
    {
        // Apply the torque to each wheel’s rotation
        leftRearJoint.SetForce(ecm, force_per_wheel);
        rightRearJoint.SetForce(ecm, force_per_wheel);
    }
}

geometry_msgs::msg::PoseWithCovarianceStamped
RaceCarPlugin::odomToPoseMsg(const nav_msgs::msg::Odometry &odom_msg)
{
  geometry_msgs::msg::PoseWithCovarianceStamped pose_msg;
  pose_msg.header = odom_msg.header;
  pose_msg.pose   = odom_msg.pose;
  return pose_msg;
}

geometry_msgs::msg::TwistWithCovarianceStamped 
RaceCarPlugin::getWheelTwist(const std::vector<double> &speeds, const double &angle) 
{
    // calculate the average speed between the 4 wheels
    geometry_msgs::msg::TwistWithCovarianceStamped wheel_twist;
    double avg_wheel_speed = (speeds[0] + speeds[1] + speeds[2] + speeds[3]) / 4.0;
    wheel_twist.twist.twist.linear.x  = avg_wheel_speed;
    wheel_twist.twist.twist.angular.z =
      avg_wheel_speed * std::tan(angle) / _vehicle->getParam().kinematic.axle_width;

    wheel_twist.header.stamp = this->_rosnode->now();
    wheel_twist.header.frame_id = _base_frame;
    return wheel_twist;
}

nav_msgs::msg::Odometry 
RaceCarPlugin::stateToOdom(const eufs::models::State &state) 
{
    // convert all state field into respective odometry fields
    nav_msgs::msg::Odometry msg;
    msg.header.stamp    = this->_rosnode->now();
    msg.header.frame_id = _odom_frame;
    msg.child_frame_id  = _base_frame;

    msg.pose.pose.position.x = state.x;
    msg.pose.pose.position.y = state.y;

    // Convert yaw -> quaternion
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, state.yaw);
    msg.pose.pose.orientation = tf2::toMsg(q);
  
    msg.twist.twist.linear.x  = state.v_x;
    msg.twist.twist.linear.y  = state.v_y;
    msg.twist.twist.angular.z = state.r_z;

    return msg;
}

nav_msgs::msg::Odometry 
RaceCarPlugin::getVisualOdom(const nav_msgs::msg::Odometry &odom) 
    {
    nav_msgs::msg::Odometry msg = odom;
    msg.child_frame_id = _base_frame;
    return msg;
}

void RaceCarPlugin::publishVehicleMotion() {
    // Get odometry msg from state
    // Publish ground-truth
    if (_pub_gt) {
        nav_msgs::msg::Odometry odom = this->stateToOdom(_state);
        if (has_subscribers(_pub_gt_odom)) {
          _pub_gt_odom->publish(odom);
        }
    }

    // Publish pose
    // nav_msgs::msg::Odometry odom_noisy = stateToOdom(_noise->applyNoise(_state));
    // geometry_msgs::msg::PoseWithCovarianceStamped pose_noisy = odomToPoseMsg(odom_noisy);
    // if (has_subscribers(_pub_pose)) {
    //     _pub_pose->publish(pose_noisy);
    // }'

    // Publish noisy odom / slam pose if needed
    eufs::models::State noisyState = _noise->applyNoise(_state);
    nav_msgs::msg::Odometry odom_noisy = this->stateToOdom(noisyState);

    if (_simulate_slam && has_subscribers(_pub_pose)) {
        auto pose_noisy = this->odomToPoseMsg(odom_noisy);
        _pub_pose->publish(pose_noisy);
    }

    // Publish visual odom (THIS CAN BE IN A DIFFERENT PLUGIN)
    // nav_msgs::msg::Odometry visual_odom = getVisualOdom(odom_noisy);
    // if (has_subscribers(_pub_vis_odom)) {
    //     // Publish at 30Hz
    //     if (_last_sim_time - _time_last_vis_odom_published > (1 / 30)) {
    //         _pub_vis_odom->publish(visual_odom);
    //         _time_last_vis_odom_published = _last_sim_time;
    //     }
    // }

    // odom_noisy = stateToOdom(_noise->applyNoise(_state));
    // if (has_subscribers(_pub_odom)) {
    //     // Publish at 50Hz
    //     if (_last_sim_time - _time_last_sbg_odom_published > (1 / 50)) {
    //         _pub_odom->publish(odom_noisy);
    //         _time_last_sbg_odom_published = _last_sim_time;
    //     }
    // }

    // Visual odom
    if (has_subscribers(_pub_vis_odom)) {
        _pub_vis_odom->publish(this->getVisualOdom(odom_noisy));
    }
    // SBG odometry
    if (has_subscribers(_pub_odom)) {
        _pub_odom->publish(odom_noisy);
    }

    // Publish steering angle
    std_msgs::msg::Float32 steering_angle;

    // un-convert steering angle from radians to degrees and from linear to angular
    steering_angle.data = _act_input.delta * 90.0 / 16.0;
    steering_angle.data = steering_angle.data * 180.0 / M_PI;

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
    std_msgs::msg::Float32 vel;
    vel.data = static_cast<float>(_state.v_x);

    if (has_subscribers(_pub_gt_velocity)) {
        _pub_gt_velocity->publish(vel);
    }

    // Add noise
    std_msgs::msg::Float32 vel_noisy;

    vel_noisy.data = _noise->applyNoiseToSteering(vel.data);
    if (has_subscribers(_pub_velocity)) {
        _pub_velocity->publish(vel_noisy);
    }

    // Publish wheel twist
    std::vector<double> wheel_speeds = {_state.v_x, _state.v_x, _state.v_x, _state.v_x};
    auto wheel_twist = this->getWheelTwist(wheel_speeds, steering_angle.data);

    if (has_subscribers(_pub_gt_wheel_twist)) {
        _pub_gt_wheel_twist->publish(wheel_twist);
    }

    auto wheel_speeds_noisy = _noise->applyNoiseToWheels(wheel_speeds);
    auto wheel_twist_noisy = getWheelTwist(wheel_speeds_noisy, steering_angle_noisy.data);

    if (has_subscribers(_pub_wheel_twist)) {
        _pub_wheel_twist->publish(wheel_twist_noisy);
    }
}

void RaceCarPlugin::publishTf() {
    eufs::models::State noise_tf_state = _noise->applyNoise(_state);

    // Send TF
    geometry_msgs::msg::TransformStamped transform_stamped;
    transform_stamped.header.stamp = this->_rosnode->now();
    transform_stamped.header.frame_id = _odom_frame;
    transform_stamped.child_frame_id  = _base_frame;
    // tf2::convert(base_to_odom, transform_stamped.transform);

    transform_stamped.transform.translation.x = noise_tf_state.x;
    transform_stamped.transform.translation.y = noise_tf_state.y;
    transform_stamped.transform.translation.z = 0.0;
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, noise_tf_state.yaw);
    transform_stamped.transform.rotation = tf2::toMsg(q);

    this->_tf_br->sendTransform(transform_stamped);

    geometry_msgs::msg::TransformStamped map_to_odom;
    map_to_odom.header.stamp    = this->_rosnode->now();
    map_to_odom.header.frame_id = _map_frame;
    map_to_odom.child_frame_id  = _odom_frame;

    // Identity: zero translation, zero rotation
    map_to_odom.transform.translation.x = 0.0;
    map_to_odom.transform.translation.y = 0.0;
    map_to_odom.transform.translation.z = 0.0;

    tf2::Quaternion odom_map_q;
    odom_map_q.setRPY(0.0, 0.0, 0.0);
    map_to_odom.transform.rotation = tf2::toMsg(odom_map_q);

    this->_tf_br->sendTransform(map_to_odom);
    this->_tf_br->sendTransform(transform_stamped);
}

void RaceCarPlugin::update(gz::sim::EntityComponentManager &ecm,
                           const gz::sim::UpdateInfo &info,
                           double dt)
{
    _des_input.acc   = this->_last_cmd.drive.acceleration;
    _des_input.vel   = this->_last_cmd.drive.speed;
    _des_input.delta = this->_last_cmd.drive.steering_angle * M_PI / 180;
    _des_input.delta *= (16.0 / 90.0); // 90* (max steering angle) = 16* (max wheel angle) 
    
    if (_command_mode == velocity) {
        double current_speed = std::sqrt(std::pow(_state.v_x, 2) + std::pow(_state.v_y, 2));
        this->_act_input.acc = (this->_des_input.vel - current_speed) / dt;
    } else {
        // acceleration mode
        this->_act_input.acc = this->_des_input.acc;
    }

    // Make sure steering rate is within limits
    double desired = this->_des_input.delta;
    double actual  = this->_act_input.delta;
    double diff    = desired - actual;
    double sign    = (diff >= 0.0) ? 1.0 : -1.0;
    double step    = std::min(_max_steering_rate * dt, std::abs(diff));
    this->_act_input.delta = actual + sign * step;

    // Update z value from simulation
    // This allows the state to have the most up to date value of z. Without this
    // the vehicle in simulation has problems interacting with the ground plane.
    // This may cause problems if the vehicle models start to take into account z
    // but because this simulation isn't for flying cars we should be ok (at least for now).
    // Depreciated call changed to ecm call in   update
    // _state.z = _model->WorldPose().Pos().Z();

    // _vehicle->updateState(_state, _act_input, dt);
    this->_vehicle->updateState(this->_state, this->_act_input, dt);

    // Now apply the new pose, velocities to the actual gz-sim model
    this->setModelState(ecm);

    // Publish occasionally
    double currentSimTimeSec = std::chrono::duration<double>(info.simTime).count();
    double timeSinceLastPub = currentSimTimeSec - this->_time_last_published;
    if (timeSinceLastPub >= (1.0 / this->_publish_rate)) {
        this->_time_last_published = currentSimTimeSec;
        // Publish states
        this->publishVehicleMotion();
        if (this->_pub_tf) {
            this->publishTf();
        }
    }

    this->applySteeringJointPositions(ecm);
}

void RaceCarPlugin::applySteeringJointPositions(gz::sim::EntityComponentManager &ecm)
{
  gz::sim::Joint leftJoint(this->_left_steering_joint);
  gz::sim::Joint rightJoint(this->_right_steering_joint);

  if (!leftJoint.Valid(ecm) || !rightJoint.Valid(ecm))
  {
    RCLCPP_WARN(this->_rosnode->get_logger(),
                "Invalid steering joint entity; steering won't be applied.");
    return;
  }

  // The steering angle we want (from the vehicle model)
  double steeringAngle = this->_act_input.delta;

  // Hard reset the position TODO: Ackermann Steering
  leftJoint.ResetPosition(ecm, {steeringAngle});
  rightJoint.ResetPosition(ecm, {steeringAngle});

  // Publish JointStates for visualization
  sensor_msgs::msg::JointState joint_state;
  joint_state.header.stamp = this->_rosnode->now();
  joint_state.name.push_back("left_steering_hinge_joint");
  joint_state.name.push_back("right_steering_hinge_joint");
  joint_state.position.push_back(steeringAngle);
  joint_state.position.push_back(steeringAngle);
  this->_pub_joint_state->publish(joint_state);
}

void RaceCarPlugin::onCmd(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg) {
    // We store the command, so that next PostUpdate uses it
    _last_cmd = *msg;
    // Remember the time, try not to forget.
    _last_cmd_time = this->_last_sim_time;
}

}  // namespace eufs_plugins
}  // namespace gazebo_plugins

//=============================================================================
// Plugin registration (replaces the old GZ_REGISTER_MODEL_PLUGIN macro)
GZ_ADD_PLUGIN(
  gazebo_plugins::eufs_plugins::RaceCarPlugin,
  gz::sim::System,
  gazebo_plugins::eufs_plugins::RaceCarPlugin::ISystemConfigure,
  gazebo_plugins::eufs_plugins::RaceCarPlugin::ISystemPreUpdate
)

//=============================================================================
