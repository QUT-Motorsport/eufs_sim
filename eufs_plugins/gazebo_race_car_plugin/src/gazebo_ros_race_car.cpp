#include "gazebo_race_car_plugin/gazebo_ros_race_car.hpp"

namespace gazebo_plugins {
namespace eufs_plugins {

RaceCarPlugin::RaceCarPlugin() = default;

RaceCarPlugin::~RaceCarPlugin() = default;

void RaceCarPlugin::Configure(
    const gz::sim::Entity &entity,
    const std::shared_ptr<const sdf::Element> &sdf,
    gz::sim::EntityComponentManager &ecm,
    gz::sim::EventManager &eventMgr)
{
  (void)eventMgr;

  this->entity_ = entity;
  this->model_ = gz::sim::Model(entity);

  if (!this->model_.Valid(ecm)) {
    gzerr << "RaceCarPlugin loaded on invalid model entity." << std::endl;
    return;
  }

  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }

  this->ros_node_ = std::make_shared<rclcpp::Node>("race_car_plugin_node");

  this->readSdfParams(sdf);

  this->initial_pose_ = gz::sim::worldPose(this->entity_, ecm);

  this->state_.x = this->initial_pose_.Pos().X();
  this->state_.y = this->initial_pose_.Pos().Y();
  this->state_.z = this->initial_pose_.Pos().Z();
  this->state_.yaw = this->initial_pose_.Rot().Yaw();
  this->state_.speed = 0.0;
  this->state_.steering_rad = 0.0;

  this->last_cmd_.drive.speed = 0.0;
  this->last_cmd_.drive.acceleration = 0.0;
  this->last_cmd_.drive.steering_angle = 0.0;

  this->pub_velocity_ =
      this->ros_node_->create_publisher<std_msgs::msg::Float32>(
          "/vehicle/velocity", 10);

  this->pub_steering_angle_ =
      this->ros_node_->create_publisher<std_msgs::msg::Float32>(
          "/vehicle/steering_angle", 10);

  this->pub_wheel_twist_ =
      this->ros_node_->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(
          "/vehicle/wheel_twist", 10);

  this->pub_odom_ =
      this->ros_node_->create_publisher<nav_msgs::msg::Odometry>(
          "/odometry/sbg_ekf", 10);

  this->pub_visual_odom_ =
      this->ros_node_->create_publisher<nav_msgs::msg::Odometry>(
          "/zed2i/zed_node/odom", 10);

  this->pub_slam_pose_ =
      this->ros_node_->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
          "slam/car_pose", 10);

  this->pub_joint_state_ =
      this->ros_node_->create_publisher<sensor_msgs::msg::JointState>(
          "/joint_states/steering", 10);

  this->pub_gt_odom_ =
      this->ros_node_->create_publisher<nav_msgs::msg::Odometry>(
          "/ground_truth/odom", 10);

  this->pub_gt_velocity_ =
      this->ros_node_->create_publisher<std_msgs::msg::Float32>(
          "/ground_truth/velocity", 10);

  this->pub_gt_steering_angle_ =
      this->ros_node_->create_publisher<std_msgs::msg::Float32>(
          "/ground_truth/steering_angle", 10);

  this->pub_gt_wheel_twist_ =
      this->ros_node_->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(
          "/ground_truth/wheel_twist", 10);

  this->sub_cmd_ =
      this->ros_node_->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
          "/control/driving_command",
          10,
          std::bind(&RaceCarPlugin::onCmd, this, std::placeholders::_1));

  this->reset_service_ =
      this->ros_node_->create_service<std_srvs::srv::Trigger>(
          "/system/reset_car_pos",
          std::bind(
              &RaceCarPlugin::resetVehiclePosition,
              this,
              std::placeholders::_1,
              std::placeholders::_2));

    this->reset_cones_service_ =
        this->ros_node_->create_service<std_srvs::srv::Trigger>(
            "/reset_cones",
            std::bind(
                &RaceCarPlugin::resetCones,
                this,
                std::placeholders::_1,
                std::placeholders::_2));

    this->reset_cones_system_service_ =
        this->ros_node_->create_service<std_srvs::srv::Trigger>(
            "/system/reset_cones",
            std::bind(
                &RaceCarPlugin::resetCones,
                this,
                std::placeholders::_1,
                std::placeholders::_2));

  this->command_mode_service_ =
      this->ros_node_->create_service<std_srvs::srv::Trigger>(
          "/race_car_model/command_mode",
          std::bind(
              &RaceCarPlugin::returnCommandMode,
              this,
              std::placeholders::_1,
              std::placeholders::_2));

  this->tf_broadcaster_ =
      std::make_unique<tf2_ros::TransformBroadcaster>(this->ros_node_);
}

void RaceCarPlugin::readSdfParams(const std::shared_ptr<const sdf::Element> &sdf)
{
  if (!sdf) {
    return;
  }

  if (sdf->HasElement("command_mode")) {
    this->command_mode_ = sdf->Get<std::string>("command_mode");
  }

  if (sdf->HasElement("wheelbase")) {
    this->wheelbase_ = sdf->Get<double>("wheelbase");
  }

  if (sdf->HasElement("max_speed")) {
    this->max_speed_ = sdf->Get<double>("max_speed");
  }

  if (sdf->HasElement("max_accel")) {
    this->max_accel_ = sdf->Get<double>("max_accel");
  }

  if (sdf->HasElement("max_decel")) {
    this->max_decel_ = sdf->Get<double>("max_decel");
  }

  if (sdf->HasElement("max_steering_deg")) {
    this->max_steering_deg_ = sdf->Get<double>("max_steering_deg");
  }

  if (sdf->HasElement("publish_rate")) {
    this->publish_rate_ = sdf->Get<double>("publish_rate");
  }

  if (sdf->HasElement("update_rate")) {
    this->update_rate_ = sdf->Get<double>("update_rate");
  }

  if (sdf->HasElement("command_timeout")) {
    this->command_timeout_ = sdf->Get<double>("command_timeout");
  }

  if (sdf->HasElement("map_frame")) {
    this->map_frame_ = sdf->Get<std::string>("map_frame");
  }

  if (sdf->HasElement("odom_frame")) {
    this->odom_frame_ = sdf->Get<std::string>("odom_frame");
  }

  if (sdf->HasElement("base_frame")) {
    this->base_frame_ = sdf->Get<std::string>("base_frame");
  }

  if (sdf->HasElement("simulate_transform")) {
    this->publish_tf_ = sdf->Get<bool>("simulate_transform");
  }

  if (sdf->HasElement("publish_ground_truth")) {
    this->publish_ground_truth_ = sdf->Get<bool>("publish_ground_truth");
  }

  if (sdf->HasElement("simulate_slam")) {
    this->simulate_slam_ = sdf->Get<bool>("simulate_slam");
  }

  this->wheelbase_ = std::max(0.01, this->wheelbase_);
  this->publish_rate_ = std::max(1.0, this->publish_rate_);
  this->update_rate_ = std::max(1.0, this->update_rate_);
}

void RaceCarPlugin::PreUpdate(
    const gz::sim::UpdateInfo &info,
    gz::sim::EntityComponentManager &ecm)
{
  if (this->ros_node_) {
    rclcpp::spin_some(this->ros_node_);
  }

  if (info.paused) {
    return;
  }

  const double current_time =
      std::chrono::duration<double>(info.simTime).count();

  if (this->last_update_time_ < 0.0) {
    this->last_update_time_ = current_time;
    this->last_publish_time_ = current_time;
    return;
  }

  double dt = current_time - this->last_update_time_;

  if (dt <= 0.0) {
    return;
  }

  const double update_period = 1.0 / this->update_rate_;

  if (dt < update_period) {
    return;
  }

  this->last_update_time_ = current_time;

  this->updateVehicle(dt);
  this->applyModelPose(ecm);

  const double publish_period = 1.0 / this->publish_rate_;

  if ((current_time - this->last_publish_time_) >= publish_period) {
    this->last_publish_time_ = current_time;
    this->publishOutputs(info);
  }
}

void RaceCarPlugin::onCmd(
    const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(this->command_mutex_);
  this->last_cmd_ = *msg;
  this->has_command_ = true;
  this->last_command_sim_time_ = this->last_update_time_;
}

void RaceCarPlugin::updateVehicle(double dt)
{
  ackermann_msgs::msg::AckermannDriveStamped cmd;
  bool has_cmd = false;
  double last_cmd_time = -1.0;

  {
    std::lock_guard<std::mutex> lock(this->command_mutex_);
    cmd = this->last_cmd_;
    has_cmd = this->has_command_;
    last_cmd_time = this->last_command_sim_time_;
  }

  double target_speed = 0.0;
  double target_accel = 0.0;
  double target_steering_deg = 0.0;

  const bool command_timed_out =
      !has_cmd ||
      (last_cmd_time >= 0.0 &&
       this->last_update_time_ - last_cmd_time > this->command_timeout_);

  if (!command_timed_out) {
    target_speed = cmd.drive.speed;
    target_accel = cmd.drive.acceleration;
    target_steering_deg = cmd.drive.steering_angle;
  }

  target_speed = this->clamp(target_speed, -this->max_speed_, this->max_speed_);
  target_steering_deg =
      this->clamp(target_steering_deg, -this->max_steering_deg_, this->max_steering_deg_);

  const double target_steering_rad = target_steering_deg * M_PI / 180.0;

  if (this->command_mode_ == "acceleration") {
    double accel = this->clamp(target_accel, -this->max_decel_, this->max_accel_);
    this->state_.speed += accel * dt;
    this->state_.speed = this->clamp(this->state_.speed, -this->max_speed_, this->max_speed_);
  } else {
    const double speed_error = target_speed - this->state_.speed;
    const double max_delta =
        (speed_error >= 0.0 ? this->max_accel_ : this->max_decel_) * dt;

    this->state_.speed += this->clamp(speed_error, -max_delta, max_delta);
  }

  this->state_.steering_rad = target_steering_rad;

  const double yaw_rate =
      this->state_.speed / this->wheelbase_ * std::tan(this->state_.steering_rad);

  this->state_.yaw += yaw_rate * dt;

  while (this->state_.yaw > M_PI) {
    this->state_.yaw -= 2.0 * M_PI;
  }

  while (this->state_.yaw < -M_PI) {
    this->state_.yaw += 2.0 * M_PI;
  }

  this->state_.x += this->state_.speed * std::cos(this->state_.yaw) * dt;
  this->state_.y += this->state_.speed * std::sin(this->state_.yaw) * dt;
}

void RaceCarPlugin::applyModelPose(gz::sim::EntityComponentManager &ecm)
{
  gz::math::Pose3d target_pose(
      this->state_.x,
      this->state_.y,
      this->state_.z,
      0.0,
      0.0,
      this->state_.yaw);

  this->model_.SetWorldPoseCmd(ecm, target_pose);
}

void RaceCarPlugin::publishOutputs(const gz::sim::UpdateInfo &info)
{
  const auto stamp = this->simTimeToRosTime(info.simTime);

  std_msgs::msg::Float32 velocity_msg;
  velocity_msg.data = static_cast<float>(this->state_.speed);
  this->pub_velocity_->publish(velocity_msg);

  std_msgs::msg::Float32 steering_msg;
  steering_msg.data = static_cast<float>(this->state_.steering_rad * 180.0 / M_PI);
  this->pub_steering_angle_->publish(steering_msg);

  auto wheel_twist_msg = this->makeWheelTwistMsg(stamp);
  this->pub_wheel_twist_->publish(wheel_twist_msg);

  auto odom_msg = this->makeOdomMsg(stamp, this->odom_frame_, this->base_frame_);
  this->pub_odom_->publish(odom_msg);
  this->pub_visual_odom_->publish(odom_msg);

  sensor_msgs::msg::JointState joint_state_msg;
  joint_state_msg.header.stamp = stamp;
  joint_state_msg.name.push_back("left_steering_hinge_joint");
  joint_state_msg.name.push_back("right_steering_hinge_joint");
  joint_state_msg.position.push_back(this->state_.steering_rad);
  joint_state_msg.position.push_back(this->state_.steering_rad);
  this->pub_joint_state_->publish(joint_state_msg);

  if (this->publish_ground_truth_) {
    this->pub_gt_velocity_->publish(velocity_msg);
    this->pub_gt_steering_angle_->publish(steering_msg);
    this->pub_gt_wheel_twist_->publish(wheel_twist_msg);
    this->pub_gt_odom_->publish(odom_msg);
  }

  if (this->simulate_slam_) {
    geometry_msgs::msg::PoseWithCovarianceStamped pose_msg;
    pose_msg.header = odom_msg.header;
    pose_msg.pose = odom_msg.pose;
    this->pub_slam_pose_->publish(pose_msg);
  }

  if (this->publish_tf_) {
    this->publishTf(stamp);
  }
}

nav_msgs::msg::Odometry RaceCarPlugin::makeOdomMsg(
    const builtin_interfaces::msg::Time &stamp,
    const std::string &frame_id,
    const std::string &child_frame_id) const
{
  nav_msgs::msg::Odometry msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = frame_id;
  msg.child_frame_id = child_frame_id;

  msg.pose.pose.position.x = this->state_.x;
  msg.pose.pose.position.y = this->state_.y;
  msg.pose.pose.position.z = this->state_.z;

  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, this->state_.yaw);
  msg.pose.pose.orientation = tf2::toMsg(q);

  msg.twist.twist.linear.x = this->state_.speed;
  msg.twist.twist.linear.y = 0.0;
  msg.twist.twist.angular.z =
      this->state_.speed / this->wheelbase_ * std::tan(this->state_.steering_rad);

  return msg;
}

geometry_msgs::msg::TwistWithCovarianceStamped RaceCarPlugin::makeWheelTwistMsg(
    const builtin_interfaces::msg::Time &stamp) const
{
  geometry_msgs::msg::TwistWithCovarianceStamped msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = this->base_frame_;

  msg.twist.twist.linear.x = this->state_.speed;
  msg.twist.twist.angular.z =
      this->state_.speed / this->wheelbase_ * std::tan(this->state_.steering_rad);

  return msg;
}

void RaceCarPlugin::publishTf(const builtin_interfaces::msg::Time &stamp)
{
  geometry_msgs::msg::TransformStamped map_to_odom;
  map_to_odom.header.stamp = stamp;
  map_to_odom.header.frame_id = this->map_frame_;
  map_to_odom.child_frame_id = this->odom_frame_;
  map_to_odom.transform.translation.x = 0.0;
  map_to_odom.transform.translation.y = 0.0;
  map_to_odom.transform.translation.z = 0.0;

  tf2::Quaternion map_q;
  map_q.setRPY(0.0, 0.0, 0.0);
  map_to_odom.transform.rotation = tf2::toMsg(map_q);

  geometry_msgs::msg::TransformStamped odom_to_base;
  odom_to_base.header.stamp = stamp;
  odom_to_base.header.frame_id = this->odom_frame_;
  odom_to_base.child_frame_id = this->base_frame_;
  odom_to_base.transform.translation.x = this->state_.x;
  odom_to_base.transform.translation.y = this->state_.y;
  odom_to_base.transform.translation.z = this->state_.z;

  tf2::Quaternion base_q;
  base_q.setRPY(0.0, 0.0, this->state_.yaw);
  odom_to_base.transform.rotation = tf2::toMsg(base_q);

  this->tf_broadcaster_->sendTransform(map_to_odom);
  this->tf_broadcaster_->sendTransform(odom_to_base);
}

bool RaceCarPlugin::resetVehiclePosition(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  (void)request;

  this->state_.x = this->initial_pose_.Pos().X();
  this->state_.y = this->initial_pose_.Pos().Y();
  this->state_.z = this->initial_pose_.Pos().Z();
  this->state_.yaw = this->initial_pose_.Rot().Yaw();
  this->state_.speed = 0.0;
  this->state_.steering_rad = 0.0;

  {
    std::lock_guard<std::mutex> lock(this->command_mutex_);
    this->last_cmd_.drive.speed = 0.0;
    this->last_cmd_.drive.acceleration = 0.0;
    this->last_cmd_.drive.steering_angle = 0.0;
    this->has_command_ = false;
  }

  response->success = true;
  response->message = "Vehicle reset to initial pose";
  return true;
}

bool RaceCarPlugin::resetCones(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  (void)request;

  response->success = true;
  response->message = "Cone reset stub active. No cone backend is currently connected.";
  return true;
}

void RaceCarPlugin::returnCommandMode(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  (void)request;

  response->success = true;
  response->message = this->command_mode_;
}

builtin_interfaces::msg::Time RaceCarPlugin::simTimeToRosTime(
    const std::chrono::steady_clock::duration &sim_time) const
{
  const auto ns =
      std::chrono::duration_cast<std::chrono::nanoseconds>(sim_time).count();

  builtin_interfaces::msg::Time stamp;
  stamp.sec = static_cast<int32_t>(ns / 1000000000);
  stamp.nanosec = static_cast<uint32_t>(ns % 1000000000);
  return stamp;
}

double RaceCarPlugin::clamp(
    double value,
    double min_value,
    double max_value) const
{
  return std::max(min_value, std::min(value, max_value));
}

}  // namespace eufs_plugins
}  // namespace gazebo_plugins

GZ_ADD_PLUGIN(
    gazebo_plugins::eufs_plugins::RaceCarPlugin,
    gz::sim::System,
    gazebo_plugins::eufs_plugins::RaceCarPlugin::ISystemConfigure,
    gazebo_plugins::eufs_plugins::RaceCarPlugin::ISystemPreUpdate)