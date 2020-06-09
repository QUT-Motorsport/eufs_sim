/*
 * AMZ-Driverless
 * Copyright (c) 2018 Authors:
 *   - Juraj Kabzan <kabzanj@gmail.com>
 *   - Miguel de la Iglesia Valls <dmiguel@ethz.ch>
 *   - Manuel Dangel <mdangel@student.ethz.ch>
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

#include "vehicle_model.hpp"

namespace gazebo {
namespace fssim {

VehicleModel::VehicleModel(physics::ModelPtr &_model,
                 sdf::ElementPtr &_sdf,
                 boost::shared_ptr<ros::NodeHandle> &nh,
                 transport::NodePtr &gznode)
  : nh_(nh),
    model(_model),
    state_machine_(nh)
{
  // For the Gaussian Kernel random number generation
  this->seed = 0;

  // Initialization
  this->initParam(_sdf);
  this->initModel(_sdf);
  this->initVehicleParam(_sdf);

  // ROS Publishers
  this->pub_car_state_     = nh->advertise<eufs_msgs::CarState>(this->state_topic_name_, 1);
  this->pub_wheel_speeds_  = nh->advertise<eufs_msgs::WheelSpeedsStamped>(this->wheel_speeds_topic_name_, 1);
  this->pub_odom_          = nh->advertise<nav_msgs::Odometry>(this->odom_topic_name_, 1);

  // ROS Subscribers
  this->sub_cmd_          = nh->subscribe("/cmd_vel_out", 1, &VehicleModel::onCmd, this);
  this->sub_initial_pose_ = nh->subscribe("/initialpose", 1, &VehicleModel::onInitialPose, this);

  this->setPositionFromWorld();

  this->time_last_cmd_ = 0.0;
}

void VehicleModel::initParam(sdf::ElementPtr &_sdf) {
  if (!_sdf->HasElement("referenceFrame")) {
    ROS_DEBUG_NAMED("state_ground_truth", "state_ground_truth plugin missing <referenceFrame>, defaults to map");
    this->reference_frame_ = "map";
  } else {
    this->reference_frame_ = _sdf->GetElement("referenceFrame")->Get<std::string>();
  }

  if (!_sdf->HasElement("robotFrame")) {
    ROS_DEBUG_NAMED("state_ground_truth", "state_ground_truth plugin missing <robotFrame>, defaults to base_footprint");
    this->robot_frame_ = "base_footprint";
  } else {
    this->robot_frame_ = _sdf->GetElement("robotFrame")->Get<std::string>();
  }

  if (!_sdf->HasElement("publishTransform")) {
    ROS_DEBUG_NAMED("state_ground_truth", "state_ground_truth plugin missing <publishTransform>, defaults to false");
    this->publish_tf_ = false;
  } else {
    this->publish_tf_ = _sdf->GetElement("publishTransform")->Get<bool>();
  }

  if (!_sdf->HasElement("wheelSpeedsTopicName")) {
    ROS_FATAL_NAMED("state_ground_truth", "state_ground_truth plugin missing <wheelSpeedsTopicName>, cannot proceed");
    return;
  } else {
    this->wheel_speeds_topic_name_ = _sdf->GetElement("wheelSpeedsTopicName")->Get<std::string>();
  }

  if (!_sdf->HasElement("stateTopicName")) {
    ROS_FATAL_NAMED("state_ground_truth", "state_ground_truth plugin missing <stateTopicName>, cannot proceed");
    return;
  } else {
    this->state_topic_name_ = _sdf->GetElement("stateTopicName")->Get<std::string>();
  }

  if (!_sdf->HasElement("odometryTopicName")) {
    ROS_FATAL_NAMED("state_ground_truth", "state_ground_truth plugin missing <odometryTopicName>, cannot proceed");
    return;
  } else {
    this->odom_topic_name_ = _sdf->GetElement("odometryTopicName")->Get<std::string>();
  }

  if (!_sdf->HasElement("positionNoise")) {
    ROS_DEBUG_NAMED("state_ground_truth",
                    "state_ground_truth plugin missing <positionNoise>, defaults to 0.0, 0.0, 0.0");
    this->position_noise_ = {0.0, 0.0, 0.0};
  } else {
    auto temp = _sdf->GetElement("positionNoise")->Get<ignition::math::Vector3d>();
    this->position_noise_ = {temp.X(), temp.Y(), temp.Z()};
  }

  if (this->position_noise_.size() != 3) {
    ROS_FATAL_NAMED("state_ground_truth", "positionNoise parameter vector is not of size 3");
  }

  if (!_sdf->HasElement("orientationNoise")) {
    ROS_DEBUG_NAMED("state_ground_truth",
                    "state_ground_truth plugin missing <orientationNoise>, defaults to 0.0, 0.0, 0.0");
    this->orientation_noise_ = {0.0, 0.0, 0.0};
  } else {
    auto temp = _sdf->GetElement("orientationNoise")->Get<ignition::math::Vector3d>();
    this->orientation_noise_ = {temp.X(), temp.Y(), temp.Z()};
  }

  if (this->orientation_noise_.size() != 3) {
    ROS_FATAL_NAMED("state_ground_truth", "orientationNoise parameter vector is not of size 3");
  }

  // convert orientation to quaternion
  this->orientation_quat_noise_ = this->ToQuaternion(this->orientation_noise_);

  if (!_sdf->HasElement("linearVelocityNoise")) {
    ROS_DEBUG_NAMED("state_ground_truth",
                    "state_ground_truth plugin missing <linearVelocityNoise>, defaults to 0.0, 0.0, 0.0");
    this->linear_velocity_noise_ = {0.0, 0.0, 0.0};
  } else {
    auto temp = _sdf->GetElement("linearVelocityNoise")->Get<ignition::math::Vector3d>();
    this->linear_velocity_noise_ = {temp.X(), temp.Y(), temp.Z()};
  }

  if (this->linear_velocity_noise_.size() != 3) {
    ROS_FATAL_NAMED("state_ground_truth", "linearVelocityNoise parameter vector is not of size 3");
  }

  if (!_sdf->HasElement("angularVelocityNoise")) {
    ROS_DEBUG_NAMED("state_ground_truth",
                    "state_ground_truth plugin missing <angularVelocityNoise>, defaults to 0.0, 0.0, 0.0");
    this->angular_velocity_noise_ = {0.0, 0.0, 0.0};
  } else {
    auto temp = _sdf->GetElement("angularVelocityNoise")->Get<ignition::math::Vector3d>();
    this->angular_velocity_noise_ = {temp.X(), temp.Y(), temp.Z()};
  }

  if (this->angular_velocity_noise_.size() != 3) {
    ROS_FATAL_NAMED("state_ground_truth", "angularVelocityNoise parameter vector is not of size 3");
  }

  if (!_sdf->HasElement("linearAccelerationNoise")) {
    ROS_DEBUG_NAMED("state_ground_truth",
                    "state_ground_truth plugin missing <linearAccelerationNoise>, defaults to 0.0, 0.0, 0.0");
    this->linear_acceleration_noise_ = {0.0, 0.0, 0.0};
  } else {
    auto temp = _sdf->GetElement("linearAccelerationNoise")->Get<ignition::math::Vector3d>();
    this->linear_acceleration_noise_ = {temp.X(), temp.Y(), temp.Z()};
  }

  if (this->linear_acceleration_noise_.size() != 3) {
    ROS_FATAL_NAMED("state_ground_truth", "linearAccelerationNoise parameter vector is not of size 3");
  }
}

std::vector<double> VehicleModel::ToQuaternion(std::vector<double> &euler) {
  // Abbreviations for the various angular functions
  double cy = cos(euler[0] * 0.5);
  double sy = sin(euler[0] * 0.5);
  double cp = cos(euler[1] * 0.5);
  double sp = sin(euler[1] * 0.5);
  double cr = cos(euler[2] * 0.5);
  double sr = sin(euler[2] * 0.5);

  std::vector<double> q;
  q.reserve(4);
  q[0] = cy * cp * cr + sy * sp * sr;
  q[1] = cy * cp * sr - sy * sp * cr;
  q[2] = sy * cp * sr + cy * sp * cr;
  q[3] = sy * cp * cr - cy * sp * sr;

  return q;
}


void VehicleModel::setPositionFromWorld() {
  auto       pos   = model->WorldPose();
  const auto vel   = model->WorldLinearVel();
  const auto accel = model->WorldLinearAccel();
  const auto r     = model->WorldAngularVel();

  state_.x   = pos.Pos().X();
  state_.y   = pos.Pos().Y();
  state_.yaw = pos.Rot().Yaw();
  state_.v_x = 0.0;
  state_.v_y = 0.0;
  state_.r   = 0.0;
  state_.a_x = 0.0;
  state_.a_y = 0.0;
}

void VehicleModel::initModel(sdf::ElementPtr &_sdf) {
  // Steering joints
  std::string leftSteeringJointName = model->GetName() + "::" + _sdf->Get<std::string>("front_left_wheel_steering");
  left_steering_joint = model->GetJoint(leftSteeringJointName);
  std::string rightSteeringJointName = model->GetName() + "::" + _sdf->Get<std::string>("front_right_wheel_steering");
  right_steering_joint = model->GetJoint(rightSteeringJointName);

  // Front wheels
  std::string frontLeftWheelName = model->GetName() + "::" + _sdf->Get<std::string>("front_left_wheel");
  front_left_wheel = model->GetJoint(frontLeftWheelName);
  std::string frontRightWheelName = model->GetName() + "::" + _sdf->Get<std::string>("front_right_wheel");
  front_right_wheel = model->GetJoint(frontRightWheelName);

  // Rear wheels
  std::string rearLeftWheelName = model->GetName() + "::" + _sdf->Get<std::string>("rear_left_wheel");
  rear_left_wheel = model->GetJoint(rearLeftWheelName);
  std::string rearRightWheelName = model->GetName() + "::" + _sdf->Get<std::string>("rear_right_wheel");
  rear_right_wheel = model->GetJoint(rearRightWheelName);
}

void VehicleModel::initVehicleParam(sdf::ElementPtr &_sdf) {
  ROS_INFO("RaceCarModelPlugin loading params");

  std::string yaml_name = "config.yaml";
  yaml_name = getParam(_sdf, "yaml_config", yaml_name);

  initParamStruct(param_, yaml_name);
}

// TODO: Update this function
void VehicleModel::printInfo() {}

void VehicleModel::update(const double dt) {
  input_.dc = ros::Time::now().toSec() - time_last_cmd_ < 1.0 ? input_.dc : -1.0;

  // TODO: Check if this should always be the case
  left_steering_joint->SetPosition(0, input_.delta);
  right_steering_joint->SetPosition(0, input_.delta);

  State new_state = state_;
  Input new_input = input_;

  updateState(new_state, new_input, dt);

  state_ = new_state;
  input_ = new_input;

  setModelState();

  // Publish Everything
  publishCarState();
  publishWheelSpeeds();
  publishOdom();

  // TODO: Only do this if it is selected in the launcher
  if (this->publish_tf_) {
    publishTf();
  }

  state_machine_.spinOnce();
}

void VehicleModel::updateState(State& state, Input& input, const double dt) {}

void VehicleModel::setModelState() {
  const ignition::math::Pose3d   pose(state_.x, state_.y, model->WorldPose().Pos().Z(), 0, 0.0, state_.yaw);
  const ignition::math::Vector3d vel(state_.v_x * cos(state_.yaw) - state_.v_y * sin(state_.yaw), state_.v_x * sin (state_.yaw) + state_.v_y * cos(state_.yaw), 0.0);
  const ignition::math::Vector3d angular(0.0, 0.0, state_.r);
  model->SetWorldPose(pose);
  model->SetAngularVel(angular);
  model->SetLinearVel(vel);
}

// TODO: Implement publishCarState function properly
void VehicleModel::publishCarState() {
  // Publish Car Info
  eufs_msgs::CarState car_state;
  car_state.header.stamp = ros::Time::now();

  // TODO: Check what the child_frame_id of the car state should be
  car_state.child_frame_id = "eufs";

  car_state.pose.pose.position.x = this->state_.x + this->GaussianKernel(0, this->position_noise_[0]);
  car_state.pose.pose.position.y = this->state_.y + this->GaussianKernel(0, this->position_noise_[1]);
  car_state.pose.pose.position.z = model->WorldPose().Pos().Z() + this->GaussianKernel(0, this->position_noise_[2]);

  std::vector<double> orientation = {0.0, 0.0, state_.yaw};
  orientation = this->ToQuaternion(orientation);

  car_state.pose.pose.orientation.x = orientation[0] + this->GaussianKernel(0, this->orientation_noise_[0]);
  car_state.pose.pose.orientation.y = orientation[1] + this->GaussianKernel(0, this->orientation_noise_[1]);
  car_state.pose.pose.orientation.z = orientation[2] + this->GaussianKernel(0, this->orientation_noise_[2]);
  car_state.pose.pose.orientation.w = orientation[3] + this->GaussianKernel(0, this->orientation_noise_[3]);

  car_state.twist.twist.linear.x = state_.v_x + this->GaussianKernel(0, this->linear_velocity_noise_[0]);
  car_state.twist.twist.linear.y = state_.v_y + this->GaussianKernel(0, this->linear_velocity_noise_[1]);
  car_state.twist.twist.linear.z = 0 + this->GaussianKernel(0, this->linear_velocity_noise_[2]);
  car_state.twist.twist.angular.x = 0 + this->GaussianKernel(0, this->angular_velocity_noise_[0]);
  car_state.twist.twist.angular.y = 0 + this->GaussianKernel(0, this->angular_velocity_noise_[1]);
  car_state.twist.twist.angular.z = state_.r + this->GaussianKernel(0, this->angular_velocity_noise_[2]);

  // TODO: Make sure I don't need this
//  // now rotate linear velocities to correct orientation
//  auto q0 = car_state.pose.pose.orientation.w;
//  auto q1 = car_state.pose.pose.orientation.x;
//  auto q2 = car_state.pose.pose.orientation.y;
//  auto q3 = car_state.pose.pose.orientation.z;
//  auto yaw = atan2(2 * q1 * q2 + 2 * q0 * q3, q1 * q1 + q0 * q0 - q3 * q3 - q2 * q2);
//  auto new_x_vel =
//    cos(yaw) * car_state.twist.twist.linear.x + sin(yaw) * car_state.twist.twist.linear.y;
//  auto new_y_vel =
//    -sin(yaw) * car_state.twist.twist.linear.x + cos(yaw) * car_state.twist.twist.linear.y;
//  car_state.twist.twist.linear.x = new_x_vel;
//  car_state.twist.twist.linear.y = new_y_vel;

  // fill in covariance matrix
  car_state.pose.covariance[0] = pow(this->position_noise_[0], 2);
  car_state.pose.covariance[7] = pow(this->position_noise_[1], 2);
  car_state.pose.covariance[14] = pow(this->position_noise_[2], 2);
  car_state.pose.covariance[21] = pow(this->orientation_noise_[1], 2);
  car_state.pose.covariance[28] = pow(this->orientation_noise_[1], 2);
  car_state.pose.covariance[35] = pow(this->orientation_noise_[2], 2);

  car_state.twist.covariance[0] = pow(this->linear_velocity_noise_[0], 2);
  car_state.twist.covariance[7] = pow(this->linear_velocity_noise_[1], 2);
  car_state.twist.covariance[14] = pow(this->linear_velocity_noise_[2], 2);
  car_state.twist.covariance[21] = pow(this->angular_velocity_noise_[0], 2);
  car_state.twist.covariance[28] = pow(this->angular_velocity_noise_[1], 2);
  car_state.twist.covariance[35] = pow(this->angular_velocity_noise_[2], 2);


  // TODO: set linear_acceleration
  car_state.linear_acceleration.x = 0 + this->GaussianKernel(0, this->linear_acceleration_noise_[0]);
  car_state.linear_acceleration.y = 0 + this->GaussianKernel(0, this->linear_acceleration_noise_[1]);
  car_state.linear_acceleration.z = 0 + this->GaussianKernel(0, this->linear_acceleration_noise_[2]);

  car_state.linear_acceleration_covariance[0] = pow(this->linear_acceleration_noise_[0], 2);
  car_state.linear_acceleration_covariance[4] = pow(this->linear_acceleration_noise_[1], 2);
  car_state.linear_acceleration_covariance[8] = pow(this->linear_acceleration_noise_[2], 2);

  // geometry_msgs/Vector3 linear_acceleration # m/s^2
  geometry_msgs::Vector3 linear_acceleration;
  car_state.linear_acceleration = linear_acceleration;

  car_state.slip_angle = getSlipAngle();

  car_state.state_of_charge = 999;

  pub_car_state_.publish(car_state);
}

// TODO: Check if these formulas are correct
double VehicleModel::getSlipAngle(bool isFront) {
  unsigned int id = 0;

  double lever_arm_length_ = param_.kinematic.l * param_.kinematic.w_front;

  if (!isFront) {
    double axle_width_ = (rear_left_wheel->GetChild()->GetCollision(id)->WorldPose().Pos() -
                          rear_right_wheel->GetChild()->GetCollision(id)->WorldPose().Pos()).Length();

    // From Ignat
    // return -std::atan2(state_.v_y - state_.r * lever_arm_length_, state_.v_x);

    // From fssim
    double v_x = std::max(1.0, state_.v_x);
    return std::atan((state_.v_y + -1 * lever_arm_length_ * state_.r) / (v_x - 0.5 * axle_width_ * state_.r));
  }

  double axle_width_ = (front_left_wheel->GetChild()->GetCollision(id)->WorldPose().Pos() -
                        front_right_wheel->GetChild()->GetCollision(id)->WorldPose().Pos()).Length();

  // From Ignat
  // return input_.delta - std::atan2(state_.v_y + state_.r * lever_arm_length_, state_.v_x);

  // From fssim
  double v_x = std::max(1.0, state_.v_x);
  return std::atan((state_.v_y + lever_arm_length_ * state_.r) / (v_x - 0.5 * axle_width_ * state_.r)) - input_.delta;
}

void VehicleModel::publishWheelSpeeds() {
  eufs_msgs::WheelSpeedsStamped wheel_speeds;

  wheel_speeds.steering = input_.delta;

  // TODO: Change these to a different value?
  wheel_speeds.lf_speed = 0;
  wheel_speeds.rf_speed = 0;

  float PI = 3.14159265;
  float wheel_circumference = PI * param_.tire.radius;

  wheel_speeds.lb_speed = (state_.v_x / wheel_circumference) * 60;
  wheel_speeds.rb_speed = (state_.v_x / wheel_circumference) * 60;

  pub_wheel_speeds_.publish(wheel_speeds);
}

void VehicleModel::publishOdom() {
  nav_msgs::Odometry odom;

  odom.header.stamp = ros::Time::now();

  // TODO: Check what the child_frame_id of the car state should be
  odom.child_frame_id = "eufs";

  odom.pose.pose.position.x = this->state_.x + this->GaussianKernel(0, this->position_noise_[0]);
  odom.pose.pose.position.y = this->state_.y + this->GaussianKernel(0, this->position_noise_[1]);
  odom.pose.pose.position.z = model->WorldPose().Pos().Z() + this->GaussianKernel(0, this->position_noise_[2]);

  std::vector<double> orientation = {0.0, 0.0, state_.yaw};
  orientation = this->ToQuaternion(orientation);

  odom.pose.pose.orientation.x = orientation[0] + this->GaussianKernel(0, this->orientation_noise_[0]);
  odom.pose.pose.orientation.y = orientation[1] + this->GaussianKernel(0, this->orientation_noise_[1]);
  odom.pose.pose.orientation.z = orientation[2] + this->GaussianKernel(0, this->orientation_noise_[2]);
  odom.pose.pose.orientation.w = orientation[3] + this->GaussianKernel(0, this->orientation_noise_[3]);

  odom.twist.twist.linear.x = state_.v_x + this->GaussianKernel(0, this->linear_velocity_noise_[0]);
  odom.twist.twist.linear.y = state_.v_y + this->GaussianKernel(0, this->linear_velocity_noise_[1]);
  odom.twist.twist.linear.z = 0 + this->GaussianKernel(0, this->linear_velocity_noise_[2]);
  odom.twist.twist.angular.x = 0 + this->GaussianKernel(0, this->angular_velocity_noise_[0]);
  odom.twist.twist.angular.y = 0 + this->GaussianKernel(0, this->angular_velocity_noise_[1]);
  odom.twist.twist.angular.z = state_.r + this->GaussianKernel(0, this->angular_velocity_noise_[2]);

  // TODO: Make sure I don't need this
//  // now rotate linear velocities to correct orientation
//  auto q0 = odom.pose.pose.orientation.w;
//  auto q1 = odom.pose.pose.orientation.x;
//  auto q2 = odom.pose.pose.orientation.y;
//  auto q3 = odom.pose.pose.orientation.z;
//  auto yaw = atan2(2 * q1 * q2 + 2 * q0 * q3, q1 * q1 + q0 * q0 - q3 * q3 - q2 * q2);
//  auto new_x_vel =
//    cos(yaw) * odom.twist.twist.linear.x + sin(yaw) * odom.twist.twist.linear.y;
//  auto new_y_vel =
//    -sin(yaw) * odom.twist.twist.linear.x + cos(yaw) * odom.twist.twist.linear.y;
//  odom.twist.twist.linear.x = new_x_vel;
//  odom.twist.twist.linear.y = new_y_vel;

  // fill in covariance matrix
  odom.pose.covariance[0] = pow(this->position_noise_[0], 2);
  odom.pose.covariance[7] = pow(this->position_noise_[1], 2);
  odom.pose.covariance[14] = pow(this->position_noise_[2], 2);
  odom.pose.covariance[21] = pow(this->orientation_noise_[1], 2);
  odom.pose.covariance[28] = pow(this->orientation_noise_[1], 2);
  odom.pose.covariance[35] = pow(this->orientation_noise_[2], 2);

  odom.twist.covariance[0] = pow(this->linear_velocity_noise_[0], 2);
  odom.twist.covariance[7] = pow(this->linear_velocity_noise_[1], 2);
  odom.twist.covariance[14] = pow(this->linear_velocity_noise_[2], 2);
  odom.twist.covariance[21] = pow(this->angular_velocity_noise_[0], 2);
  odom.twist.covariance[28] = pow(this->angular_velocity_noise_[1], 2);
  odom.twist.covariance[35] = pow(this->angular_velocity_noise_[2], 2);

  pub_odom_.publish(odom);
}

void VehicleModel::publishTf() {
  // Position
  tf::Transform transform;
  transform.setOrigin(tf::Vector3(state_.x, state_.y, 0.0));

  // Orientation
  tf::Quaternion q;
  q.setRPY(0.0, 0.0, state_.yaw);
  transform.setRotation(q);

  // TODO: Add noise to the odom message

  // Send TF
  tf_br_.sendTransform(tf::StampedTransform(transform, ros::Time::now(), this->reference_frame_, this->robot_frame_));
}

void VehicleModel::onCmd(const ackermann_msgs::AckermannDriveStampedConstPtr &msg) {
  if (state_machine_.canDrive()) {
    input_.delta = msg->drive.steering_angle;

    // The input delta is capped at the max_steering angle
    if (input_.delta < -param_.tire.max_steering) {
      input_.delta = -param_.tire.max_steering;
    } else if (input_.delta > param_.tire.max_steering) {
      input_.delta = param_.tire.max_steering;
    }

    input_.dc    = msg->drive.acceleration;
  } else {
    // TODO: Stop the car somehow
    input_.delta = 0;
    input_.dc    = -1;
  }

  time_last_cmd_ = ros::Time::now().toSec();
}

void VehicleModel::onInitialPose(const geometry_msgs::PoseWithCovarianceStamped &msg) {
  state_.x   = msg.pose.pose.position.x;
  state_.y   = msg.pose.pose.position.y;
  state_.yaw = tf::getYaw(msg.pose.pose.orientation);
  state_.v_x = state_.v_y = state_.r = state_.a_x = state_.a_y = 0.0;
}

double VehicleModel::GaussianKernel(double mu, double sigma) {
  // using Box-Muller transform to generate two independent standard
  // normally distributed normal variables see wikipedia

  // normalized uniform random variable
  double U = static_cast<double>(rand_r(&seed)) /
    static_cast<double>(RAND_MAX);

  // normalized uniform random variable
  double V = static_cast<double>(rand_r(&seed)) /
    static_cast<double>(RAND_MAX);

  double X = sqrt(-2.0 * ::log(U)) * cos(2.0 * M_PI * V);
  // double Y = sqrt(-2.0 * ::log(U)) * sin(2.0*M_PI * V);

  // there are 2 indep. vars, we'll just use X
  // scale to our mu and sigma
  X = sigma * X + mu;
  return X;
}

} // namespace fssim
} // namespace gazebo
