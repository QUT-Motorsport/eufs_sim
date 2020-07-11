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
namespace eufs {

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

  this->setPositionFromWorld();

  this->time_last_cmd_ = 0.0;
}

void VehicleModel::initParam(sdf::ElementPtr &_sdf) {
  if (!_sdf->HasElement("publish_rate")) {
    this->publish_rate_ = 200.0;
  } else {
    this->publish_rate_ = _sdf->GetElement("publish_rate")->Get<double>();
  }

  if (!_sdf->HasElement("referenceFrame")) {
    ROS_DEBUG_NAMED("gazebo_ros_race_car_model", "gazebo_ros_race_car_model plugin missing <referenceFrame>, defaults to map");
    this->reference_frame_ = "map";
  } else {
    this->reference_frame_ = _sdf->GetElement("referenceFrame")->Get<std::string>();
  }

  if (!_sdf->HasElement("robotFrame")) {
    ROS_DEBUG_NAMED("gazebo_ros_race_car_model", "gazebo_ros_race_car_model plugin missing <robotFrame>, defaults to base_footprint");
    this->robot_frame_ = "base_footprint";
  } else {
    this->robot_frame_ = _sdf->GetElement("robotFrame")->Get<std::string>();
  }

  if (!_sdf->HasElement("publishTransform")) {
    ROS_DEBUG_NAMED("gazebo_ros_race_car_model", "gazebo_ros_race_car_model plugin missing <publishTransform>, defaults to false");
    this->publish_tf_ = false;
  } else {
    this->publish_tf_ = _sdf->GetElement("publishTransform")->Get<bool>();
  }

  if (!_sdf->HasElement("wheelSpeedsTopicName")) {
    ROS_FATAL_NAMED("gazebo_ros_race_car_model", "gazebo_ros_race_car_model plugin missing <wheelSpeedsTopicName>, cannot proceed");
    return;
  } else {
    this->wheel_speeds_topic_name_ = _sdf->GetElement("wheelSpeedsTopicName")->Get<std::string>();
  }

  if (!_sdf->HasElement("stateTopicName")) {
    ROS_FATAL_NAMED("gazebo_ros_race_car_model", "gazebo_ros_race_car_model plugin missing <stateTopicName>, cannot proceed");
    return;
  } else {
    this->state_topic_name_ = _sdf->GetElement("stateTopicName")->Get<std::string>();
  }

  if (!_sdf->HasElement("odometryTopicName")) {
    ROS_FATAL_NAMED("gazebo_ros_race_car_model", "gazebo_ros_race_car_model plugin missing <odometryTopicName>, cannot proceed");
    return;
  } else {
    this->odom_topic_name_ = _sdf->GetElement("odometryTopicName")->Get<std::string>();
  }

  if (!_sdf->HasElement("positionNoise")) {
    ROS_DEBUG_NAMED("gazebo_ros_race_car_model",
                    "gazebo_ros_race_car_model plugin missing <positionNoise>, defaults to 0.0, 0.0, 0.0");
    this->position_noise_ = {0.0, 0.0, 0.0};
  } else {
    auto temp = _sdf->GetElement("positionNoise")->Get<ignition::math::Vector3d>();
    this->position_noise_ = {temp.X(), temp.Y(), temp.Z()};
  }

  if (this->position_noise_.size() != 3) {
    ROS_FATAL_NAMED("gazebo_ros_race_car_model", "positionNoise parameter vector is not of size 3");
  }

  if (!_sdf->HasElement("orientationNoise")) {
    ROS_DEBUG_NAMED("gazebo_ros_race_car_model",
                    "gazebo_ros_race_car_model plugin missing <orientationNoise>, defaults to 0.0, 0.0, 0.0");
    this->orientation_noise_ = {0.0, 0.0, 0.0};
  } else {
    auto temp = _sdf->GetElement("orientationNoise")->Get<ignition::math::Vector3d>();
    this->orientation_noise_ = {temp.X(), temp.Y(), temp.Z()};
  }

  if (this->orientation_noise_.size() != 3) {
    ROS_FATAL_NAMED("gazebo_ros_race_car_model", "orientationNoise parameter vector is not of size 3");
  }

  if (!_sdf->HasElement("linearVelocityNoise")) {
    ROS_DEBUG_NAMED("gazebo_ros_race_car_model",
                    "gazebo_ros_race_car_model plugin missing <linearVelocityNoise>, defaults to 0.0, 0.0, 0.0");
    this->linear_velocity_noise_ = {0.0, 0.0, 0.0};
  } else {
    auto temp = _sdf->GetElement("linearVelocityNoise")->Get<ignition::math::Vector3d>();
    this->linear_velocity_noise_ = {temp.X(), temp.Y(), temp.Z()};
  }

  if (this->linear_velocity_noise_.size() != 3) {
    ROS_FATAL_NAMED("gazebo_ros_race_car_model", "linearVelocityNoise parameter vector is not of size 3");
  }

  if (!_sdf->HasElement("angularVelocityNoise")) {
    ROS_DEBUG_NAMED("gazebo_ros_race_car_model",
                    "gazebo_ros_race_car_model plugin missing <angularVelocityNoise>, defaults to 0.0, 0.0, 0.0");
    this->angular_velocity_noise_ = {0.0, 0.0, 0.0};
  } else {
    auto temp = _sdf->GetElement("angularVelocityNoise")->Get<ignition::math::Vector3d>();
    this->angular_velocity_noise_ = {temp.X(), temp.Y(), temp.Z()};
  }

  if (this->angular_velocity_noise_.size() != 3) {
    ROS_FATAL_NAMED("gazebo_ros_race_car_model", "angularVelocityNoise parameter vector is not of size 3");
  }

  if (!_sdf->HasElement("linearAccelerationNoise")) {
    ROS_DEBUG_NAMED("gazebo_ros_race_car_model",
                    "gazebo_ros_race_car_model plugin missing <linearAccelerationNoise>, defaults to 0.0, 0.0, 0.0");
    this->linear_acceleration_noise_ = {0.0, 0.0, 0.0};
  } else {
    auto temp = _sdf->GetElement("linearAccelerationNoise")->Get<ignition::math::Vector3d>();
    this->linear_acceleration_noise_ = {temp.X(), temp.Y(), temp.Z()};
  }

  if (this->linear_acceleration_noise_.size() != 3) {
    ROS_FATAL_NAMED("gazebo_ros_race_car_model", "linearAccelerationNoise parameter vector is not of size 3");
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
  q[0] = cy * cp * sr - sy * sp * cr; // x
  q[1] = sy * cp * sr + cy * sp * cr; // y
  q[2] = sy * cp * cr - cy * sp * sr; // z
  q[3] = cy * cp * cr + sy * sp * sr; // w

  return q;
}


void VehicleModel::setPositionFromWorld() {
#if GAZEBO_MAJOR_VERSION >= 8
  auto       pos   = model->WorldPose();
  const auto vel   = model->WorldLinearVel();
  const auto accel = model->WorldLinearAccel();
  const auto r     = model->WorldAngularVel();
#else
  const auto pos   = model->GetWorldPose().Ign();
  const auto vel   = model->GetWorldLinearVel().Ign();
  const auto accel = model->GetWorldLinearAccel().Ign();
  const auto r     = model->GetWorldAngularVel().Ign();
#endif

  offset_ = pos;

  ROS_DEBUG_NAMED("gazebo_ros_race_car_model",
                  "Got starting offset %f %f %f",
                  this->offset_.Pos()[0],
                  this->offset_.Pos()[1],
                  this->offset_.Pos()[2]);

  state_.x   = 0.0;
  state_.y   = 0.0;
  state_.yaw = 0.0;
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
  std::string yaml_name = "";

  if (!_sdf->HasElement("yaml_config")) {
    ROS_FATAL_NAMED("gazebo_ros_race_car_model", "gazebo_ros_race_car_model plugin missing <yaml_config>, cannot proceed");
    return;
  } else {
    yaml_name = _sdf->GetElement("yaml_config")->Get<std::string>();
  }

  initParamStruct(param_, yaml_name);

  ROS_INFO("RaceCarModelPlugin finished loading params");
}

void VehicleModel::printInfo() {}

void VehicleModel::update(const double dt) {
  input_.acc = ros::Time::now().toSec() - time_last_cmd_ < 1.0 ? input_.acc : -1.0;

  left_steering_joint->SetPosition(0, input_.delta);
  right_steering_joint->SetPosition(0, input_.delta);

  State new_state = state_;
  Input new_input = input_;

  updateState(new_state, new_input, dt);

  state_ = new_state;
  input_ = new_input;

  setModelState();

  double current_time = ros::Time::now().toSec();

  double time_since_last_published = current_time - this->time_last_published_;

  if (time_since_last_published < (1 / this->publish_rate_)) {
    return;
  }

  this->time_last_published_ = current_time;

  // Publish Everything
  publishCarState();
  publishWheelSpeeds();
  publishOdom();

  if (this->publish_tf_) {
    publishTf();
  }

  state_machine_.spinOnce();
}

void VehicleModel::updateState(State& state, Input& input, const double dt) {}

void VehicleModel::setModelState() {
  double yaw = state_.yaw + offset_.Rot().Yaw();

  double x = offset_.Pos().X() + state_.x * cos(offset_.Rot().Yaw()) - state_.y * sin(offset_.Rot().Yaw());
  double y = offset_.Pos().Y() + state_.x * sin(offset_.Rot().Yaw()) + state_.y * cos(offset_.Rot().Yaw());

#if GAZEBO_MAJOR_VERSION >= 8
  double z = model->WorldPose().Pos().Z();
#else
  double z = model->GetWorldPose().Ign().Pos().Z();
#endif

  double vx = state_.v_x * cos(yaw) - state_.v_y * sin(yaw);
  double vy = state_.v_x * sin(yaw) + state_.v_y * cos(yaw);

  const ignition::math::Pose3d   pose(x, y, z, 0, 0.0, yaw);
  const ignition::math::Vector3d vel(vx, vy, 0.0);
  const ignition::math::Vector3d angular(0.0, 0.0, state_.r);

  model->SetWorldPose(pose);
  model->SetAngularVel(angular);
  model->SetLinearVel(vel);
}

void VehicleModel::publishCarState() {
  // Publish Car Info
  eufs_msgs::CarState car_state;

  car_state.header.stamp = ros::Time::now();
  car_state.header.frame_id = this->reference_frame_;
  car_state.child_frame_id = this->robot_frame_;

#if GAZEBO_MAJOR_VERSION >= 8
  double z = model->WorldPose().Pos().Z();
#else
  double z = model->GetWorldPose().Ign().Pos().Z();
#endif
  car_state.pose.pose.position.x = this->state_.x + this->GaussianKernel(0, this->position_noise_[0]);
  car_state.pose.pose.position.y = this->state_.y + this->GaussianKernel(0, this->position_noise_[1]);
  car_state.pose.pose.position.z = z + this->GaussianKernel(0, this->position_noise_[2]);

  std::vector<double> orientation = {state_.yaw, 0.0, 0.0};

  orientation[0] += this->GaussianKernel(0, this->orientation_noise_[0]);
  orientation[1] += this->GaussianKernel(0, this->orientation_noise_[1]);
  orientation[2] += this->GaussianKernel(0, this->orientation_noise_[2]);

  orientation = this->ToQuaternion(orientation);

  car_state.pose.pose.orientation.x = orientation[0];
  car_state.pose.pose.orientation.y = orientation[1];
  car_state.pose.pose.orientation.z = orientation[2];
  car_state.pose.pose.orientation.w = orientation[3];

  car_state.twist.twist.linear.x = state_.v_x + this->GaussianKernel(0, this->linear_velocity_noise_[0]);
  car_state.twist.twist.linear.y = state_.v_y + this->GaussianKernel(0, this->linear_velocity_noise_[1]);
  car_state.twist.twist.linear.z = 0 + this->GaussianKernel(0, this->linear_velocity_noise_[2]);
  car_state.twist.twist.angular.x = 0 + this->GaussianKernel(0, this->angular_velocity_noise_[0]);
  car_state.twist.twist.angular.y = 0 + this->GaussianKernel(0, this->angular_velocity_noise_[1]);
  car_state.twist.twist.angular.z = state_.r + this->GaussianKernel(0, this->angular_velocity_noise_[2]);

  // fill in covariance matrix
  car_state.pose.covariance[0] = pow(this->position_noise_[0], 2);
  car_state.pose.covariance[7] = pow(this->position_noise_[1], 2);
  car_state.pose.covariance[14] = pow(this->position_noise_[2], 2);
  car_state.pose.covariance[21] = pow(this->orientation_noise_[0], 2);
  car_state.pose.covariance[28] = pow(this->orientation_noise_[1], 2);
  car_state.pose.covariance[35] = pow(this->orientation_noise_[2], 2);

  car_state.twist.covariance[0] = pow(this->linear_velocity_noise_[0], 2);
  car_state.twist.covariance[7] = pow(this->linear_velocity_noise_[1], 2);
  car_state.twist.covariance[14] = pow(this->linear_velocity_noise_[2], 2);
  car_state.twist.covariance[21] = pow(this->angular_velocity_noise_[0], 2);
  car_state.twist.covariance[28] = pow(this->angular_velocity_noise_[1], 2);
  car_state.twist.covariance[35] = pow(this->angular_velocity_noise_[2], 2);

  car_state.linear_acceleration.x = state_.a_x + this->GaussianKernel(0, this->linear_acceleration_noise_[0]);
  car_state.linear_acceleration.y = state_.a_y + this->GaussianKernel(0, this->linear_acceleration_noise_[1]);
  car_state.linear_acceleration.z = 0 + this->GaussianKernel(0, this->linear_acceleration_noise_[2]);

  car_state.linear_acceleration_covariance[0] = pow(this->linear_acceleration_noise_[0], 2);
  car_state.linear_acceleration_covariance[4] = pow(this->linear_acceleration_noise_[1], 2);
  car_state.linear_acceleration_covariance[8] = pow(this->linear_acceleration_noise_[2], 2);

  car_state.slip_angle = getSlipAngle();

  car_state.state_of_charge = 999;

  if (pub_car_state_.getNumSubscribers() > 0) {
    pub_car_state_.publish(car_state);
  }
}

double VehicleModel::getSlipAngle(bool isFront) {
  unsigned int id = 0;

  double lever_arm_length_ = param_.kinematic.l * param_.kinematic.w_front;

  if (!isFront) {
#if GAZEBO_MAJOR_VERSION >= 8
    double axle_width_ = (rear_left_wheel->GetChild()->GetCollision(id)->WorldPose().Pos() -
                          rear_right_wheel->GetChild()->GetCollision(id)->WorldPose().Pos()).Length();
#else
    double axle_width_ = (rear_left_wheel->GetChild()->GetCollision(id)->GetWorldPose().Ign().Pos() -
                          rear_right_wheel->GetChild()->GetCollision(id)->GetWorldPose().Ign().Pos()).Length();
#endif

    double v_x = std::max(1.0, state_.v_x);
    return std::atan((state_.v_y - lever_arm_length_ * state_.r) / (v_x - 0.5 * axle_width_ * state_.r));
  }

#if GAZEBO_MAJOR_VERSION >= 8
  double axle_width_ = (front_left_wheel->GetChild()->GetCollision(id)->WorldPose().Pos() -
                        front_right_wheel->GetChild()->GetCollision(id)->WorldPose().Pos()).Length();
#else
  double axle_width_ = (front_left_wheel->GetChild()->GetCollision(id)->GetWorldPose().Ign().Pos() -
                        front_right_wheel->GetChild()->GetCollision(id)->GetWorldPose().Ign().Pos()).Length();
#endif

  double v_x = std::max(1.0, state_.v_x);
  return std::atan((state_.v_y + lever_arm_length_ * state_.r) / (v_x - 0.5 * axle_width_ * state_.r)) - input_.delta;
}

void VehicleModel::publishWheelSpeeds() {
  eufs_msgs::WheelSpeedsStamped wheel_speeds;
  
  wheel_speeds.header.stamp = ros::Time::now();
  wheel_speeds.header.frame_id = this->robot_frame_;

  wheel_speeds.steering = input_.delta;

  wheel_speeds.lf_speed = 999;
  wheel_speeds.rf_speed = 999;

  float PI = 3.14159265;
  float wheel_circumference = 2 * PI * param_.tire.radius;

  wheel_speeds.lb_speed = (state_.v_x / wheel_circumference) * 60;
  wheel_speeds.rb_speed = (state_.v_x / wheel_circumference) * 60;

  if (pub_wheel_speeds_.getNumSubscribers() > 0) {
    pub_wheel_speeds_.publish(wheel_speeds);
  }
}

void VehicleModel::publishOdom() {
  nav_msgs::Odometry odom;

  odom.header.stamp = ros::Time::now();

  odom.header.frame_id = this->reference_frame_;
  odom.child_frame_id = this->robot_frame_;

#if GAZEBO_MAJOR_VERSION >= 8
  double z = model->WorldPose().Pos().Z();
#else
  double z = model->GetWorldPose().Ign().Pos().Z();
#endif

  odom.pose.pose.position.x = this->state_.x + this->GaussianKernel(0, this->position_noise_[0]);
  odom.pose.pose.position.y = this->state_.y + this->GaussianKernel(0, this->position_noise_[1]);
  odom.pose.pose.position.z = z + this->GaussianKernel(0, this->position_noise_[2]);

  std::vector<double> orientation = {state_.yaw, 0.0, 0.0};

  orientation[0] += this->GaussianKernel(0, this->orientation_noise_[0]);
  orientation[1] += this->GaussianKernel(0, this->orientation_noise_[1]);
  orientation[2] += this->GaussianKernel(0, this->orientation_noise_[2]);

  orientation = this->ToQuaternion(orientation);

  odom.pose.pose.orientation.x = orientation[0];
  odom.pose.pose.orientation.y = orientation[1];
  odom.pose.pose.orientation.z = orientation[2];
  odom.pose.pose.orientation.w = orientation[3];

  odom.twist.twist.linear.x = state_.v_x + this->GaussianKernel(0, this->linear_velocity_noise_[0]);
  odom.twist.twist.linear.y = state_.v_y + this->GaussianKernel(0, this->linear_velocity_noise_[1]);
  odom.twist.twist.linear.z = 0 + this->GaussianKernel(0, this->linear_velocity_noise_[2]);
  odom.twist.twist.angular.x = 0 + this->GaussianKernel(0, this->angular_velocity_noise_[0]);
  odom.twist.twist.angular.y = 0 + this->GaussianKernel(0, this->angular_velocity_noise_[1]);
  odom.twist.twist.angular.z = state_.r + this->GaussianKernel(0, this->angular_velocity_noise_[2]);

  // fill in covariance matrix
  odom.pose.covariance[0] = pow(this->position_noise_[0], 2);
  odom.pose.covariance[7] = pow(this->position_noise_[1], 2);
  odom.pose.covariance[14] = pow(this->position_noise_[2], 2);
  odom.pose.covariance[21] = pow(this->orientation_noise_[0], 2);
  odom.pose.covariance[28] = pow(this->orientation_noise_[1], 2);
  odom.pose.covariance[35] = pow(this->orientation_noise_[2], 2);

  odom.twist.covariance[0] = pow(this->linear_velocity_noise_[0], 2);
  odom.twist.covariance[7] = pow(this->linear_velocity_noise_[1], 2);
  odom.twist.covariance[14] = pow(this->linear_velocity_noise_[2], 2);
  odom.twist.covariance[21] = pow(this->angular_velocity_noise_[0], 2);
  odom.twist.covariance[28] = pow(this->angular_velocity_noise_[1], 2);
  odom.twist.covariance[35] = pow(this->angular_velocity_noise_[2], 2);

  if (pub_odom_.getNumSubscribers() > 0) {
    pub_odom_.publish(odom);
  }
}

void VehicleModel::publishTf() {
  // Position
  tf::Transform transform;
  transform.setOrigin(tf::Vector3(state_.x + this->GaussianKernel(0, this->position_noise_[0]),
                                  state_.y + this->GaussianKernel(0, this->position_noise_[1]),
                                  0.0));

  // Orientation
  tf::Quaternion q;
  q.setRPY(0.0, 0.0, state_.yaw + this->GaussianKernel(0, this->angular_velocity_noise_[2]));
  transform.setRotation(q);

  // Send TF
  tf_br_.sendTransform(tf::StampedTransform(transform, ros::Time::now(), this->reference_frame_, this->robot_frame_));
}

void VehicleModel::onCmd(const ackermann_msgs::AckermannDriveStampedConstPtr &msg) {
  // TODO: Should add delay to the controls
  if (state_machine_.canDrive()) {
    input_.delta = msg->drive.steering_angle;
    input_.acc    = msg->drive.acceleration;
  } else {
    // TODO: Should  do something else to stop the car but is this good for now
    input_.delta = 0;
    input_.acc    = -100;
  }

  input_.validate(param_);

  time_last_cmd_ = ros::Time::now().toSec();
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

} // namespace eufs
} // namespace gazebo
