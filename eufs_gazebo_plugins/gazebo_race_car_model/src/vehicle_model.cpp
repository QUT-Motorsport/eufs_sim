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
  seed = 0;

  // ROS Publishers
  pub_car_state_    = nh->advertise<eufs_msgs::CarState>("/ground_truth/state", 1);
  pub_wheel_speeds_  = nh->advertise<eufs_msgs::WheelSpeedsStamped>("/ros_can/wheel_speeds", 1);

  // ROS Subscribers
  sub_cmd_          = nh->subscribe("/cmd_vel_out", 1, &VehicleModel::onCmd, this);
  sub_initial_pose_ = nh->subscribe("/initialpose", 1, &VehicleModel::onInitialPose, this);

  // Initializatoin
  initModel(_sdf);
  initVehicleParam(_sdf);

  setPositionFromWorld();

  time_last_cmd_ = 0.0;
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
  std::string yaml_name = "config.yaml";
  yaml_name = getParam(_sdf, "yaml_config", yaml_name);

  initParam(param_, yaml_name);
}

// TODO: Update this function
void VehicleModel::printInfo() {}

void VehicleModel::update(const double dt) {
  // TODO: Implement some kind of state machine
  input_.dc = ros::Time::now().toSec() - time_last_cmd_ < 1.0 ? input_.dc : -1.0;

  // TODO: create current (state) and input variables to make sure they do not change during the process

  // TODO: Check if this should always be the case
  left_steering_joint->SetPosition(0, input_.delta);
  right_steering_joint->SetPosition(0, input_.delta);

  updateState(dt);

  setModelState();

  // Publish Everything
  publishCarState();
  publishWheelSpeeds();

  // TODO: Only do this if it is selected in the launcher
  publishTf();

  state_machine_.spinOnce();
}

void VehicleModel::updateState(const double dt) {}

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

  geometry_msgs::PoseWithCovariance pose;
  // TODO: set pose
  car_state.pose = pose;

  geometry_msgs::TwistWithCovariance twist;
  // TODO: set twist
  car_state.twist = twist;
  // geometry_msgs/Vector3 linear_acceleration # m/s^2
  geometry_msgs::Vector3 linear_acceleration;
  // TODO: set linear_acceleration
  car_state.linear_acceleration = linear_acceleration;

  // TODO: Overlay Noise on Velocities
  // The param_.sensors does not exist anymore
  // state_pub.vx += GaussianKernel(0.0, param_.sensors.noise_vx_sigma);
  // state_pub.vy += GaussianKernel(0.0, param_.sensors.noise_vy_sigma);
  // state_pub.r += GaussianKernel(0.0, param_.sensors.noise_r_sigma);

  boost::array<double, 9> linear_acceleration_covariance = { 0 };
  // TODO: set linear_acceleration_covariance
  car_state.linear_acceleration_covariance = linear_acceleration_covariance;

  car_state.slip_angle = getSlipAngle();

  // TODO: set state_of_charge
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

// TODO: Implement publishWheelSpeeds function
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

void VehicleModel::publishTf() {
  // Position
  tf::Transform transform;
  transform.setOrigin(tf::Vector3(state_.x, state_.y, 0.0));

  // Orientation
  tf::Quaternion q;
  q.setRPY(0.0, 0.0, state_.yaw);
  transform.setRotation(q);

  // Send TF
  tf_br_.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/fssim_map", "/fssim/vehicle/base_link"));
}

void VehicleModel::onCmd(const ackermann_msgs::AckermannDriveStampedConstPtr &msg) {
  if (state_machine_.canDrive()) {
    input_.delta = msg->drive.steering_angle;
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
