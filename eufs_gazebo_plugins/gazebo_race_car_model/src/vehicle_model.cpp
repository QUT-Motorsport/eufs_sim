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
    front_axle_(_model, _sdf, "front", gznode, nh),
    rear_axle_(_model, _sdf, "rear", gznode, nh),
    aero_(param_.aero) {

  // For the Gaussian Kernel random number generation
  seed = 0;

  // ROS Publishers
  pub_car_state_    = nh->advertise<eufs_msgs::CarState>("/ground_truth/state", 1);

  // ROS Subscribers
  sub_cmd_          = nh->subscribe("/fssim/cmd", 1, &VehicleModel::onCmd, this);
  sub_initial_pose_ = nh->subscribe("/initialpose", 1, &VehicleModel::onInitialPose, this);

  // Initializatoin
  initModel(_sdf);
  initVehicleParam(_sdf);

  // Set Axle parameters
  front_axle_.setLeverArm(param_.kinematic.l, 1.0 - param_.kinematic.w_front, param_.kinematic.b_F);
  rear_axle_.setLeverArm(param_.kinematic.l, param_.kinematic.w_front, param_.kinematic.b_R);
  front_axle_.setParam(param_);
  rear_axle_.setParam(param_);

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

  std::string chassisLinkName = model->GetName() + "::" + _sdf->Get<std::string>("chassis");
  getLink(chassisLink, model, chassisLinkName);

  std::string baseLinkName = model->GetName() + "::" + _sdf->Get<std::string>("base_link");
  getLink(base_link_, model, baseLinkName);

  // then the wheelbase is the distance between the axle centers
  auto vec3 = front_axle_.getAxlePos() - rear_axle_.getAxlePos();
  param_.kinematic.l = vec3.Length();
}

void VehicleModel::initVehicleParam(sdf::ElementPtr &_sdf) {
  robot_name_ = getParam<std::string>(_sdf, "robot_name");

  std::string yaml_name = "config.yaml";

  yaml_name = getParam(_sdf, "yaml_config", yaml_name);
  initParam(param_, yaml_name);

  yaml_name = getParam(_sdf, "yaml_sensors", yaml_name);
  initParamSensors(param_, yaml_name);
}

void VehicleModel::publish(const double sim_time) {

}

void VehicleModel::printInfo() {
  front_axle_.printInfo();
  rear_axle_.printInfo();
}

std::ostream &operator<<(std::ostream &os, const State s) {
  os << s.getString();
}

void VehicleModel::update(const double dt) {
  // TODO: Implement some kind of state machine
  input_.dc = ros::Time::now().toSec() - time_last_cmd_ < 1.0 ? input_.dc : -1.0;

  front_axle_.setSteering(input_.delta);

  updateState(dt);

  // Publish Everything
  setModelState(state_);
  publishTf(state_);
  publishCarState();
}

void VehicleModel::publishTf(const State &x) {
  // Position
  tf::Transform transform;
  transform.setOrigin(tf::Vector3(x.x, x.y, 0.0));

  // Orientation
  tf::Quaternion q;
  q.setRPY(0.0, 0.0, x.yaw);
  transform.setRotation(q);

  // Send TF
  tf_br_.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/fssim_map", "/fssim/vehicle/base_link"));
}

double VehicleModel::getFx(const State &x, const Input &u) {
  const double dc = x.v_x <= 0.0 && u.dc < 0.0 ? 0.0 : u.dc;
  const double Fx = dc * param_.driveTrain.cm1 - aero_.getFdrag(x) - param_.driveTrain.cr0;
  return Fx;
}

double VehicleModel::getMTv(const State &x, const Input &u) const {
  const double shrinkage = param_.torqueVectoring.shrinkage;
  const double K_stab    = param_.torqueVectoring.K_stability;
  const double l         = param_.kinematic.l;

  const double delta = u.delta;
  const double v_x   = x.v_x;

  return 0.0;
}

void VehicleModel::onCmd(const ackermann_msgs::AckermannDriveStampedConstPtr &msg) {
  input_.delta = msg->drive.steering_angle;
  input_.dc    = msg->drive.acceleration;
  time_last_cmd_ = ros::Time::now().toSec();
}

void VehicleModel::onInitialPose(const geometry_msgs::PoseWithCovarianceStamped &msg) {
  state_.x   = msg.pose.pose.position.x;
  state_.y   = msg.pose.pose.position.y;
  state_.yaw = tf::getYaw(msg.pose.pose.orientation);
  state_.v_x = state_.v_y = state_.r = state_.a_x = state_.a_y = 0.0;
}

double VehicleModel::getNormalForce(const State &x) {
  return param_.inertia.g * param_.inertia.m + aero_.getFdown(x);
}

void VehicleModel::setModelState(const State &x) {
  const ignition::math::Pose3d    pose(x.x, x.y, 0.0, 0, 0.0, x.yaw);
  const ignition::math::Vector3d vel(x.v_x, x.v_y, 0.0);
  const ignition::math::Vector3d angular(0.0, 0.0, x.r);
  model->SetWorldPose(pose);
  model->SetAngularVel(angular);
  model->SetLinearVel(vel);
}

double VehicleModel::getGaussianNoise(double mean, double var) const {
  std::normal_distribution<double> distribution(mean, var);
  // construct a trivial random generator engine from a time-based seed:
  long                             seed = std::chrono::system_clock::now().time_since_epoch().count();
  std::default_random_engine       generator(seed);
  return distribution(generator);
}

void VehicleModel::publishCarState() {
  // Publish Car Info
  eufs_msgs::CarState car_state;
  car_state.header.stamp = ros::Time::now();

  car_state.child_frame_id = robot_name_;

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

  // Overlay Noise on Velocities
  // auto state_pub = state_.toRos(ros::Time::now());
  // state_pub.vx += GaussianKernel(0.0, param_.sensors.noise_vx_sigma);
  // state_pub.vy += GaussianKernel(0.0, param_.sensors.noise_vy_sigma);
  // state_pub.r += GaussianKernel(0.0, param_.sensors.noise_r_sigma);

  boost::array<double, 9> linear_acceleration_covariance = { 0 };
  // TODO: set linear_acceleration_covariance
  car_state.linear_acceleration_covariance = linear_acceleration_covariance;

  // TODO: set slip_angle
  car_state.slip_angle = 0;

  // TODO: set state_of_charge
  car_state.state_of_charge = 1000;

  pub_car_state_.publish(car_state);
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
