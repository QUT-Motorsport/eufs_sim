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
#ifndef GAZEBO_VEHICLE_HPP
#define GAZEBO_VEHICLE_HPP

#include <ros/ros.h>

#include <gazebo/physics/physics.hh>

// Include
#include "config.hpp"
#include "definitions.hpp"
#include "state_machine.hpp"

// ROS msgs
#include "eufs_msgs/CarState.h"
#include "eufs_msgs/WheelSpeedsStamped.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/PoseWithCovariance.h"
#include "geometry_msgs/TwistWithCovariance.h"
#include "geometry_msgs/Vector3.h"
#include "ackermann_msgs/AckermannDriveStamped.h"
// ROS
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

namespace gazebo {
namespace eufs {

class VehicleModel {
public:
  VehicleModel(physics::ModelPtr &_model,
          sdf::ElementPtr &_sdf,
          boost::shared_ptr<ros::NodeHandle> &nh,
          transport::NodePtr &gznode);

  void update(double dt);

  void printInfo();

  StateMachine state_machine_;

protected:
  void setPositionFromWorld();
  ignition::math::Pose3d offset_;

  void initParam(sdf::ElementPtr &_sdf);

  void initModel(sdf::ElementPtr &_sdf);

  void initVehicleParam(sdf::ElementPtr &_sdf);

  virtual void updateState(State& state, Input& input, const double dt);

  void setModelState();

  void publishCarState();

  double getSlipAngle(bool isFront = true);

  void publishWheelSpeeds();

  void publishOdom();

  void publishTf();

  void onCmd(const ackermann_msgs::AckermannDriveStampedConstPtr &msg);

  State &getState() { return state_; }

  Input &getInput() { return input_; }

  // States
  State state_;
  Input input_;
  double time_last_cmd_;

  // ROS Nodehandle
  boost::shared_ptr<ros::NodeHandle> nh_;

  // Pointer to the parent model
  physics::ModelPtr model;

  // Rate to publish ros messages
  double publish_rate_;
  double time_last_published_;

  std::string state_topic_name_;
  std::string wheel_speeds_topic_name_;
  std::string odom_topic_name_;

  // ROS Publishers
  ros::Publisher pub_car_state_, pub_car_info_;
  ros::Publisher pub_wheel_speeds_;
  ros::Publisher pub_odom_;

  // ROS Subscribers
  ros::Subscriber sub_cmd_;

  bool publish_tf_;
  std::string reference_frame_;
  std::string robot_frame_;

  std::vector<double> position_noise_;
  std::vector<double> orientation_noise_;
  std::vector<double> linear_velocity_noise_;
  std::vector<double> angular_velocity_noise_;
  std::vector<double> linear_acceleration_noise_;

  /// @brief Converts an euler orientation to quaternion
  std::vector<double> ToQuaternion(std::vector<double> &euler);

  // ROS TF
  tf::TransformBroadcaster tf_br_;

  // Gaussian Kernel for random number generation
  unsigned int seed;
  double GaussianKernel(double mu, double sigma);

 protected:
  // Steering joints
  physics::JointPtr left_steering_joint;
  physics::JointPtr right_steering_joint;

  // Wheels
  physics::JointPtr front_left_wheel;
  physics::JointPtr front_right_wheel;
  physics::JointPtr rear_left_wheel;
  physics::JointPtr rear_right_wheel;

  // Parameters
  Param param_;
};

typedef std::unique_ptr<VehicleModel> VehicleModelPtr;

} // namespace eufs
} // namespace gazebo

#endif //GAZEBO_VEHICLE_HPP
