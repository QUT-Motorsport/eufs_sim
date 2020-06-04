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
#ifndef FSSIM_GAZEBO_VEHICLE_H
#define FSSIM_GAZEBO_VEHICLE_H

#include <ros/ros.h>

#include <gazebo/physics/physics.hh>

// FSSIM Include
#include "gazebo_utills.hpp"
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
template<class VecOut, class VecIn>
VecOut getRosVesMsg(VecIn &in) {
  VecOut out;
  out.x = in.X();
  out.y = in.Y();
  out.z = in.Z();
  return out;
};

namespace fssim {

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

  void initModel(sdf::ElementPtr &_sdf);

  void initVehicleParam(sdf::ElementPtr &_sdf);

  virtual void updateState(const double dt);

  void setModelState();

  void publishCarState();

  double getSlipAngle(bool isFront = true);

  void publishWheelSpeeds();

  void publishTf();

  void onCmd(const ackermann_msgs::AckermannDriveStampedConstPtr &msg);

  void onInitialPose(const geometry_msgs::PoseWithCovarianceStamped &msg);

  State &getState() { return state_; }

  Input &getInput() { return input_; }

protected:

  // ROS Nodehandle
  boost::shared_ptr<ros::NodeHandle> nh_;

  // ROS Publishers
  ros::Publisher pub_car_state_;
  ros::Publisher pub_wheel_speeds_;

  // ROS Subscribers
  ros::Subscriber sub_cmd_;
  ros::Subscriber sub_initial_pose_;

  // ROS TF
  tf::TransformBroadcaster tf_br_;

  /// Pointer to the parent model
  physics::ModelPtr model;

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

  // States
  State state_;
  Input input_;
  double time_last_cmd_;

  // Gaussian Kernel for random number generation
  unsigned int seed;
  double GaussianKernel(double mu, double sigma);
};

typedef std::unique_ptr<VehicleModel> VehicleModelPtr;

} // namespace fssim
} // namespace gazebo

#endif //FSSIM_GAZEBO_VEHICLE_H
