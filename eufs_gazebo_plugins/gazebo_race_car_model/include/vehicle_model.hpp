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

// FSSIM Includes
#include "axle.hpp"
#include "aero.hpp"

// ROS Msgs
#include "eufs_msgs/CarState.h"
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

  void publish(double sim_time);

 protected:

  virtual void updateState(const double dt) = 0;

  void setPositionFromWorld();

  void publishTf(const State &x);

  void publishCarState();

  double getFx(const State &x, const Input &u);

  double getMTv(const State &x, const Input &u) const;

  void onCmd(const ackermann_msgs::AckermannDriveStampedConstPtr &msg);

  void onInitialPose(const geometry_msgs::PoseWithCovarianceStamped &msg);

  void initModel(sdf::ElementPtr &_sdf);

  void initVehicleParam(sdf::ElementPtr &_sdf);

  double getNormalForce(const State &x);

  void setModelState(const State &x);

  double getGaussianNoise(double mean, double var) const;

  State &getState() { return state_; }

  Input &getInput() { return input_; }

 protected:

  // ROS Nodehandle
  boost::shared_ptr<ros::NodeHandle> nh_;

  // ROS Publishrs
  ros::Publisher pub_car_state_;

  // ROS Subscribers
  ros::Subscriber sub_cmd_;
  ros::Subscriber sub_initial_pose_;

  // ROS TF
  tf::TransformBroadcaster tf_br_;

  /// Pointer to the parent model
  physics::ModelPtr model;

  /// Chassis link and Base Link
  physics::LinkPtr chassisLink;
  physics::LinkPtr base_link_;

  // Front and Rear Axle
  FrontAxle front_axle_;
  RearAxle  rear_axle_;

  // Parameters
  Param param_;

  // States
  State state_;
  Input input_;
  double time_last_cmd_;

  // Consider Aerodynamics
  Aero aero_;

  // Name of the System
  std::string robot_name_;

  // Gaussian Kernel for random number generation
  unsigned int seed;
  double GaussianKernel(double mu, double sigma);
};

typedef std::unique_ptr<VehicleModel> VehicleModelPtr;

} // namespace fssim
} // namespace gazebo

#endif //FSSIM_GAZEBO_VEHICLE_H
