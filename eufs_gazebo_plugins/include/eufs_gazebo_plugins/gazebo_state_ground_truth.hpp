/*
 * Copyright 2012 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

/*
 * Desc: 3D position interface for ground truth.
 * Author: Sachin Chitta and John Hsu
 * Date: 1 June 2008
 */

#ifndef GAZEBO_ROS_P3D_HH
#define GAZEBO_ROS_P3D_HH

#include <string>

#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>

#include <ros/callback_queue.h>
#include <ros/advertise_options.h>

#include <gazebo/physics/physics.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>

#include <gazebo_plugins/PubQueue.h>

namespace gazebo {
class GazeboStateGroundTruth : public ModelPlugin {
  
 public:
  /// \brief Constructor
  GazeboStateGroundTruth();
  virtual ~GazeboStateGroundTruth();

  /// \brief Load the controller
  void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

 protected:
  /// \brief Update the controller
  virtual void UpdateChild();

 private:
  physics::WorldPtr world_;
  physics::ModelPtr model_;

  /// \brief The parent Model
  physics::LinkPtr link_;

  /// \brief The body of the frame to display pose, twist
  physics::LinkPtr reference_link_;

  /// \brief pointer to ros node
  ros::NodeHandle *rosnode_;
  ros::Publisher pub_;
  PubQueue<nav_msgs::Odometry>::Ptr pub_Queue;

  /// \brief ros message
  nav_msgs::Odometry pose_msg_;

  /// \brief store bodyname
  std::string link_name_;

  /// \brief topic name
  std::string topic_name_;

  /// \brief frame transform name, should match link name
  /// FIXME: extract link name directly?
  std::string frame_name_;
  std::string tf_frame_name_;

  /// \brief allow specifying constant xyz and rpy offsets
  bool got_offset_;
  ignition::math::Pose3d offset_;

  /// \brief mutex to lock access to fields used in message callbacks
  boost::mutex lock;

  /// \brief save last_time
  common::Time last_time_;
  ignition::math::Vector3d last_vpos_;
  ignition::math::Vector3d last_veul_;
  ignition::math::Vector3d apos_;
  ignition::math::Vector3d aeul_;
  ignition::math::Vector3d last_frame_vpos_;
  ignition::math::Vector3d last_frame_veul_;
  ignition::math::Vector3d frame_apos_;
  ignition::math::Vector3d frame_aeul_;

  // rate control
  double update_rate_;

  /// \brief Gaussian noise
  double gaussian_noise_;

  /// \brief Gaussian noise generator
  double GaussianKernel(double mu, double sigma);

  /// \brief for setting ROS name space
  std::string robot_namespace_;

  ros::CallbackQueue p3d_queue_;
  void P3DQueueThread();
  boost::thread callback_queue_thread_;

  // Pointer to the update event connection
  event::ConnectionPtr update_connection_;

  unsigned int seed;

  // ros publish multi queue, prevents publish() blocking
  PubMultiQueue pmq;
};
}
#endif