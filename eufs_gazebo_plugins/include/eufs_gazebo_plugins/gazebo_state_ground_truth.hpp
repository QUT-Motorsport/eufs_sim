/*MIT License
*
* Copyright (c) 2019 Edinburgh University Formula Student (EUFS)
*
* Permission is hereby granted, free of charge, to any person obtaining a copy
*         of this software and associated documentation files (the "Software"), to deal
* in the Software without restriction, including without limitation the rights
*         to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
*         copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in all
*         copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
*         AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
* SOFTWARE.*/

/**
 * @file gazebo_state_ground_truth.h
 * @author Ignat Georgiev <ignat.m.georgiev@gmail.com>
 * @date Dec 05, 2019
 * @copyright 2019 Edinburgh University Formula Student (EUFS)
 * @brief ground truth state Gazebo plugin
 *
 * @details Provides ground truth state in simulation in the form of nav_msgs/Odometry and
 * eufs_msgs/CarState. Additionally can publish transform.
 **/

#ifndef GAZEBO_STATE_GROUND_TRUTH
#define GAZEBO_STATE_GROUND_TRUTH

#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <eufs_msgs/CarState.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/convert.h>
#include <tf2_ros/transform_broadcaster.h>

#include <ros/callback_queue.h>
#include <ros/advertise_options.h>

#include <gazebo/physics/physics.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>

#include <gazebo_plugins/PubQueue.h>

namespace gazebo {

/**
* @class GazeboStateGroundTruth
* @brief Provides ground truth state in simulation in the form of nav_msgs/Odometry and
* eufs_msgs/CarState. Additionally can publish transform.
*/
class GazeboStateGroundTruth : public ModelPlugin {

 public:
  GazeboStateGroundTruth();     /// @brief Construction
  virtual ~GazeboStateGroundTruth();    /// @brief Destructor

  /// @brief Load the controller (i.e. plugin)
  void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

 protected:
  /// @brief Update the controller (i.e. plugin). This is called periodically by Gazebo
  virtual void UpdateChild();

 private:
  /// @brief for setting ROS name space
  std::string robot_namespace_;

  /// @brief Control rate
  double update_rate_;

  /// @brief Pointer to the update event connection
  event::ConnectionPtr update_connection_;

  /// @brief ROS publish multi queue, prevents publish() blocking
  PubMultiQueue pmq;

  /// @brief the Gazeo world we're operating in
  physics::WorldPtr world_;

  /// @brief the model to which this plugin points
  physics::ModelPtr model_;

  /// @brief The link of the robot
  physics::LinkPtr robot_link_;
  std::string robot_link_name_;

  /// @brief The frame in which to publish data
  physics::LinkPtr reference_link_;

  /// @brief pointer to ros node. In gazebo all plugins operate under the same node
  ros::NodeHandle *rosnode_;

  /// @brief nav_msgs/Odometry publisher
  ros::Publisher odom_pub_;

  /// @brief eufs_msgs/CarState publisher
  ros::Publisher state_pub_;

  /// \brief topic name for nav_msgs/Odometry
  std::string odom_topic_name_;

  /// \brief topic name for eufs_msgs/CarState
  std::string state_topic_name_;

  /// @brief queue for nav_msgs/Odometry messages
  PubQueue<nav_msgs::Odometry>::Ptr odom_pub_queue_;

  /// @brief queue for eufs_msgs/CarState messages
  PubQueue<eufs_msgs::CarState>::Ptr state_pub_queue_;

  /// @brief message to be published. Private variable to save computation.
  nav_msgs::Odometry odom_msg_;

  /// @brief message to be published. Private variable to save computation.
  eufs_msgs::CarState state_msg_;

  /// @brief queues for messages to be publishes
  ros::CallbackQueue odom_queue_, state_queue_;

  /// @brief threads for publishing messages
  boost::thread odom_queue_thread_, state_queue_thread_;

  // Noise parameters
  std::vector<double> position_noise_;
  std::vector<double> orientation_noise_;
  std::vector<double> orientation_quat_noise_;
  std::vector<double> linear_velocity_noise_;
  std::vector<double> angular_velocity_noise_;
  std::vector<double> linear_acceleration_noise_;

  /// @brief reference frame transform name, should match link name
  std::string frame_name_;

  /// @brief if true publishes transform
  bool publish_tf_;

  tf2_ros::TransformBroadcaster tf_broadcaster_;

  /// @brief Offset for initialising the plugin
  bool got_offset_;
  ignition::math::Pose3d offset_;

  /// @brief mutex to lock access to fields used in message callbacks
  boost::mutex lock;

  /// @brief save last_time
  common::Time last_time_;

  // Phyiscal properties from the simulation
  ignition::math::Vector3d last_vpos_;
  ignition::math::Vector3d last_veul_;
  ignition::math::Vector3d apos_;
  ignition::math::Vector3d aeul_;
  ignition::math::Vector3d last_frame_vpos_;
  ignition::math::Vector3d last_frame_veul_;
  ignition::math::Vector3d frame_apos_;
  ignition::math::Vector3d frame_aeul_;

  /// @brief Random seed
  unsigned int seed;

  /// @brief Gaussian noise generator
  double GaussianKernel(double mu, double sigma);

  /// @brief Publishes a transfrom derived from the nav_msgs/Odometry input
  void PublishTransform(const nav_msgs::Odometry &odom_msg);

  /// @brief Converts an euler orientation to quaternion
  std::vector<double> ToQuaternion(std::vector<double> &euler);

  /// @brief Threaded queue to publish nav_msgs/Odometry messages
  void OdomQueueThread();

  /// @brief Threaded queue to publish eufs_msgs/CarState messages
  void StateQueueThread();
};
}
#endif