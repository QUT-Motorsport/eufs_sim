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
 * @file gazebo_state_ground_truth.cpp
 * @author Ignat Georgiev <ignat.m.georgiev@gmail.com>
 * @date Dec 05, 2019
 * @copyright 2019 Edinburgh University Formula Student (EUFS)
 * @brief ground truth state Gazebo plugin
 *
 * @details Provides ground truth state in simulation in the form of nav_msgs/Odometry and
 * eufs_msgs/CarState. Additionally can publish transform.
 **/

#include "eufs_gazebo_plugins/gazebo_state_ground_truth.hpp"

namespace gazebo {
GZ_REGISTER_MODEL_PLUGIN(GazeboStateGroundTruth);

GazeboStateGroundTruth::GazeboStateGroundTruth() {
  this->seed = 0;
}

GazeboStateGroundTruth::~GazeboStateGroundTruth() {
  this->update_connection_.reset();
  this->rosnode_->shutdown();
  this->odom_queue_.clear();
  this->odom_queue_.disable();
  this->odom_queue_thread_.join();
  this->state_queue_.clear();
  this->state_queue_.disable();
  this->state_queue_thread_.join();
  delete this->rosnode_;
}

void GazeboStateGroundTruth::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) {
  // Get the world name.
  this->world_ = _parent->GetWorld();
  this->model_ = _parent;

  // load parameters
  this->robot_namespace_ = "";
  if (_sdf->HasElement("robotNamespace"))
    this->robot_namespace_ =
        _sdf->GetElement("robotNamespace")->Get<std::string>() + "/";

  if (!_sdf->HasElement("robotFrame")) {
    ROS_FATAL_NAMED("state_ground_truth", "state_ground_truth plugin missing <robotFrame>, cannot proceed");
    return;
  } else
    this->robot_link_name_ = _sdf->GetElement("robotFrame")->Get<std::string>();

  this->robot_link_ = _parent->GetLink(this->robot_link_name_);
  if (!this->robot_link_) {
    ROS_FATAL_NAMED("state_ground_truth", "state_ground_truth plugin error: robotFrame: %s does not exist\n",
                    this->robot_link_name_.c_str());
    return;
  }

  if (!_sdf->HasElement("referenceFrame")) {
    ROS_DEBUG_NAMED("state_ground_truth", "state_ground_truth plugin missing <referenceFrame>, defaults to map");
    this->frame_name_ = "map";
  } else
    this->frame_name_ = _sdf->GetElement("referenceFrame")->Get<std::string>();

  if (!_sdf->HasElement("publishTransform")) {
    ROS_DEBUG_NAMED("state_ground_truth", "state_ground_truth plugin missing <publishTransform>, defaults to false");
    this->publish_tf_ = false;
  } else
    this->publish_tf_ = _sdf->GetElement("publishTransform")->Get<bool>();

  if (!_sdf->HasElement("odometryTopicName")) {
    ROS_FATAL_NAMED("state_ground_truth", "state_ground_truth plugin missing <odometryTopicName>, cannot proceed");
    return;
  } else
    this->odom_topic_name_ = _sdf->GetElement("odometryTopicName")->Get<std::string>();

  if (!_sdf->HasElement("stateTopicName")) {
    ROS_FATAL_NAMED("state_ground_truth", "state_ground_truth plugin missing <carStateTopicName>, cannot proceed");
    return;
  } else
    this->state_topic_name_ = _sdf->GetElement("stateTopicName")->Get<std::string>();

  if (!_sdf->HasElement("positionNoise")) {
    ROS_DEBUG_NAMED("state_ground_truth",
                    "state_ground_truth plugin missing <positionNoise>, defaults to 0.0, 0.0, 0.0");
    this->position_noise_ = {0.0, 0.0, 0.0};
  } else {
    auto temp = _sdf->GetElement("positionNoise")->Get<ignition::math::Vector3d>();
    this->position_noise_ = {temp.X(), temp.Y(), temp.Z()};
  }

  if (this->position_noise_.size() != 3)
    ROS_FATAL_NAMED("state_ground_truth", "positionNoise parameter vector is not of size 3");

  if (!_sdf->HasElement("orientationNoise")) {
    ROS_DEBUG_NAMED("state_ground_truth",
                    "state_ground_truth plugin missing <orientationNoise>, defaults to 0.0, 0.0, 0.0");
    this->orientation_noise_ = {0.0, 0.0, 0.0};
  } else {
    auto temp = _sdf->GetElement("orientationNoise")->Get<ignition::math::Vector3d>();
    this->orientation_noise_ = {temp.X(), temp.Y(), temp.Z()};
  }

  if (this->orientation_noise_.size() != 3)
    ROS_FATAL_NAMED("state_ground_truth", "orientationNoise parameter vector is not of size 3");

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

  if (this->linear_velocity_noise_.size() != 3)
    ROS_FATAL_NAMED("state_ground_truth", "linearVelocityNoise parameter vector is not of size 3");

  if (!_sdf->HasElement("angularVelocityNoise")) {
    ROS_DEBUG_NAMED("state_ground_truth",
                    "state_ground_truth plugin missing <angularVelocityNoise>, defaults to 0.0, 0.0, 0.0");
    this->angular_velocity_noise_ = {0.0, 0.0, 0.0};
  } else {
    auto temp = _sdf->GetElement("angularVelocityNoise")->Get<ignition::math::Vector3d>();
    this->angular_velocity_noise_ = {temp.X(), temp.Y(), temp.Z()};
  }

  if (this->angular_velocity_noise_.size() != 3)
    ROS_FATAL_NAMED("state_ground_truth", "angularVelocityNoise parameter vector is not of size 3");

  if (!_sdf->HasElement("linearAccelerationNoise")) {
    ROS_DEBUG_NAMED("state_ground_truth",
                    "state_ground_truth plugin missing <linearAccelerationNoise>, defaults to 0.0, 0.0, 0.0");
    this->linear_acceleration_noise_ = {0.0, 0.0, 0.0};
  } else {
    auto temp = _sdf->GetElement("linearAccelerationNoise")->Get<ignition::math::Vector3d>();
    this->linear_acceleration_noise_ = {temp.X(), temp.Y(), temp.Z()};
  }

  if (this->linear_acceleration_noise_.size() != 3)
    ROS_FATAL_NAMED("state_ground_truth", "linearAccelerationNoise parameter vector is not of size 3");

  if (!_sdf->HasElement("updateRate")) {
    ROS_DEBUG_NAMED("state_ground_truth", "state_ground_truth plugin missing <updateRate>, defaults to 0.0"
                                          " (as fast as possible)");
    this->update_rate_ = 0;
  } else
    this->update_rate_ = _sdf->GetElement("updateRate")->Get<double>();

  // Initialise the static fields of publishes messages
  this->odom_msg_.header.frame_id = this->frame_name_;
  this->odom_msg_.child_frame_id = this->robot_link_name_;
  this->state_msg_.header.frame_id = this->frame_name_;
  this->state_msg_.child_frame_id = this->robot_link_name_;

  // Make sure the ROS node for Gazebo has already been initialized
  if (!ros::isInitialized()) {
    ROS_FATAL_STREAM_NAMED("state_ground_truth",
                           "A ROS node for Gazebo has not been initialized, unable to load plugin. "
                               << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }

  this->rosnode_ = new ros::NodeHandle(this->robot_namespace_);

  // publish multi queue
  this->pmq.startServiceThread();

  // resolve tf prefix
  std::string prefix;
  this->rosnode_->getParam(std::string("tf_prefix"), prefix);

  if (this->odom_topic_name_ != "" && this->state_topic_name_ != "") {
    this->odom_pub_queue_ = this->pmq.addPub<nav_msgs::Odometry>();
    this->state_pub_queue_ = this->pmq.addPub<eufs_msgs::CarState>();
    this->odom_pub_ = this->rosnode_->advertise<nav_msgs::Odometry>(this->odom_topic_name_, 1);
    this->state_pub_ = this->rosnode_->advertise<eufs_msgs::CarState>(this->state_topic_name_, 1);
  }

#if GAZEBO_MAJOR_VERSION >= 8
  this->last_time_ = this->world_->SimTime();
#else
  this->last_time_ = this->world_->GetSimTime();
#endif
  // initialize body
#if GAZEBO_MAJOR_VERSION >= 8
  this->last_vpos_ = this->robot_link_->WorldLinearVel();
  this->last_veul_ = this->robot_link_->WorldAngularVel();
#else
  this->last_vpos_ = this->robot_link_->GetWorldLinearVel().Ign();
  this->last_veul_ = this->robot_link_->GetWorldAngularVel().Ign();
#endif
  this->apos_ = 0;
  this->aeul_ = 0;

  // if frameName specified is "/world", "world", "/map" or "map" report
  // back inertial values in the gazebo world
  if (this->frame_name_ != "/world" &&
      this->frame_name_ != "world" &&
      this->frame_name_ != "/map" &&
      this->frame_name_ != "map") {
    this->reference_link_ = this->model_->GetLink(this->frame_name_);
    if (!this->reference_link_) {
      ROS_ERROR_NAMED("state_ground_truth", "state_ground_truth plugin: frameName: %s does not exist, will"
                                            " not publish pose\n", this->frame_name_.c_str());
      return;
    }
  }

  // init reference frame state
  if (this->reference_link_) {
    ROS_DEBUG_NAMED("state_ground_truth", "got body %s", this->reference_link_->GetName().c_str());
    this->frame_apos_ = 0;
    this->frame_aeul_ = 0;
#if GAZEBO_MAJOR_VERSION >= 8
    this->last_frame_vpos_ = this->reference_link_->WorldLinearVel();
    this->last_frame_veul_ = this->reference_link_->WorldAngularVel();
#else
    this->last_frame_vpos_ = this->reference_link_->GetWorldLinearVel().Ign();
    this->last_frame_veul_ = this->reference_link_->GetWorldAngularVel().Ign();
#endif
  }

  // start custom queue for p3d
  this->odom_queue_thread_ = boost::thread(
      boost::bind(&GazeboStateGroundTruth::OdomQueueThread, this));

  this->state_queue_thread_ = boost::thread(
      boost::bind(&GazeboStateGroundTruth::StateQueueThread, this));

  // New Mechanism for Updating every World Cycle
  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->update_connection_ = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&GazeboStateGroundTruth::UpdateChild, this));
}

// Update the controller
void GazeboStateGroundTruth::UpdateChild() {

  if (!this->robot_link_)
    return;

  if (!this->got_offset_) {
    // init reference position (aka. offset)
#if GAZEBO_MAJOR_VERSION >= 8
    this->offset_ = this->robot_link_->WorldPose();
#else
    this->offset_ = this->robot_link_->GetWorldPose().Ign();
#endif
    ROS_DEBUG_NAMED("state_ground_truth",
                    "Got starting offset %f %f %f",
                    this->offset_.Pos()[0],
                    this->offset_.Pos()[1],
                    this->offset_.Pos()[2]);
    this->got_offset_ = true;
  }

#if GAZEBO_MAJOR_VERSION >= 8
  common::Time cur_time = this->world_->SimTime();
#else
  common::Time cur_time = this->world_->GetSimTime();
#endif

  if (cur_time < this->last_time_) {
    ROS_WARN_NAMED("p3d", "Negative update time difference detected.");
    this->last_time_ = cur_time;
  }

  // rate control
  if (this->update_rate_ > 0 &&
      (cur_time - this->last_time_).Double() < (1.0 / this->update_rate_))
    return;

  if (this->odom_pub_.getNumSubscribers() > 0 || this->state_pub_.getNumSubscribers() > 0 || this->publish_tf_) {
    // differentiate to get accelerations
    double tmp_dt = cur_time.Double() - this->last_time_.Double();
    if (tmp_dt != 0) {
      this->lock.lock();

      if (this->odom_topic_name_ != "") {
        // copy data into pose message
        this->odom_msg_.header.stamp.nsec = cur_time.nsec;
        this->odom_msg_.header.stamp.sec = cur_time.sec;

        ignition::math::Pose3d pose, frame_pose;
        ignition::math::Vector3d frame_vpos;
        ignition::math::Vector3d frame_veul;

        // get inertial Rates
        // Get Pose/Orientation
#if GAZEBO_MAJOR_VERSION >= 8
        ignition::math::Vector3d vpos = this->robot_link_->WorldLinearVel();
        ignition::math::Vector3d veul = this->robot_link_->WorldAngularVel();
        pose = this->robot_link_->WorldPose();
#else
        ignition::math::Vector3d vpos = this->robot_link_->GetWorldLinearVel().Ign();
        ignition::math::Vector3d veul = this->robot_link_->GetWorldAngularVel().Ign();
        pose = this->robot_link_->GetWorldPose().Ign();
#endif

        // Apply Reference Frame
        if (this->reference_link_) {
          // convert to relative pose, rates
#if GAZEBO_MAJOR_VERSION >= 8
          frame_pose = this->reference_link_->WorldPose();
          frame_vpos = this->reference_link_->WorldLinearVel();
          frame_veul = this->reference_link_->WorldAngularVel();
#else
          frame_pose = this->reference_link_->GetWorldPose().Ign();
          frame_vpos = this->reference_link_->GetWorldLinearVel().Ign();
          frame_veul = this->reference_link_->GetWorldAngularVel().Ign();
#endif
          pose.Pos() = pose.Pos() - frame_pose.Pos();
          pose.Pos() = frame_pose.Rot().RotateVectorReverse(pose.Pos());
          pose.Rot() *= frame_pose.Rot().Inverse();

          vpos = frame_pose.Rot().RotateVector(vpos - frame_vpos);
          veul = frame_pose.Rot().RotateVector(veul - frame_veul);
        }

        // Apply Constant Offsets
        // apply xyz offsets and get position and rotation components
        pose.Pos() = pose.Pos() - this->offset_.Pos();
        // apply rpy offsets
        pose.Rot() = pose.Rot() * this->offset_.Rot();
        pose.Rot().Normalize();

        // compute accelerations
        this->apos_ = (this->last_vpos_ - vpos) / tmp_dt;
        this->aeul_ = (this->last_veul_ - veul) / tmp_dt;
        this->last_vpos_ = vpos;
        this->last_veul_ = veul;

        this->frame_apos_ = (this->last_frame_vpos_ - frame_vpos) / tmp_dt;
        this->frame_aeul_ = (this->last_frame_veul_ - frame_veul) / tmp_dt;
        this->last_frame_vpos_ = frame_vpos;
        this->last_frame_veul_ = frame_veul;

        // Create nav_msgs/Odometry . It is always needed
        this->odom_msg_.pose.pose.position.x = pose.Pos().X() + this->GaussianKernel(0, this->position_noise_[0]);
        this->odom_msg_.pose.pose.position.y = pose.Pos().Y() + this->GaussianKernel(0, this->position_noise_[1]);
        this->odom_msg_.pose.pose.position.z = pose.Pos().Z() + this->GaussianKernel(0, this->position_noise_[2]);

        this->odom_msg_.pose.pose.orientation.x = pose.Rot().X() + this->GaussianKernel(0, this->orientation_noise_[0]);
        this->odom_msg_.pose.pose.orientation.y = pose.Rot().Y() + this->GaussianKernel(0, this->orientation_noise_[1]);
        this->odom_msg_.pose.pose.orientation.z = pose.Rot().Z() + this->GaussianKernel(0, this->orientation_noise_[2]);
        this->odom_msg_.pose.pose.orientation.w = pose.Rot().W() + this->GaussianKernel(0, this->orientation_noise_[3]);

        this->odom_msg_.twist.twist.linear.x = vpos.X() +
            this->GaussianKernel(0, this->linear_velocity_noise_[0]);
        this->odom_msg_.twist.twist.linear.y = vpos.Y() +
            this->GaussianKernel(0, this->linear_velocity_noise_[1]);
        this->odom_msg_.twist.twist.linear.z = vpos.Z() +
            this->GaussianKernel(0, this->linear_velocity_noise_[2]);
        this->odom_msg_.twist.twist.angular.x = veul.X() +
            this->GaussianKernel(0, this->angular_velocity_noise_[0]);
        this->odom_msg_.twist.twist.angular.y = veul.Y() +
            this->GaussianKernel(0, this->angular_velocity_noise_[1]);
        this->odom_msg_.twist.twist.angular.z = veul.Z() +
            this->GaussianKernel(0, this->angular_velocity_noise_[2]);

        // now rotate linear velocities to correct orientation
        auto q0 = this->odom_msg_.pose.pose.orientation.w;
        auto q1 = this->odom_msg_.pose.pose.orientation.x;
        auto q2 = this->odom_msg_.pose.pose.orientation.y;
        auto q3 = this->odom_msg_.pose.pose.orientation.z;
        auto yaw = atan2(2 * q1 * q2 + 2 * q0 * q3, q1 * q1 + q0 * q0 - q3 * q3 - q2 * q2);
        auto new_x_vel =
            cos(yaw) * this->odom_msg_.twist.twist.linear.x + sin(yaw) * this->odom_msg_.twist.twist.linear.y;
        auto new_y_vel =
            -sin(yaw) * this->odom_msg_.twist.twist.linear.x + cos(yaw) * this->odom_msg_.twist.twist.linear.y;
        this->odom_msg_.twist.twist.linear.x = new_x_vel;
        this->odom_msg_.twist.twist.linear.y = new_y_vel;

        // fill in covariance matrix
        this->odom_msg_.pose.covariance[0] = pow(this->position_noise_[0], 2);
        this->odom_msg_.pose.covariance[7] = pow(this->position_noise_[1], 2);
        this->odom_msg_.pose.covariance[14] = pow(this->position_noise_[2], 2);
        this->odom_msg_.pose.covariance[21] = pow(this->orientation_noise_[1], 2);
        this->odom_msg_.pose.covariance[28] = pow(this->orientation_noise_[1], 2);
        this->odom_msg_.pose.covariance[35] = pow(this->orientation_noise_[2], 2);

        this->odom_msg_.twist.covariance[0] = pow(this->linear_velocity_noise_[0], 2);
        this->odom_msg_.twist.covariance[7] = pow(this->linear_velocity_noise_[1], 2);
        this->odom_msg_.twist.covariance[14] = pow(this->linear_velocity_noise_[2], 2);
        this->odom_msg_.twist.covariance[21] = pow(this->angular_velocity_noise_[0], 2);
        this->odom_msg_.twist.covariance[28] = pow(this->angular_velocity_noise_[1], 2);
        this->odom_msg_.twist.covariance[35] = pow(this->angular_velocity_noise_[2], 2);

        // publish Odometry message only if necessary
        if (this->odom_pub_.getNumSubscribers() > 0)
          this->odom_pub_queue_->push(this->odom_msg_, this->odom_pub_);

        // Now make a eufs_msgs/CarState message if necessary
        if (this->state_pub_.getNumSubscribers() > 0) {
          this->state_msg_.header.stamp = this->odom_msg_.header.stamp;
          this->state_msg_.pose = this->odom_msg_.pose;
          this->state_msg_.twist = this->odom_msg_.twist;

          // Handle accelerations
          this->state_msg_.linear_acceleration.x =
              this->apos_.X() + this->GaussianKernel(0, this->linear_acceleration_noise_[0]);
          this->state_msg_.linear_acceleration.y =
              this->apos_.Y() + this->GaussianKernel(0, this->linear_acceleration_noise_[1]);
          this->state_msg_.linear_acceleration.z =
              this->apos_.Z() + this->GaussianKernel(0, this->linear_acceleration_noise_[2]);
          this->state_msg_.linear_acceleration_covariance[0] = pow(this->linear_acceleration_noise_[0], 2);
          this->state_msg_.linear_acceleration_covariance[4] = pow(this->linear_acceleration_noise_[1], 2);
          this->state_msg_.linear_acceleration_covariance[8] = pow(this->linear_acceleration_noise_[2], 2);

          // Calculate slip angle
          this->state_msg_.slip_angle =
              -atan2(this->state_msg_.twist.twist.linear.y, this->state_msg_.twist.twist.linear.x);

          // Battery charge is currently not simulated so just put in incorrect values
          this->state_msg_.state_of_charge = 99999;

          // publish to ros
          this->state_pub_queue_->push(this->state_msg_, this->state_pub_);
        }

        if (publish_tf_) {
          this->PublishTransform(this->odom_msg_);
        }
      }

      this->lock.unlock();

      // save last time stamp
      this->last_time_ = cur_time;
    }
  }
}

double GazeboStateGroundTruth::GaussianKernel(double mu, double sigma) {
  // using Box-Muller transform to generate two independent standard
  // normally distributed normal variables see wikipedia

  // normalized uniform random variable
  double U = static_cast<double>(rand_r(&this->seed)) /
      static_cast<double>(RAND_MAX);

  // normalized uniform random variable
  double V = static_cast<double>(rand_r(&this->seed)) /
      static_cast<double>(RAND_MAX);

  double X = sqrt(-2.0 * ::log(U)) * cos(2.0 * M_PI * V);
  // double Y = sqrt(-2.0 * ::log(U)) * sin(2.0*M_PI * V);

  // there are 2 indep. vars, we'll just use X
  // scale to our mu and sigma
  X = sigma * X + mu;
  return X;
}

void GazeboStateGroundTruth::PublishTransform(const nav_msgs::Odometry &odom_msg) {
  geometry_msgs::TransformStamped transform;
  transform.header = odom_msg.header;
  transform.child_frame_id = odom_msg.child_frame_id;
  transform.transform.translation.x = odom_msg.pose.pose.position.x;
  transform.transform.translation.y = odom_msg.pose.pose.position.y;
  transform.transform.translation.z = odom_msg.pose.pose.position.z;
  transform.transform.rotation.x = odom_msg.pose.pose.orientation.x;
  transform.transform.rotation.y = odom_msg.pose.pose.orientation.y;
  transform.transform.rotation.z = odom_msg.pose.pose.orientation.z;
  transform.transform.rotation.w = odom_msg.pose.pose.orientation.w;
  this->tf_broadcaster_.sendTransform(transform);
}

std::vector<double> GazeboStateGroundTruth::ToQuaternion(std::vector<double> &euler) {
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

void GazeboStateGroundTruth::OdomQueueThread() {
  static const double timeout = 0.01;

  while (this->rosnode_->ok()) {
    this->odom_queue_.callAvailable(ros::WallDuration(timeout));
  }
}

void GazeboStateGroundTruth::StateQueueThread() {
  static const double timeout = 0.01;

  while (this->rosnode_->ok()) {
    this->state_queue_.callAvailable(ros::WallDuration(timeout));
  }
}

}