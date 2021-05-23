/*
 * AMZ-Driverless
 * Copyright (c) 2018 Authors:
 *   - Juraj Kabzan <kabzanj@gmail.com>
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

#ifndef GAZEBO_ROS_RACE_CAR_HPP
#define GAZEBO_ROS_RACE_CAR_HPP

#include <memory>

// ROS Includes
#include "rclcpp/rclcpp.hpp"

// ROS msgs
#include "eufs_msgs/msg/ackermann_drive_stamped.hpp"
#include "eufs_msgs/msg/car_state.hpp"
#include "eufs_msgs/msg/wheel_speeds_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance.hpp"
#include "geometry_msgs/msg/twist_with_covariance.hpp"
#include "geometry_msgs/msg/vector3.hpp"

// ROS TF2
#include <tf2/transform_datatypes.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/utils.h>

// ROS  srvs
#include <std_srvs/srv/trigger.hpp>

// Gazebo Includes
#include <gazebo/common/Time.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo_ros/node.hpp>

// EUFS includes
#include "state_machine.hpp"
#include "eufs_models/vehicle_model.hpp"
#include "eufs_models/dynamic_bicycle.hpp"
#include "eufs_models/point_mass.hpp"

namespace gazebo_plugins
{
  namespace eufs_plugins
  {

    class RaceCarModelPlugin : public gazebo::ModelPlugin
    {
    public:
      RaceCarModelPlugin();

      ~RaceCarModelPlugin() override;

      void Reset() override;

      void Load(gazebo::physics::ModelPtr _parent, sdf::ElementPtr _sdf) override;

      eufs::models::State &getState() { return state_; }
      eufs::models::Input &getInput() { return input_; }

    private:
      void update();
      void updateState(double dt, gazebo::common::Time current_time);

      void InitVehicleModel(const sdf::ElementPtr &sdf);
      void InitParams(const sdf::ElementPtr &sdf);
      void InitModel(const sdf::ElementPtr &sdf);

      void publishCarState();
      void publishWheelSpeeds();
      void publishOdom();
      void publishTf();

      void onCmd(const eufs_msgs::msg::AckermannDriveStamped::SharedPtr msg);

      /// @brief Converts an euler orientation to quaternion
      std::vector<double> ToQuaternion(std::vector<double> &euler);

      double getSlipAngle(bool isFront = true);

      // Gaussian Kernel for random number generation
      unsigned int seed = 0;
      double GaussianKernel(double mu, double sigma);

      std::shared_ptr<rclcpp::Node> rosnode;
      eufs::models::VehicleModelPtr vehicle;

      void setPositionFromWorld();
      bool resetVehiclePosition(std::shared_ptr<std_srvs::srv::Trigger::Request> request, std::shared_ptr<std_srvs::srv::Trigger::Response> response);
      void setModelState();

      // States
      std::unique_ptr<StateMachine> state_machine_;
      eufs::models::State state_;
      eufs::models::Input input_;
      double time_last_cmd_;
      ignition::math::Pose3d offset_;

      // Gazebo
      gazebo::physics::WorldPtr world;
      gazebo::physics::ModelPtr model;
      gazebo::transport::NodePtr gznode;
      gazebo::event::ConnectionPtr updateConnection;
      gazebo::common::Time lastSimTime;
      gazebo::transport::PublisherPtr worldControlPub;

      // Rate to publish ros messages
      double update_rate_;
      double publish_rate_;
      gazebo::common::Time time_last_published_;

      // ROS TF
      std::unique_ptr<tf2_ros::TransformBroadcaster> tf_br_;

      // ROS topic parameters
      std::string ground_truth_car_state_topic_;
      std::string localisation_car_state_topic_;
      std::string wheel_speeds_topic_name_;
      std::string ground_truth_wheel_speeds_topic_name_;
      std::string odom_topic_name_;

      // ROS Publishers
      rclcpp::Publisher<eufs_msgs::msg::CarState>::SharedPtr pub_ground_truth_car_state_;
      rclcpp::Publisher<eufs_msgs::msg::CarState>::SharedPtr pub_localisation_car_state_;
      rclcpp::Publisher<eufs_msgs::msg::WheelSpeedsStamped>::SharedPtr pub_wheel_speeds_;
      rclcpp::Publisher<eufs_msgs::msg::WheelSpeedsStamped>::SharedPtr pub_ground_truth_wheel_speeds_;
      rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_;

      // ROS Subscriptions
      rclcpp::Subscription<eufs_msgs::msg::AckermannDriveStamped>::SharedPtr sub_cmd_;

      // ROS Services
      rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_vehicle_pos_srv;

      // Frame parameters
      bool publish_tf_;
      std::string reference_frame_;
      std::string robot_frame_;

      // Noise parameters
      std::vector<double> position_noise_;
      std::vector<double> orientation_noise_;
      std::vector<double> linear_velocity_noise_;
      std::vector<double> angular_velocity_noise_;
      std::vector<double> linear_acceleration_noise_;

      // Steering jointsState
      gazebo::physics::JointPtr left_steering_joint;
      gazebo::physics::JointPtr right_steering_joint;

      // Wheels
      gazebo::physics::JointPtr front_left_wheel;
      gazebo::physics::JointPtr front_right_wheel;
      gazebo::physics::JointPtr rear_left_wheel;
      gazebo::physics::JointPtr rear_right_wheel;

      enum CommandMode
      {
        acceleration,
        velocity
      };
      CommandMode command_mode_;
    };

  } // namespace eufs_plugins
} // namespace gazebo_plugins
#endif // GAZEBO_ROS_RACE_CAR_HPP
