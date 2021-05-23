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

// Main Include
#include "gazebo_race_car_model/gazebo_ros_race_car.hpp"

// STD Include
#include <algorithm>
#include <fstream>
#include <mutex>
#include <thread>

namespace gazebo_plugins
{
  namespace eufs_plugins
  {

    RaceCarModelPlugin::RaceCarModelPlugin() {}

    RaceCarModelPlugin::~RaceCarModelPlugin()
    {
      this->updateConnection.reset();
    }

    void RaceCarModelPlugin::Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
      this->rosnode = gazebo_ros::Node::Get(_sdf);

      RCLCPP_DEBUG(this->rosnode->get_logger(), "Loading RaceCarModelPlugin");

      this->model = _model;
      this->world = this->model->GetWorld();

      this->tf_br_ = std::make_unique<tf2_ros::TransformBroadcaster>(this->rosnode);
      this->state_machine_ = std::make_unique<StateMachine>(this->rosnode);

      // Initialize parameters
      InitParams(_sdf);

      // Initialize vehicle model
      InitVehicleModel(_sdf);

      // Initialize handles to Gazebo vehicle components
      InitModel(_sdf);

      // ROS Publishers
      this->pub_ground_truth_car_state_ = rosnode->create_publisher<eufs_msgs::msg::CarState>(this->ground_truth_car_state_topic_, 1);
      this->pub_localisation_car_state_ = rosnode->create_publisher<eufs_msgs::msg::CarState>(this->localisation_car_state_topic_, 1);
      this->pub_wheel_speeds_ = rosnode->create_publisher<eufs_msgs::msg::WheelSpeedsStamped>(this->wheel_speeds_topic_name_, 1);
      this->pub_ground_truth_wheel_speeds_ = rosnode->create_publisher<eufs_msgs::msg::WheelSpeedsStamped>(this->ground_truth_wheel_speeds_topic_name_, 1);
      this->pub_odom_ = rosnode->create_publisher<nav_msgs::msg::Odometry>(this->odom_topic_name_, 1);

      // ROS Services
      this->reset_vehicle_pos_srv = rosnode->create_service<std_srvs::srv::Trigger>("/ros_can/reset_vehicle_pos", std::bind(&RaceCarModelPlugin::resetVehiclePosition, this, std::placeholders::_1, std::placeholders::_2));

      // ROS Subscriptions
      this->sub_cmd_ = rosnode->create_subscription<eufs_msgs::msg::AckermannDriveStamped>("/cmd", 1, std::bind(&RaceCarModelPlugin::onCmd, this, std::placeholders::_1));

      // Connect to Gazebo
      this->updateConnection =
          gazebo::event::Events::ConnectWorldUpdateBegin(std::bind(&RaceCarModelPlugin::update, this));
      this->lastSimTime = this->world->SimTime();

      // Set offset
      this->setPositionFromWorld();
      this->time_last_cmd_ = 0.0;

      RCLCPP_INFO(this->rosnode->get_logger(), "RaceCarModelPlugin Loaded");
    }

    void RaceCarModelPlugin::InitParams(const sdf::ElementPtr &sdf)
    {
      if (!sdf->HasElement("update_rate"))
      {
        this->update_rate_ = 1000.0;
      }
      else
      {
        this->update_rate_ = sdf->GetElement("update_rate")->Get<double>();
      }

      if (!sdf->HasElement("publish_rate"))
      {
        this->publish_rate_ = 200.0;
      }
      else
      {
        this->publish_rate_ = sdf->GetElement("publish_rate")->Get<double>();
      }

      if (!sdf->HasElement("referenceFrame"))
      {
        RCLCPP_DEBUG(this->rosnode->get_logger(), "gazebo_ros_race_car_model plugin missing <referenceFrame>, defaults to map");
        this->reference_frame_ = "map";
      }
      else
      {
        this->reference_frame_ = sdf->GetElement("referenceFrame")->Get<std::string>();
      }

      if (!sdf->HasElement("robotFrame"))
      {
        RCLCPP_DEBUG(this->rosnode->get_logger(), "gazebo_ros_race_car_model plugin missing <robotFrame>, defaults to base_footprint");
        this->robot_frame_ = "base_footprint";
      }
      else
      {
        this->robot_frame_ = sdf->GetElement("robotFrame")->Get<std::string>();
      }

      if (!sdf->HasElement("publishTransform"))
      {
        RCLCPP_DEBUG(this->rosnode->get_logger(), "gazebo_ros_race_car_model plugin missing <publishTransform>, defaults to false");
        this->publish_tf_ = false;
      }
      else
      {
        this->publish_tf_ = sdf->GetElement("publishTransform")->Get<bool>();
      }

      if (!sdf->HasElement("wheelSpeedsTopicName"))
      {
        RCLCPP_FATAL(this->rosnode->get_logger(), "gazebo_ros_race_car_model plugin missing <wheelSpeedsTopicName>, cannot proceed");
        return;
      }
      else
      {
        this->wheel_speeds_topic_name_ = sdf->GetElement("wheelSpeedsTopicName")->Get<std::string>();
      }

      if (!sdf->HasElement("groundTruthWheelSpeedsTopicName"))
      {
        RCLCPP_FATAL(this->rosnode->get_logger(), "gazebo_ros_race_car_model plugin missing <groundTruthWheelSpeedsTopicName>, cannot proceed");
        return;
      }
      else
      {
        this->ground_truth_wheel_speeds_topic_name_ = sdf->GetElement("groundTruthWheelSpeedsTopicName")->Get<std::string>();
      }

      if (!sdf->HasElement("groundTruthCarStateTopic"))
      {
        RCLCPP_FATAL(this->rosnode->get_logger(), "gazebo_ros_race_car_model plugin missing <groundTruthCarStateTopic>, cannot proceed");
        return;
      }
      else
      {
        this->ground_truth_car_state_topic_ = sdf->GetElement("groundTruthCarStateTopic")->Get<std::string>();
      }

      if (!sdf->HasElement("localisationCarStateTopic"))
      {
        RCLCPP_FATAL(this->rosnode->get_logger(), "gazebo_ros_race_car_model plugin missing <localisationCarStateTopic>, cannot proceed");
        return;
      }
      else
      {
        this->localisation_car_state_topic_ = sdf->GetElement("localisationCarStateTopic")->Get<std::string>();
      }

      if (!sdf->HasElement("odometryTopicName"))
      {
        RCLCPP_FATAL(this->rosnode->get_logger(), "gazebo_ros_race_car_model plugin missing <odometryTopicName>, cannot proceed");
        return;
      }
      else
      {
        this->odom_topic_name_ = sdf->GetElement("odometryTopicName")->Get<std::string>();
      }

      if (!sdf->HasElement("positionNoise"))
      {
        RCLCPP_DEBUG(this->rosnode->get_logger(), "gazebo_ros_race_car_model plugin missing <positionNoise>, defaults to 0.0, 0.0, 0.0");
        this->position_noise_ = {0.0, 0.0, 0.0};
      }
      else
      {
        auto temp = sdf->GetElement("positionNoise")->Get<ignition::math::Vector3d>();
        this->position_noise_ = {temp.X(), temp.Y(), temp.Z()};
      }

      if (this->position_noise_.size() != 3)
      {
        RCLCPP_FATAL(this->rosnode->get_logger(), "positionNoise parameter vector is not of size 3");
      }

      if (!sdf->HasElement("orientationNoise"))
      {
        RCLCPP_DEBUG(this->rosnode->get_logger(), "gazebo_ros_race_car_model plugin missing <orientationNoise>, defaults to 0.0, 0.0, 0.0");
        this->orientation_noise_ = {0.0, 0.0, 0.0};
      }
      else
      {
        auto temp = sdf->GetElement("orientationNoise")->Get<ignition::math::Vector3d>();
        this->orientation_noise_ = {temp.X(), temp.Y(), temp.Z()};
      }

      if (this->orientation_noise_.size() != 3)
      {
        RCLCPP_FATAL(this->rosnode->get_logger(), "orientationNoise parameter vector is not of size 3");
      }

      if (!sdf->HasElement("linearVelocityNoise"))
      {
        RCLCPP_DEBUG(this->rosnode->get_logger(), "gazebo_ros_race_car_model plugin missing <linearVelocityNoise>, defaults to 0.0, 0.0, 0.0");
        this->linear_velocity_noise_ = {0.0, 0.0, 0.0};
      }
      else
      {
        auto temp = sdf->GetElement("linearVelocityNoise")->Get<ignition::math::Vector3d>();
        this->linear_velocity_noise_ = {temp.X(), temp.Y(), temp.Z()};
      }

      if (this->linear_velocity_noise_.size() != 3)
      {
        RCLCPP_FATAL(this->rosnode->get_logger(), "linearVelocityNoise parameter vector is not of size 3");
      }

      if (!sdf->HasElement("angularVelocityNoise"))
      {
        RCLCPP_DEBUG(this->rosnode->get_logger(), "gazebo_ros_race_car_model plugin missing <angularVelocityNoise>, defaults to 0.0, 0.0, 0.0");
        this->angular_velocity_noise_ = {0.0, 0.0, 0.0};
      }
      else
      {
        auto temp = sdf->GetElement("angularVelocityNoise")->Get<ignition::math::Vector3d>();
        this->angular_velocity_noise_ = {temp.X(), temp.Y(), temp.Z()};
      }

      if (this->angular_velocity_noise_.size() != 3)
      {
        RCLCPP_FATAL(this->rosnode->get_logger(), "angularVelocityNoise parameter vector is not of size 3");
      }

      if (!sdf->HasElement("linearAccelerationNoise"))
      {
        RCLCPP_DEBUG(this->rosnode->get_logger(), "gazebo_ros_race_car_model plugin missing <linearAccelerationNoise>, defaults to 0.0, 0.0, 0.0");
        this->linear_acceleration_noise_ = {0.0, 0.0, 0.0};
      }
      else
      {
        auto temp = sdf->GetElement("linearAccelerationNoise")->Get<ignition::math::Vector3d>();
        this->linear_acceleration_noise_ = {temp.X(), temp.Y(), temp.Z()};
      }

      if (this->linear_acceleration_noise_.size() != 3)
      {
        RCLCPP_FATAL(this->rosnode->get_logger(), "linearAccelerationNoise parameter vector is not of size 3");
      }

      if (!sdf->HasElement("commandMode"))
      {
        RCLCPP_DEBUG(this->rosnode->get_logger(), "gazebo_ros_race_car_model plugin missing <commandMode>, defaults to acceleration");
        this->command_mode_ = acceleration;
      }
      else
      {
        auto temp = sdf->GetElement("commandMode")->Get<std::string>();
        if (temp.compare("acceleration") == 0)
        {
          this->command_mode_ = acceleration;
        }
        else if (temp.compare("velocity") == 0)
        {
          this->command_mode_ = velocity;
        }
        else
        {
          RCLCPP_WARN(this->rosnode->get_logger(), "commandMode parameter string is invalid, defaults to acceleration");
          this->command_mode_ = acceleration;
        }
      }
    }

    void RaceCarModelPlugin::InitVehicleModel(const sdf::ElementPtr &sdf)
    {
      // Get the vehicle model from the sdf
      std::string vehicle_model_ = "";
      if (!sdf->HasElement("vehicle_model"))
      {
        vehicle_model_ = "DynamicBicycle";
      }
      else
      {
        vehicle_model_ = sdf->GetElement("vehicle_model")->Get<std::string>();
      }

      std::string yaml_name = "";
      if (!sdf->HasElement("yaml_config"))
      {
        RCLCPP_FATAL(this->rosnode->get_logger(), "gazebo_ros_race_car_model plugin missing <yaml_config>, cannot proceed");
        return;
      }
      else
      {
        yaml_name = sdf->GetElement("yaml_config")->Get<std::string>();
      }

      RCLCPP_DEBUG(this->rosnode->get_logger(), "RaceCarModelPlugin finished loading params");

      if (vehicle_model_ == "PointMass")
      {
        this->vehicle = std::unique_ptr<eufs::models::VehicleModel>(new eufs::models::PointMass(yaml_name));
      }
      else if (vehicle_model_ == "DynamicBicycle")
      {
        this->vehicle = std::unique_ptr<eufs::models::VehicleModel>(new eufs::models::DynamicBicycle(yaml_name));
      }
      else
      {
        RCLCPP_FATAL(this->rosnode->get_logger(), "gazebo_ros_race_car_model plugin invalid vehicle model, cannot proceed");
        return;
      }
    }

    void RaceCarModelPlugin::InitModel(const sdf::ElementPtr &_sdf)
    {
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

    void RaceCarModelPlugin::setPositionFromWorld()
    {
      offset_ = model->WorldPose();

      RCLCPP_DEBUG(this->rosnode->get_logger(),
                   "Got starting offset %f %f %f",
                   this->offset_.Pos()[0],
                   this->offset_.Pos()[1],
                   this->offset_.Pos()[2]);

      state_.x = 0.0;
      state_.y = 0.0;
      state_.yaw = 0.0;
      state_.v_x = 0.0;
      state_.v_y = 0.0;
      state_.r = 0.0;
      state_.a_x = 0.0;
      state_.a_y = 0.0;
    }

    bool RaceCarModelPlugin::resetVehiclePosition(std::shared_ptr<std_srvs::srv::Trigger::Request>, std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
      state_.x = 0.0;
      state_.y = 0.0;
      state_.yaw = 0.0;
      state_.v_x = 0.0;
      state_.v_y = 0.0;
      state_.r = 0.0;
      state_.a_x = 0.0;
      state_.a_y = 0.0;

      const ignition::math::Vector3d vel(0.0, 0.0, 0.0);
      const ignition::math::Vector3d angular(0.0, 0.0, 0.0);

      model->SetWorldPose(offset_);
      model->SetAngularVel(angular);
      model->SetLinearVel(vel);

      return response->success;
    }

    void RaceCarModelPlugin::setModelState()
    {
      double yaw = state_.yaw + offset_.Rot().Yaw();

      double x = offset_.Pos().X() + state_.x * cos(offset_.Rot().Yaw()) - state_.y * sin(offset_.Rot().Yaw());
      double y = offset_.Pos().Y() + state_.x * sin(offset_.Rot().Yaw()) + state_.y * cos(offset_.Rot().Yaw());
      double z = model->WorldPose().Pos().Z();

      double vx = state_.v_x * cos(yaw) - state_.v_y * sin(yaw);
      double vy = state_.v_x * sin(yaw) + state_.v_y * cos(yaw);

      const ignition::math::Pose3d pose(x, y, z, 0, 0.0, yaw);
      const ignition::math::Vector3d vel(vx, vy, 0.0);
      const ignition::math::Vector3d angular(0.0, 0.0, state_.r);

      model->SetWorldPose(pose);
      model->SetAngularVel(angular);
      model->SetLinearVel(vel);
    }

    void RaceCarModelPlugin::publishCarState()
    {
      // Publish Car Info
      eufs_msgs::msg::CarState car_state;

      car_state.header.stamp = this->rosnode->now();
      car_state.header.frame_id = this->reference_frame_;
      car_state.child_frame_id = this->robot_frame_;

      double z = model->WorldPose().Pos().Z();

      car_state.pose.pose.position.x = this->state_.x;
      car_state.pose.pose.position.y = this->state_.y;
      car_state.pose.pose.position.z = z;

      std::vector<double> orientation = {state_.yaw, 0.0, 0.0};

      orientation = this->ToQuaternion(orientation);

      car_state.pose.pose.orientation.x = orientation[0];
      car_state.pose.pose.orientation.y = orientation[1];
      car_state.pose.pose.orientation.z = orientation[2];
      car_state.pose.pose.orientation.w = orientation[3];

      car_state.twist.twist.linear.x = state_.v_x;
      car_state.twist.twist.linear.y = state_.v_y;
      car_state.twist.twist.linear.z = 0;
      car_state.twist.twist.angular.x = 0;
      car_state.twist.twist.angular.y = 0;
      car_state.twist.twist.angular.z = state_.r;

      car_state.linear_acceleration.x = state_.a_x;
      car_state.linear_acceleration.y = state_.a_y;
      car_state.linear_acceleration.z = 0;

      car_state.slip_angle = state_.slip_angle;

      car_state.state_of_charge = 999;

      // Publish ground_truth
      if (this->pub_ground_truth_car_state_->get_subscription_count() > 0)
      {
        this->pub_ground_truth_car_state_->publish(car_state);
      }

      // Add noise
      car_state.pose.pose.position.x += this->GaussianKernel(0, this->position_noise_[0]);
      car_state.pose.pose.position.y += this->GaussianKernel(0, this->position_noise_[1]);
      car_state.pose.pose.position.z += this->GaussianKernel(0, this->position_noise_[2]);

      // Reset orientation
      orientation = {state_.yaw, 0.0, 0.0};

      orientation[0] += this->GaussianKernel(0, this->orientation_noise_[0]);
      orientation[1] += this->GaussianKernel(0, this->orientation_noise_[1]);
      orientation[2] += this->GaussianKernel(0, this->orientation_noise_[2]);

      orientation = this->ToQuaternion(orientation);

      car_state.pose.pose.orientation.x = orientation[0];
      car_state.pose.pose.orientation.y = orientation[1];
      car_state.pose.pose.orientation.z = orientation[2];
      car_state.pose.pose.orientation.w = orientation[3];

      car_state.twist.twist.linear.x += this->GaussianKernel(0, this->linear_velocity_noise_[0]);
      car_state.twist.twist.linear.y += this->GaussianKernel(0, this->linear_velocity_noise_[1]);
      car_state.twist.twist.linear.z += this->GaussianKernel(0, this->linear_velocity_noise_[2]);
      car_state.twist.twist.angular.x += this->GaussianKernel(0, this->angular_velocity_noise_[0]);
      car_state.twist.twist.angular.y += this->GaussianKernel(0, this->angular_velocity_noise_[1]);
      car_state.twist.twist.angular.z += this->GaussianKernel(0, this->angular_velocity_noise_[2]);

      // Fill in covariance matrix
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

      car_state.linear_acceleration.x += this->GaussianKernel(0, this->linear_acceleration_noise_[0]);
      car_state.linear_acceleration.y += this->GaussianKernel(0, this->linear_acceleration_noise_[1]);
      car_state.linear_acceleration.z += this->GaussianKernel(0, this->linear_acceleration_noise_[2]);

      car_state.linear_acceleration_covariance[0] = pow(this->linear_acceleration_noise_[0], 2);
      car_state.linear_acceleration_covariance[4] = pow(this->linear_acceleration_noise_[1], 2);
      car_state.linear_acceleration_covariance[8] = pow(this->linear_acceleration_noise_[2], 2);

      // Publish with noise
      if (this->pub_localisation_car_state_->get_subscription_count() > 0)
      {
        this->pub_localisation_car_state_->publish(car_state);
      }
    }

    void RaceCarModelPlugin::publishWheelSpeeds()
    {
      eufs_msgs::msg::WheelSpeedsStamped wheel_speeds;

      wheel_speeds.header.stamp = this->rosnode->now();
      wheel_speeds.header.frame_id = this->robot_frame_;

      wheel_speeds.steering = input_.delta;

      wheel_speeds.lf_speed = 999;
      wheel_speeds.rf_speed = 999;

      float PI = 3.14159265;
      float wheel_circumference = 2 * PI * this->vehicle->getParam().tire.radius;

      // Calculate Wheel speeds
      wheel_speeds.lb_speed = (state_.v_x / wheel_circumference) * 60;
      wheel_speeds.rb_speed = (state_.v_x / wheel_circumference) * 60;

      // Publish ground truth
      if (pub_ground_truth_wheel_speeds_->get_subscription_count() > 0)
      {
        pub_ground_truth_wheel_speeds_->publish(wheel_speeds);
      }

      // TODO: Add Noise to Wheel speeds here

      // Publish with Noise
      if (pub_wheel_speeds_->get_subscription_count() > 0)
      {
        pub_wheel_speeds_->publish(wheel_speeds);
      }
    }

    void RaceCarModelPlugin::publishOdom()
    {
      nav_msgs::msg::Odometry odom;

      odom.header.stamp = this->rosnode->now();

      odom.header.frame_id = this->reference_frame_;
      odom.child_frame_id = this->robot_frame_;

      double z = model->WorldPose().Pos().Z();

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

      if (pub_odom_->get_subscription_count() > 0)
      {
        pub_odom_->publish(odom);
      }
    }

    void RaceCarModelPlugin::publishTf()
    {
      // Position
      tf2::Transform transform;
      transform.setOrigin(tf2::Vector3(state_.x + this->GaussianKernel(0, this->position_noise_[0]),
                                       state_.y + this->GaussianKernel(0, this->position_noise_[1]),
                                       0.0));

      // Orientation
      tf2::Quaternion q;
      q.setRPY(0.0, 0.0, state_.yaw + this->GaussianKernel(0, this->angular_velocity_noise_[2]));
      transform.setRotation(q);

      // Send TF
      geometry_msgs::msg::TransformStamped transform_stamped;

      transform_stamped.header.stamp = this->rosnode->now();
      transform_stamped.header.frame_id = this->reference_frame_;
      transform_stamped.child_frame_id = this->robot_frame_;
      tf2::convert(transform, transform_stamped.transform);

      tf_br_->sendTransform(transform_stamped);
    }

    void RaceCarModelPlugin::Reset()
    {
      this->lastSimTime = 0;
    }

    void RaceCarModelPlugin::update()
    {
      gazebo::common::Time curTime = this->world->SimTime();
      double dt = (curTime - this->lastSimTime).Double();
      if (dt < (1 / this->update_rate_))
      {
        return;
      }

      this->lastSimTime = curTime;
      updateState(dt, curTime.Double());
    }

    void RaceCarModelPlugin::updateState(const double dt, gazebo::common::Time current_time)
    {
      if (this->command_mode_ == velocity)
      {
        double current_speed = std::sqrt(std::pow(state_.v_x, 2) + std::pow(state_.v_y, 2));
        input_.acc = (input_.vel - current_speed) / dt;
      }
      input_.acc = this->rosnode->now().seconds() - time_last_cmd_ < 1.0 ? input_.acc : -1.0;

      this->vehicle->updateState(state_, input_, dt);

      left_steering_joint->SetPosition(0, input_.delta);
      right_steering_joint->SetPosition(0, input_.delta);
      setModelState();

      double time_since_last_published = (current_time - this->time_last_published_).Double();
      if (time_since_last_published < (1 / this->publish_rate_))
      {
        return;
      }
      this->time_last_published_ = current_time;

      // Publish Everything
      publishCarState();
      publishWheelSpeeds();
      publishOdom();

      if (this->publish_tf_)
      {
        publishTf();
      }

      state_machine_->spinOnce(current_time);
    }

    void RaceCarModelPlugin::onCmd(const eufs_msgs::msg::AckermannDriveStamped::SharedPtr msg)
    {
      // TODO: Should add delay to the controls
      if (state_machine_->canDrive())
      {
        input_.delta = msg->drive.steering_angle;
        input_.acc = msg->drive.acceleration;
        input_.vel = msg->drive.speed;
      }
      else
      {
        // TODO: Should  do something else to stop the car but is this good for now
        input_.delta = 0;
        input_.acc = -100;
        input_.vel = 0;
      }

      time_last_cmd_ = this->rosnode->now().seconds();
    }

    std::vector<double> RaceCarModelPlugin::ToQuaternion(std::vector<double> &euler)
    {
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

    double RaceCarModelPlugin::GaussianKernel(double mu, double sigma)
    {
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

    GZ_REGISTER_MODEL_PLUGIN(RaceCarModelPlugin)

  } // namespace eufs_plugins
} // namespace gazebo_plugins