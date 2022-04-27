/*
 * Software License Agreement (MIT License)
 * Copyright (c) 2019 Edinburgh University Formula Student (EUFS)
 * All rights reserved.
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
 *
 * @file state_machine.cpp
 * @author EUFS
 * @date 09/06/2019
 * @copyright MIT License
 * @brief simulated ros_can for Gazebo
 *
 * @details Simulated IO functionality of the ros_can interfacing node
 * https://gitlab.com/eufs/ros_can . This is intended to be used with eufs_sim
 * and controls and interfaces with the ADS-DV car for the FSUK competition
 */

#include "gazebo_race_car_model/state_machine.hpp"

#include <string>
#include <vector>

namespace gazebo_plugins {
namespace eufs_plugins {

StateMachine::StateMachine(std::shared_ptr<rclcpp::Node> rosnode) : rosnode(rosnode) {
  // init state machine state
  as_state_ = eufs_msgs::msg::CanState::AS_OFF;
  ami_state_ = eufs_msgs::msg::CanState::AMI_NOT_SELECTED;
  mission_completed_ = false;
  in_transition_ = false;

  // Subscriptions
  completed_sub_ = rosnode->create_subscription<std_msgs::msg::Bool>(
      "/ros_can/mission_completed", 1,
      std::bind(&StateMachine::completedCallback, this, std::placeholders::_1));

  // Services
  reset_srv_ = rosnode->create_service<std_srvs::srv::Trigger>(
      "/ros_can/reset",
      std::bind(&StateMachine::resetState, this, std::placeholders::_1, std::placeholders::_2));
  ebs_srv_ = rosnode->create_service<std_srvs::srv::Trigger>(
      "/ros_can/ebs",
      std::bind(&StateMachine::requestEBS, this, std::placeholders::_1, std::placeholders::_2));
  set_mission_srv_ = rosnode->create_service<eufs_msgs::srv::SetCanState>(
      "/ros_can/set_mission",
      std::bind(&StateMachine::setMission, this, std::placeholders::_1,  std::placeholders::_2));

  // Publishers
  state_pub_ = rosnode->create_publisher<eufs_msgs::msg::CanState>("/ros_can/state", 1);
  state_pub_str_ = rosnode->create_publisher<std_msgs::msg::String>("/ros_can/state_str", 1);
}

StateMachine::~StateMachine() {}

bool StateMachine::setMission(std::shared_ptr<eufs_msgs::srv::SetCanState::Request> request,
                              std::shared_ptr<eufs_msgs::srv::SetCanState::Response> response) {
  if (ami_state_ != eufs_msgs::msg::CanState::AMI_NOT_SELECTED) {
    RCLCPP_WARN(rosnode->get_logger(),
                "state_machine :: failed to set mission as a mission was set previously.");
    return false;
  }

  switch (request->ami_state) {
    case eufs_msgs::msg::CanState::AMI_ACCELERATION:
      ami_state_ = eufs_msgs::msg::CanState::AMI_ACCELERATION;
      break;
    case eufs_msgs::msg::CanState::AMI_SKIDPAD:
      ami_state_ = eufs_msgs::msg::CanState::AMI_SKIDPAD;
      break;
    case eufs_msgs::msg::CanState::AMI_AUTOCROSS:
      ami_state_ = eufs_msgs::msg::CanState::AMI_AUTOCROSS;
      break;
    case eufs_msgs::msg::CanState::AMI_TRACK_DRIVE:
      ami_state_ = eufs_msgs::msg::CanState::AMI_TRACK_DRIVE;
      break;
    case eufs_msgs::msg::CanState::AMI_AUTONOMOUS_DEMO:
      ami_state_ = eufs_msgs::msg::CanState::AMI_AUTONOMOUS_DEMO;
      break;
    case eufs_msgs::msg::CanState::AMI_ADS_INSPECTION:
      ami_state_ = eufs_msgs::msg::CanState::AMI_ADS_INSPECTION;
      break;
    case eufs_msgs::msg::CanState::AMI_ADS_EBS:
      ami_state_ = eufs_msgs::msg::CanState::AMI_ADS_EBS;
      break;
    case eufs_msgs::msg::CanState::AMI_DDT_INSPECTION_A:
      ami_state_ = eufs_msgs::msg::CanState::AMI_DDT_INSPECTION_A;
      break;
    case eufs_msgs::msg::CanState::AMI_DDT_INSPECTION_B:
      ami_state_ = eufs_msgs::msg::CanState::AMI_DDT_INSPECTION_B;
      break;
    case eufs_msgs::msg::CanState::AMI_JOYSTICK:
      ami_state_ = eufs_msgs::msg::CanState::AMI_JOYSTICK;
      break;
    case eufs_msgs::msg::CanState::AMI_MANUAL:
      ami_state_ = eufs_msgs::msg::CanState::AMI_MANUAL;
      break;
    default:
      ami_state_ = eufs_msgs::msg::CanState::AMI_NOT_SELECTED;
      break;
  }
  response->success = true;
  return response->success;
}

bool StateMachine::resetState(std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                              std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
  (void)request;   // suppress unused parameter warning
  (void)response;  // suppress unused parameter warning
  as_state_ = eufs_msgs::msg::CanState::AS_OFF;
  ami_state_ = eufs_msgs::msg::CanState::AMI_NOT_SELECTED;
  mission_completed_ = false;
  response->success = true;
  in_transition_ = false;
  return response->success;
}

bool StateMachine::requestEBS(std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                              std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
  (void)request;   // suppress unused parameter warning
  (void)response;  // suppress unused parameter warning
  if (ami_state_ == eufs_msgs::msg::CanState::AMI_MANUAL ||
      ami_state_ == eufs_msgs::msg::CanState::AMI_NOT_SELECTED) {
    RCLCPP_WARN(rosnode->get_logger(), "state_machine :: EBS is unavailable in current state");
    return false;
  }
  if (as_state_ == eufs_msgs::msg::CanState::AS_EMERGENCY_BRAKE) {
    RCLCPP_WARN(rosnode->get_logger(), "state_machine :: EBS is already active");
    return false;
  }

  as_state_ = eufs_msgs::msg::CanState::AS_EMERGENCY_BRAKE;
  mission_completed_ = false;
  response->success = true;
  in_transition_ = false;
  return response->success;
}

void StateMachine::updateState(gazebo::common::Time current_time) {
  switch (as_state_) {
    case eufs_msgs::msg::CanState::AS_OFF:
      if (ami_state_ != eufs_msgs::msg::CanState::AMI_NOT_SELECTED &&
          ami_state_ != eufs_msgs::msg::CanState::AMI_MANUAL) {
        // now transition to new state
        as_state_ = eufs_msgs::msg::CanState::AS_READY;
        RCLCPP_DEBUG(rosnode->get_logger(), "state_machine :: switching to AS_READY state");
      }
      break;

    case eufs_msgs::msg::CanState::AS_READY:
      if (!in_transition_) {
        transition_begin_ = current_time.Double();
        in_transition_ = true;
      } else if (current_time.Double() - transition_begin_ >= 5.0) {
        // Transition to driving.
        as_state_ = eufs_msgs::msg::CanState::AS_DRIVING;
        RCLCPP_DEBUG(rosnode->get_logger(), "state_machine :: switching to AS_DRIVING state");
        in_transition_ = false;
      }
      break;

    case eufs_msgs::msg::CanState::AS_DRIVING:
      if (mission_completed_) {
        as_state_ = eufs_msgs::msg::CanState::AS_FINISHED;
        RCLCPP_DEBUG(rosnode->get_logger(), "state_machine :: switching to AS_FINISHED state");
      }
      break;

    case eufs_msgs::msg::CanState::AS_FINISHED:
      // do nothing for now
      break;

    case eufs_msgs::msg::CanState::AS_EMERGENCY_BRAKE:
      // do nothing for now
      break;

      // default: do nothing
  }
}

void StateMachine::publishState() {
  if (state_pub_->get_subscription_count() == 0 && state_pub_str_->get_subscription_count() == 0)
    return;  // do nothing

  // create message
  eufs_msgs::msg::CanState state_msg;
  state_msg.as_state = as_state_;
  state_msg.ami_state = ami_state_;

  if (state_pub_->get_subscription_count() > 0) state_pub_->publish(state_msg);

  if (state_pub_str_->get_subscription_count() > 0)
    state_pub_str_->publish(makeStateString(state_msg));
}

std_msgs::msg::String StateMachine::makeStateString(const eufs_msgs::msg::CanState &state) {
  std::string str1, str2, str3;

  RCLCPP_DEBUG(rosnode->get_logger(), "AS STATE: %d", state.as_state);
  RCLCPP_DEBUG(rosnode->get_logger(), "AMI STATE: %d", state.ami_state);

  switch (state.as_state) {
    case eufs_msgs::msg::CanState::AS_OFF:
      str1 = "AS:OFF";
      break;
    case eufs_msgs::msg::CanState::AS_READY:
      str1 = "AS:READY";
      break;
    case eufs_msgs::msg::CanState::AS_DRIVING:
      str1 = "AS:DRIVING";
      break;
    case eufs_msgs::msg::CanState::AS_FINISHED:
      str1 = "AS:FINISHED";
      break;
    case eufs_msgs::msg::CanState::AS_EMERGENCY_BRAKE:
      str1 = "AS:EMERGENCY";
      break;
    default:
      str1 = "NO_SUCH_MESSAGE";
      break;
  }

  switch (state.ami_state) {
    case eufs_msgs::msg::CanState::AMI_NOT_SELECTED:
      str2 = "AMI:NOT_SELECTED";
      break;
    case eufs_msgs::msg::CanState::AMI_ACCELERATION:
      str2 = "AMI:ACCELERATION";
      break;
    case eufs_msgs::msg::CanState::AMI_SKIDPAD:
      str2 = "AMI:SKIDPAD";
      break;
    case eufs_msgs::msg::CanState::AMI_AUTOCROSS:
      str2 = "AMI:AUTOCROSS";
      break;
    case eufs_msgs::msg::CanState::AMI_TRACK_DRIVE:
      str2 = "AMI:TRACKDRIVE";
      break;
    case eufs_msgs::msg::CanState::AMI_ADS_INSPECTION:
      str2 = "AMI:ADS_INSPECTION";
      break;
    case eufs_msgs::msg::CanState::AMI_ADS_EBS:
      str2 = "AMI:ADS_EBS";
      break;
    case eufs_msgs::msg::CanState::AMI_DDT_INSPECTION_A:
      str2 = "AMI:DDT_INSPECTION_A";
      break;
    case eufs_msgs::msg::CanState::AMI_DDT_INSPECTION_B:
      str2 = "AMI:DDT_INSPECTION_B";
      break;
    case eufs_msgs::msg::CanState::AMI_AUTONOMOUS_DEMO:
      str2 = "AMI:BRAKETEST";
      break;
    case eufs_msgs::msg::CanState::AMI_JOYSTICK:
      str2 = "AMI:JOYSTICK";
      break;
    case eufs_msgs::msg::CanState::AMI_MANUAL:
      str2 = "AMI:MANUAL";
      break;
    default:
      str2 = "NO_SUCH_MESSAGE";
      break;
  }

  str3 = mission_completed_ ? "MISSION_COMPLETED:TRUE" : "MISSION_COMPLETED:FALSE";
  std_msgs::msg::String msg = std_msgs::msg::String();
  msg.data = str1 + " " + str2 + " " + str3;
  return msg;
}

void StateMachine::completedCallback(const std_msgs::msg::Bool::SharedPtr msg) {
  if (mission_completed_ != msg->data) {
    if (ami_state_ == eufs_msgs::msg::CanState::AMI_MANUAL) {
      RCLCPP_WARN(rosnode->get_logger(),
                  "state_machine :: mission completion is not defined for the manual mission");
      return;
    }

    RCLCPP_DEBUG(rosnode->get_logger(), "state_machine :: setting mission completed to %d",
                 msg->data);
    mission_completed_ = msg->data;
  }
}

void StateMachine::spinOnce(gazebo::common::Time current_time) {
  this->updateState(current_time);
  this->publishState();
  //    rclcpp::spin_some(this->rosnode);
}

bool StateMachine::canDrive() {
  return as_state_ == eufs_msgs::msg::CanState::AS_DRIVING ||
         ami_state_ == eufs_msgs::msg::CanState::AMI_MANUAL;
}

}  // namespace eufs_plugins
}  // namespace gazebo_plugins
