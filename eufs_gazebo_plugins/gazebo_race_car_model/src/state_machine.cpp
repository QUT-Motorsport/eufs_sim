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


#include "state_machine.hpp"
#include <string>
#include <vector>

StateMachine::StateMachine(boost::shared_ptr<ros::NodeHandle> &nh) : nh_(nh)
{
    // init state machine state
    as_state_ = eufs_msgs::CanState::AS_OFF;
    ami_state_ = eufs_msgs::CanState::AMI_NOT_SELECTED;
    driving_flag_ = false;

    // Subscribers
    flag_sub_ = nh_->subscribe<std_msgs::Bool>("/ros_can/mission_flag", 1, &StateMachine::flagCallback, this);
    set_mission_sub_ = nh_->subscribe<eufs_msgs::CanState>("/ros_can/set_mission", 1, &StateMachine::setMission, this);

    // Services
    reset_srv_ = nh_->advertiseService("/ros_can/reset", &StateMachine::resetState, this);
    ebs_srv_ = nh_->advertiseService("/ros_can/ebs", &StateMachine::requestEBS, this);

    // Publishers
    state_pub_ = nh_->advertise<eufs_msgs::CanState>("/ros_can/state", 1);
    state_pub_str_ = nh_->advertise<std_msgs::String>("/ros_can/state_str", 1);
}

StateMachine::~StateMachine()
{
}

void StateMachine::setMission(eufs_msgs::CanState state)
{
    if (state.as_state == eufs_msgs::CanState::AS_DRIVING)
    {
        as_state_ = eufs_msgs::CanState::AS_DRIVING;
        driving_flag_ = true;
    }

    if (ami_state_ == eufs_msgs::CanState::AMI_NOT_SELECTED)
    {
        switch (state.ami_state)
        {
            case eufs_msgs::CanState::AMI_ACCELERATION:
                ami_state_ = eufs_msgs::CanState::AMI_ACCELERATION;
                break;
            case eufs_msgs::CanState::AMI_SKIDPAD:
                ami_state_ = eufs_msgs::CanState::AMI_SKIDPAD;
                break;
            case eufs_msgs::CanState::AMI_AUTOCROSS:
                ami_state_ = eufs_msgs::CanState::AMI_AUTOCROSS;
                break;
            case eufs_msgs::CanState::AMI_TRACK_DRIVE:
                ami_state_ = eufs_msgs::CanState::AMI_TRACK_DRIVE;
                break;
            case eufs_msgs::CanState::AMI_AUTONOMOUS_DEMO:
                ami_state_ = eufs_msgs::CanState::AMI_AUTONOMOUS_DEMO;
                break;
            case eufs_msgs::CanState::AMI_ADS_INSPECTION:
                ami_state_ = eufs_msgs::CanState::AMI_ADS_INSPECTION;
                break;
            case eufs_msgs::CanState::AMI_ADS_EBS:
                ami_state_ = eufs_msgs::CanState::AMI_ADS_EBS;
                break;
            case eufs_msgs::CanState::AMI_DDT_INSPECTION_A:
                ami_state_ = eufs_msgs::CanState::AMI_DDT_INSPECTION_A;
                break;
            case eufs_msgs::CanState::AMI_DDT_INSPECTION_B:
                ami_state_ = eufs_msgs::CanState::AMI_DDT_INSPECTION_B;
                break;
            case eufs_msgs::CanState::AMI_MANUAL:
                ami_state_ = eufs_msgs::CanState::AMI_MANUAL;
                break;
            default:
                ami_state_ = eufs_msgs::CanState::AMI_NOT_SELECTED;
                break;
        }
    }
    else
    {
        ROS_WARN("Failed to set mission as a mission was set previously");
    }
}

bool StateMachine::resetState(std_srvs::Trigger::Request& request, std_srvs::Trigger::Response& response)
{
    (void)request;   // suppress unused parameter warning
    (void)response;  // suppress unused parameter warning
    as_state_ = eufs_msgs::CanState::AS_OFF;
    ami_state_ = eufs_msgs::CanState::AMI_NOT_SELECTED;
    driving_flag_ = false;
    response.success = true;
    return response.success;
}

bool StateMachine::requestEBS(std_srvs::Trigger::Request& request, std_srvs::Trigger::Response& response)
{
    (void)request;   // suppress unused parameter warning
    (void)response;  // suppress unused parameter warning
    as_state_ = eufs_msgs::CanState::AS_EMERGENCY_BRAKE;
    ami_state_ = eufs_msgs::CanState::AMI_NOT_SELECTED;
    driving_flag_ = false;
    response.success = true;
    return response.success;
}

void StateMachine::updateState()
{
    using namespace std::chrono_literals;

    switch (as_state_)
    {
        case eufs_msgs::CanState::AS_OFF:
            if ((ami_state_ != eufs_msgs::CanState::AMI_NOT_SELECTED) && driving_flag_)
            {
                // first sleep for 5s as is with the real car
                std::this_thread::sleep_for(5s);

                // now transition to new state
                as_state_ = eufs_msgs::CanState::AS_READY;
                ROS_DEBUG("state_machine :: switching to AS_READY state");
            }
            break;

        case eufs_msgs::CanState::AS_READY:
            if (driving_flag_)
            {
                as_state_ = eufs_msgs::CanState::AS_DRIVING;
                ROS_DEBUG("state_machine :: switching to AS_DRIVING state");
            }
            break;

        case eufs_msgs::CanState::AS_DRIVING:
            if (!driving_flag_)
            {
                as_state_ = eufs_msgs::CanState::AS_FINISHED;
                ROS_DEBUG("state_machine :: switching to AS_FINISHED state");
            }
            break;

        case eufs_msgs::CanState::AS_FINISHED:
            // do nothing for now
            break;

        case eufs_msgs::CanState::AS_EMERGENCY_BRAKE:
            // do nothing for now
            break;

            // default: do nothing
    }
}

void StateMachine::publishState()
{
    if (state_pub_.getNumSubscribers() == 0 &&
            state_pub_str_.getNumSubscribers() == 0)
        return;  // do nothing

    // create message
    eufs_msgs::CanState state_msg;
    state_msg.as_state = as_state_;
    state_msg.ami_state = ami_state_;
    state_msg.mission_flag = driving_flag_;

    if (state_pub_.getNumSubscribers() > 0)
        state_pub_.publish(state_msg);

    if (state_pub_str_.getNumSubscribers() > 0)
        state_pub_str_.publish(makeStateString(state_msg));
}

std_msgs::String StateMachine::makeStateString(const eufs_msgs::CanState &state)
{
    std::string str1;
    std::string str2;
    std::string str3;

    ROS_DEBUG("AS STATE: %d", state.as_state);
    ROS_DEBUG("AMI STATE: %d", state.ami_state);

    switch (state.as_state)
    {
        case eufs_msgs::CanState::AS_OFF:
            str1 = "AS:OFF";
            break;
        case eufs_msgs::CanState::AS_READY:
            str1 = "AS:READY";
            break;
        case eufs_msgs::CanState::AS_DRIVING:
            str1 = "AS:DRIVING";
            break;
        case eufs_msgs::CanState::AS_FINISHED:
            str1 = "AS:FINISHED";
            break;
        case eufs_msgs::CanState::AS_EMERGENCY_BRAKE:
            str1 = "AS:EMERGENCY";
            break;
        default:
            str1 = "NO_SUCH_MESSAGE";
    }

    switch (state.ami_state)
    {
        case eufs_msgs::CanState::AMI_NOT_SELECTED:
            str2 = "AMI:NOT_SELECTED";
            break;
        case eufs_msgs::CanState::AMI_ACCELERATION:
            str2 = "AMI:ACCELERATION";
            break;
        case eufs_msgs::CanState::AMI_SKIDPAD:
            str2 = "AMI:SKIDPAD";
            break;
        case eufs_msgs::CanState::AMI_AUTOCROSS:
            str2 = "AMI:AUTOCROSS";
            break;
        case eufs_msgs::CanState::AMI_TRACK_DRIVE:
            str2 = "AMI:TRACKDRIVE";
            break;
        case eufs_msgs::CanState::AMI_ADS_INSPECTION:
            str2 = "AS:ADS_INSPECTION";
            break;
        case eufs_msgs::CanState::AMI_ADS_EBS:
            str2 = "AS:ADS_EBS";
            break;
        case eufs_msgs::CanState::AMI_DDT_INSPECTION_A:
            str2 = "AS:DDT_INSPECTION_A";
            break;
        case eufs_msgs::CanState::AMI_DDT_INSPECTION_B:
            str2 = "AS:DDT_INSPECTION_B";
            break;
        case eufs_msgs::CanState::AMI_AUTONOMOUS_DEMO:
            str2 = "AS:BRAKETEST";
            break;
        default:
            str2 = "NO_SUCH_MESSAGE";
    }

    if (driving_flag_)
        str3 = "DRIVING:TRUE";
    else
        str3 = "DRIVING:FALSE";

    std_msgs::String msg = std_msgs::String();
    msg.data = str1 + " " + str2 + " " + str3;
    return msg;
}

void StateMachine::flagCallback(std_msgs::Bool msg)
{
    ROS_DEBUG("state_machine :: setting driving flag to %d", msg.data);
    driving_flag_ = msg.data;
}

bool StateMachine::spinOnce()
{
    this->updateState();
    this->publishState();
    ros::spinOnce();
}

bool StateMachine::canDrive() {
  return as_state_ == eufs_msgs::CanState::AS_DRIVING;
}
