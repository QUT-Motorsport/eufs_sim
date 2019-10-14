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
 * @file ros_can_sim.cpp
 * @author EUFS
 * @date 09/06/2019
 * @copyright MIT License
 * @brief simulated ros_can for Gazebo
 *
 * @details Simulated IO functionality of the ros_can interfacing node
 * https://gitlab.com/eufs/ros_can . This is intended to be used with eufs_sim
 * and controls and interfaces with the ADS-DV car for the FSUK competition
 */


#include "ros_can_sim.hpp"

RosCanSim::RosCanSim() : nh_("~") {
    ROS_INFO("ros_can_sim :: Starting ");

    // Ackermann configuration - traction - topics
    nh_.param<std::string>("frw_vel_topic", frw_vel_topic_, "/eufs/right_front_axle_controller/command");
    nh_.param<std::string>("flw_vel_topic", flw_vel_topic_, "/eufs/left_front_axle_controller/command");
    nh_.param<std::string>("blw_vel_topic", blw_vel_topic_, "/eufs/left_rear_axle_controller/command");
    nh_.param<std::string>("brw_vel_topic", brw_vel_topic_, "/eufs/right_rear_axle_controller/command");
    nh_.param<std::string>("frw_pos_topic", frw_pos_topic_, "/eufs/right_steering_joint_controller/command");
    nh_.param<std::string>("flw_pos_topic", flw_pos_topic_, "/eufs/left_steering_joint_controller/command");

    // Ackermann configuration - traction - joint names
    nh_.param<std::string>("joint_front_right_wheel", joint_front_right_wheel, "right_front_axle");
    nh_.param<std::string>("joint_front_left_wheel",  joint_front_left_wheel, "left_front_axle");
    nh_.param<std::string>("joint_back_left_wheel",   joint_back_left_wheel, "left_rear_axle");
    nh_.param<std::string>("joint_back_right_wheel",  joint_back_right_wheel, "right_rear_axle");
    nh_.param<std::string>("joint_front_right_steer", joint_front_right_steer, "right_steering_joint");
    nh_.param<std::string>("joint_front_left_steer",  joint_front_left_steer, "left_steering_joint");

    // car parameters
    nh_.param<double>("wheelbase", wheelbase_, 1.53);
    nh_.param<double>("wheel_radius", wheel_radius_, 0.505/2);
    nh_.param<double>("max_speed", max_speed_, 20.0);
    nh_.param<double>("max_steering", max_steering_, 0.523599); // 27.2 degrees

    // steering link lengths for the ADS-DV are 0.02534 which are derived
    // from the vehicle specs by Ignat. Derived by inverse solving the
    // equations from page 30 of http://www.imgeorgiev.com/files/Ignat_MInf1_project.pdf
    nh_.param<double>("steering_link_length", steering_link_length_, 0.02534);

    nh_.param<double>("desired_freq", desired_freq_, 100.0);
    nh_.param<double>("joints_state_time_window", joints_state_time_window_, 1.0);

    // Control signals
    vel_ref_ = 0.0;
    steering_ref_ = 0.0;

    // init state machine state
    as_state_ = as_state_type::AS_OFF;
    ami_state_ = ami_state_type::AMI_NOT_SELECTED;
    driving_flag_ = false;

    // Flag to indicate joint_state has been read
    read_state_ = false;

    // Subscribers
    joint_state_sub_ = nh_.subscribe<sensor_msgs::JointState>("/eufs/joint_states", 1, &RosCanSim::jointStateCallback, this);
    cmd_sub_ = nh_.subscribe<ackermann_msgs::AckermannDriveStamped>("/cmd_vel_out", 1, &RosCanSim::commandCallback,
                                                                    this);
    flag_sub_ = nh_.subscribe<std_msgs::Bool>("/ros_can/mission_flag", 1, &RosCanSim::flagCallback, this);
    set_mission_sub_ = nh_.subscribe<eufs_msgs::CanState>("/ros_can/set_mission", 1, &RosCanSim::setMission, this);

    // Services
    reset_srv_ = nh_.advertiseService("/ros_can/reset", &RosCanSim::resetState, this);
    ebs_srv_ = nh_.advertiseService("/ros_can/ebs", &RosCanSim::requestEBS, this);

    // Publishers
    state_pub_ = nh_.advertise<eufs_msgs::CanState>("/ros_can/state", 1);
    state_pub_str_ = nh_.advertise<std_msgs::String>("/ros_can/state_str", 1);
    wheel_speed_pub_ = nh_.advertise<eufs_msgs::WheelSpeeds>("/ros_can/wheel_speeds", 10);

    // Advertise reference topics for the controllers
    ref_vel_frw_ = nh_.advertise<std_msgs::Float64>(frw_vel_topic_, 50);
    ref_vel_flw_ = nh_.advertise<std_msgs::Float64>(flw_vel_topic_, 50);
    ref_vel_blw_ = nh_.advertise<std_msgs::Float64>(blw_vel_topic_, 50);
    ref_vel_brw_ = nh_.advertise<std_msgs::Float64>(brw_vel_topic_, 50);
    ref_pos_frw_ = nh_.advertise<std_msgs::Float64>(frw_pos_topic_, 50);
    ref_pos_flw_ = nh_.advertise<std_msgs::Float64>(flw_pos_topic_, 50);
}

RosCanSim::~RosCanSim() {

}

int RosCanSim::starting() {
    // Initialize joint indexes according to joint names
    if (read_state_) {
        std::vector<std::string> joint_names = joint_state_.name;
        frw_vel_ = find(joint_names.begin(), joint_names.end(), std::string(joint_front_right_wheel)) -
                   joint_names.begin();
        flw_vel_ =
                find(joint_names.begin(), joint_names.end(), std::string(joint_front_left_wheel)) - joint_names.begin();
        blw_vel_ =
                find(joint_names.begin(), joint_names.end(), std::string(joint_back_left_wheel)) - joint_names.begin();
        brw_vel_ =
                find(joint_names.begin(), joint_names.end(), std::string(joint_back_right_wheel)) - joint_names.begin();
        frw_pos_ = find(joint_names.begin(), joint_names.end(), std::string(joint_front_right_steer)) -
                   joint_names.begin();
        flw_pos_ =
                find(joint_names.begin(), joint_names.end(), std::string(joint_front_left_steer)) - joint_names.begin();
        return 0;
    } else {
        ROS_WARN("RosCanSim::starting: joint_states are not being received");
        return -1;
    }
}

void RosCanSim::UpdateControl() {
    // Compute state control actions
    // State feedback error 4 position loops / 4 velocity loops
    // For more details refer to page 30 of http://www.imgeorgiev.com/files/Ignat_MInf1_project.pdf
    double R;
    double steering_ref_left, steering_ref_right;
    if (std::fabs(steering_ref_) > 1e-06) {  // to avoid division by 0
        R = wheelbase_ / tan(steering_ref_);
        steering_ref_left = atan2(wheelbase_, R - steering_link_length_);
        steering_ref_right = atan2(wheelbase_, R + steering_link_length_);
        if (steering_ref_ < 0.0) {
            steering_ref_left = steering_ref_left - PI;
            steering_ref_right = steering_ref_right - PI;
        }
    } else {
        steering_ref_left = 0.0;
        steering_ref_right = 0.0;
    }

    // Angular position ref publish
    std_msgs::Float64 frw_ref_pos_msg;
    std_msgs::Float64 flw_ref_pos_msg;
    std_msgs::Float64 brw_ref_pos_msg;
    std_msgs::Float64 blw_ref_pos_msg;

    flw_ref_pos_msg.data = steering_ref_left;
    frw_ref_pos_msg.data = steering_ref_right;

    // Linear speed ref publish
    double ref_speed_joint = vel_ref_ / wheel_radius_;

    std_msgs::Float64 frw_ref_vel_msg;
    std_msgs::Float64 flw_ref_vel_msg;
    std_msgs::Float64 brw_ref_vel_msg;
    std_msgs::Float64 blw_ref_vel_msg;
    frw_ref_vel_msg.data = -ref_speed_joint;
    flw_ref_vel_msg.data = -ref_speed_joint;
    brw_ref_vel_msg.data = -ref_speed_joint;
    blw_ref_vel_msg.data = -ref_speed_joint;

    // Publish msgs traction and direction
    ref_vel_frw_.publish(frw_ref_vel_msg);
    ref_vel_flw_.publish(flw_ref_vel_msg);
    ref_vel_blw_.publish(blw_ref_vel_msg);
    ref_vel_brw_.publish(brw_ref_vel_msg);
    ref_pos_frw_.publish(frw_ref_pos_msg);
    ref_pos_flw_.publish(flw_ref_pos_msg);

    ROS_DEBUG("Published steering angles %f rad  %f rad", steering_ref_left, steering_ref_right);
    ROS_DEBUG("Published wheel speeds %f rad/s", -ref_speed_joint);
}

void RosCanSim::setMission(eufs_msgs::CanState state) {
    if (state.as_state == eufs_msgs::CanState::AS_DRIVING) {
        as_state_ = as_state_type::AS_DRIVING;
        driving_flag_ = true;
    }

    if (ami_state_ == ami_state_type::AMI_NOT_SELECTED) {
        switch (state.ami_state) {
            case eufs_msgs::CanState::AMI_ACCELERATION:
                ami_state_ = ami_state_type::AMI_ACCELERATION;
                break;
            case eufs_msgs::CanState::AMI_SKIDPAD:
                ami_state_ = ami_state_type::AMI_SKIDPAD;
                break;
            case eufs_msgs::CanState::AMI_AUTOCROSS:
                ami_state_ = ami_state_type::AMI_AUTOCROSS;
                break;
            case eufs_msgs::CanState::AMI_TRACK_DRIVE:
                ami_state_ = ami_state_type::AMI_TRACK_DRIVE;
                break;
            case eufs_msgs::CanState::AMI_BRAKE_TEST:
                ami_state_ = ami_state_type::AMI_BRAKE_TEST;
                break;
            case eufs_msgs::CanState::AMI_INSPECTION:
                ami_state_ = ami_state_type::AMI_INSPECTION;
                break;
            case eufs_msgs::CanState::AMI_MANUAL:
                ami_state_ = ami_state_type::AMI_MANUAL;
                break;
            default:
                ami_state_ = ami_state_type::AMI_NOT_SELECTED;
                break;
        }
    } else {
        ROS_WARN("Failed to set mission as a mission was set previously");
    }
}

bool RosCanSim::resetState(std_srvs::Trigger::Request& request, std_srvs::Trigger::Response& response) {
    (void)request;  // suppress unused parameter warning
    (void)response; // suppress unused parameter warning
    as_state_ = as_state_type::AS_OFF;
    ami_state_ = ami_state_type::AMI_NOT_SELECTED;
    driving_flag_ = false;
    response.success = true;
    return response.success;
}

bool RosCanSim::requestEBS(std_srvs::Trigger::Request& request, std_srvs::Trigger::Response& response) {
    (void)request;  // suppress unused parameter warning
    (void)response; // suppress unused parameter warning
    as_state_ = as_state_type::AS_EMERGENCY_BRAKE;
    ami_state_ = ami_state_type::AMI_NOT_SELECTED;
    driving_flag_ = false;
    response.success = true;
    return response.success;
}

void RosCanSim::publishWheelSpeeds() {

    if ((ros::Time::now() - joint_state_.header.stamp).toSec() > joints_state_time_window_) {
        ROS_WARN_THROTTLE(2, "ros_can_sim :: joint_states are not being received");
        return;
    }

    // get steering feedback
    // it's inverted in order to mimick the outputs of the real ros_can
    float right_steering_feedback = joint_state_.position[frw_pos_];
    float left_steering_feedback = joint_state_.position[flw_pos_];
    float steering_feedback = (right_steering_feedback + left_steering_feedback) / 2.0;

    if (fabs(steering_feedback) > max_steering_) {
        ROS_DEBUG("ros_can_sim :: steering feedback exceeded limit");
        ROS_DEBUG("ros_can_sim :: steering feedback = %f", steering_feedback);
        ROS_DEBUG("ros_can_sim :: max steering = %f", max_steering_);
    }

    // get velocity feedback
    auto lf_wheel_rpm = angularToRPM(-joint_state_.velocity[flw_vel_]);
    auto rf_wheel_rpm = angularToRPM(-joint_state_.velocity[frw_vel_]);
    auto lb_wheel_rpm = angularToRPM(-joint_state_.velocity[blw_vel_]);
    auto rb_wheel_rpm = angularToRPM(-joint_state_.velocity[brw_vel_]);

    eufs_msgs::WheelSpeeds msg;
    msg.header.stamp = joint_state_.header.stamp;
    msg.header.frame_id = "base_link";
    msg.lf_speed = lf_wheel_rpm;
    msg.rf_speed = rf_wheel_rpm;
    msg.lb_speed = lb_wheel_rpm;
    msg.rb_speed = rb_wheel_rpm;
    msg.steering = steering_feedback;

    wheel_speed_pub_.publish(msg);

}

void RosCanSim::updateState() {
    using namespace std::literals::chrono_literals;

    switch (as_state_) {
        case as_state_type::AS_OFF:
            if ((ami_state_ != ami_state_type::AMI_NOT_SELECTED) && driving_flag_) {
                // first sleep for 5s as is with the real car
                std::this_thread::sleep_for(5s);

                // now transition to new state
                as_state_ = as_state_type::AS_READY;
                ROS_DEBUG("ros_can_sim :: switching to AS_READY state");
            }
            break;

        case as_state_type::AS_READY:
            if (driving_flag_) {
                as_state_ = as_state_type::AS_DRIVING;
                ROS_DEBUG("ros_can_sim :: switching to AS_DRIVING state");
            }
            break;

        case as_state_type::AS_DRIVING:
            if (!driving_flag_) {
                as_state_ = as_state_type::AS_FINISHED;
                ROS_DEBUG("ros_can_sim :: switching to AS_FINISHED state");
            }
            break;

        case as_state_type::AS_FINISHED:
            // do nothing for now
            break;

        case as_state_type::AS_EMERGENCY_BRAKE:
            // do nothing for now
            break;

            // default: do nothing
    }
}

void RosCanSim::publishState() {
    if (state_pub_.getNumSubscribers() == 0 &&
            state_pub_str_.getNumSubscribers() == 0)
        return; // do nothing

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

std_msgs::String RosCanSim::makeStateString(const eufs_msgs::CanState &state) {
    std::string str1;
    std::string str2;
    std::string str3;

    ROS_DEBUG("AS STATE: %d", state.as_state);
    ROS_DEBUG("AMI STATE: %d", state.ami_state);

    switch (state.as_state) {
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

    switch (state.ami_state) {
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
        case eufs_msgs::CanState::AMI_INSPECTION:
            str2 = "AS:INSPECTION";
            break;
        case eufs_msgs::CanState::AMI_BRAKE_TEST:
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

void RosCanSim::flagCallback(std_msgs::Bool msg) {
    ROS_DEBUG("ros_can_sim :: setting driving flag to %d", msg.data);
    driving_flag_ = msg.data;
}

void RosCanSim::jointStateCallback(const sensor_msgs::JointStateConstPtr &msg) {
    ROS_DEBUG("ros_can_sim ::Joint states have been received");
    joint_state_ = *msg;
    read_state_ = true;
}

void RosCanSim::commandCallback(const ackermann_msgs::AckermannDriveStamped::ConstPtr &msg) {
    // set commands only if in AS_DRIVING state
    if (as_state_ == as_state_type::AS_DRIVING) {
        vel_ref_ = saturation(msg->drive.speed, -max_speed_, max_speed_);
        steering_ref_ = saturation(msg->drive.steering_angle, -max_steering_, max_steering_);
    } else {
        vel_ref_ = 0.0;
        steering_ref_ = 0.0;
    }
}

double RosCanSim::saturation(double u, double min, double max) {
    if (u > max) u = max;
    if (u < min) u = min;
    return u;
}

double RosCanSim::angularToRPM(double angular_vel) {
    // RPM = (60 * Omega) / 2 PI
    return (60 * angular_vel) / (2 * PI);
}

bool RosCanSim::spin() {
    ROS_INFO("ros_can_sim::spin()");
    ros::Rate r(desired_freq_);

    while (!ros::isShuttingDown()) // Using ros::isShuttingDown to avoid restarting the node during a shutdown.
    {
        if (starting() == 0) {
            while (ros::ok() && nh_.ok()) {
                this->UpdateControl();
                this->publishWheelSpeeds();
                this->updateState();
                this->publishState();
                ros::spinOnce();
                r.sleep();
            }
            ROS_INFO("END OF ros::ok() !!!");
        } else {
            // No need for diagnostic here since a broadcast occurs in start
            // when there is an error.
            usleep(1000000);
            ros::spinOnce();
        }
    }
    return true;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "ros_can_sim");

    RosCanSim node = RosCanSim();
    node.spin();

    return (0);
}
