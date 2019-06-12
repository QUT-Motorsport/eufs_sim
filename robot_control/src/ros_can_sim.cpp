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


#include "ros_can_sim.h"

RosCanSim::RosCanSim() : nh_("~") {
    ROS_INFO("ros_can_sim :: Starting ");

    // Get robot model from the parameters
    if (!nh_.getParam("model", robot_model_)) {
        ROS_ERROR("Robot model not defined.");
        exit(-1);
    }

    // Ackermann configuration - traction - topics
    nh_.param<std::string>("frw_vel_topic", frw_vel_topic_, robot_model_ + "/right_front_axle_controller/command");
    nh_.param<std::string>("flw_vel_topic", flw_vel_topic_, robot_model_ + "/left_front_axle_controller/command");
    nh_.param<std::string>("blw_vel_topic", blw_vel_topic_, robot_model_ + "/left_rear_axle_controller/command");
    nh_.param<std::string>("brw_vel_topic", brw_vel_topic_, robot_model_ + "/right_rear_axle_controller/command");

    // Ackermann configuration - traction - joint names
    nh_.param<std::string>("joint_front_right_wheel", joint_front_right_wheel, "right_front_axle");
    nh_.param<std::string>("joint_front_left_wheel", joint_front_left_wheel, "left_front_axle");
    nh_.param<std::string>("joint_back_left_wheel", joint_back_left_wheel, "left_rear_axle");
    nh_.param<std::string>("joint_back_right_wheel", joint_back_right_wheel, "right_rear_axle");

    // Ackermann configuration - direction - topics
    nh_.param<std::string>("frw_pos_topic", frw_pos_topic_, robot_model_ + "/right_steering_joint_controller/command");
    nh_.param<std::string>("flw_pos_topic", flw_pos_topic_, robot_model_ + "/left_steering_joint_controller/command");

    nh_.param<std::string>("joint_front_right_steer", joint_front_right_steer, "right_steering_joint");
    nh_.param<std::string>("joint_front_left_steer", joint_front_left_steer, "left_steering_joint");

    // car parameters
    nh_.param<double>("wheelbase", wheelbase_, 1.53);
    nh_.param<double>("wheel_radius", wheel_radius_, 0.505/2);
    nh_.param<double>("max_speed", max_speed_, 20);
    nh_.param<double>("max_steering", max_steering_, 0.523599); // 27.2 degrees

    // steering link lengths for the ADS-DV are 0.02534 which are derived
    // from the vehicle specs by Ignat. Derived by inverse solving the
    // equations from page 30 of http://www.imgeorgiev.com/files/Ignat_MInf1_project.pdf
    nh_.param<double>("steering_link_length", steering_link_length_, 0.02534);

    nh_.param<double>("desired_freq", desired_freq_, 100.0);
    nh_.param<double>("joints_state_time_window", joints_state_time_window_, 1.0);

    // Robot Positions
    x_pos_ = 0.0;
    y_pos_ = 0.0;
    theta_ = 0.0;
    x_vel_ = 0.0;
    y_vel_ = 0.0;
    theta_vel_ = 0.0;

    // Times
    current_time = ros::Time::now();
    last_time = current_time;

    wheel_speed_sequence_ = 0;

    // Robot state space control references
    vel_ref_ = 0.0;
    steering_ref_ = 0.0;

    // init states
    as_state_ = as_state_type::AS_OFF;
    ami_state_ = ami_state_type::AMI_NOT_SELECTED;

    // Subscribers
    joint_state_sub_ = nh_.subscribe<sensor_msgs::JointState>("joint_states", 1, &RosCanSim::jointStateCallback, this);
    cmd_sub_ = nh_.subscribe<ackermann_msgs::AckermannDriveStamped>("/cmd_vel_out", 1, &RosCanSim::commandCallback,
                                                                    this);
    flag_sub_ = nh_.subscribe<std_msgs::Bool>("/ros_can/flag", 1, &RosCanSim::flagCallback, this);

    reset_srv_ = nh_.advertiseService("/ros_can/reset", &RosCanSim::resetState, this);
    ebs_srv_ = nh_.advertiseService("/ros_can/ebs", &RosCanSim::requestEBS, this);

    state_pub_ = nh_.advertise<eufs_msgs::canState>("/ros_can/state", 1);
    state_pub_str_ = nh_.advertise<std_msgs::String>("/ros_can/state_str", 1);
    wheel_speed_pub_ = nh_.advertise<eufs_msgs::wheelSpeeds>("ros_can/wheel_speeds", 10);


    // Advertise reference topics for the controllers
    ref_vel_frw_ = nh_.advertise<std_msgs::Float64>(frw_vel_topic_, 50);
    ref_vel_flw_ = nh_.advertise<std_msgs::Float64>(flw_vel_topic_, 50);
    ref_vel_blw_ = nh_.advertise<std_msgs::Float64>(blw_vel_topic_, 50);
    ref_vel_brw_ = nh_.advertise<std_msgs::Float64>(brw_vel_topic_, 50);
    ref_pos_frw_ = nh_.advertise<std_msgs::Float64>(frw_pos_topic_, 50);
    ref_pos_flw_ = nh_.advertise<std_msgs::Float64>(flw_pos_topic_, 50);

    // Flag to indicate joint_state has been read
    read_state_ = false;
}

RosCanSim::~RosCanSim() {

}

/// Controller startup in realtime
// TODO
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

/// Controller update loop
// TODO documentation
void RosCanSim::UpdateControl() {
    // Compute state control actions
    // State feedback error 4 position loops / 4 velocity loops
    // For more details refer to page 30 of http://www.imgeorgiev.com/files/Ignat_MInf1_project.pdf
    double R;
    double steering_ref_left, steering_ref_right;
    if (std::fabs(steering_ref_) > 1e06) {  // to avoid division by 0
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
}

bool RosCanSim::resetState(std_srvs::Trigger::Request& request, std_srvs::Trigger::Response& response) {
    (void)request;  // suppress unused parameter warning
    (void)response; // suppress unused parameter warning
    as_state_ = as_state_type::AS_OFF;
    ami_state_ = ami_state_type::AMI_NOT_SELECTED;
    response.success = true;
    return response.success;
}

bool RosCanSim::requestEBS(std_srvs::Trigger::Request& request, std_srvs::Trigger::Response& response) {
    (void)request;  // suppress unused parameter warning
    (void)response; // suppress unused parameter warning
    as_state_ = as_state_type::AS_EMERGENCY_BRAKE;
    ami_state_ = ami_state_type::AMI_NOT_SELECTED;
    response.success = true;
    return response.success;
}

void RosCanSim::publishWheelSpeeds() {

    if ((ros::Time::now() - joint_state_.header.stamp).toSec() > joints_state_time_window_) {
        ROS_WARN_THROTTLE(2, "ros_can_sim :: joint_states are not being received");
        return;
    }

    // get steering feedback
    // warning converting doubles to floats
    float right_steering_feedback = joint_state_.position[frw_pos_];
    float left_steering_feedback = joint_state_.position[flw_pos_];
    float steering_feedback = right_steering_feedback + left_steering_feedback / 2.0;

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

    eufs_msgs::wheelSpeeds msg;
    msg.header.stamp = joint_state_.header.stamp;
    msg.header.seq = wheel_speed_sequence_;
    msg.header.frame_id = "base_link";
    msg.lfSpeed = lf_wheel_rpm;
    msg.rfSpeed = rf_wheel_rpm;
    msg.lbSpeed = lb_wheel_rpm;
    msg.rbSpeed = rb_wheel_rpm;
    msg.steering = steering_feedback;

    wheel_speed_pub_.publish(msg);

    wheel_speed_sequence_++;
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

void RosCanSim::flagCallback(std_msgs::Bool msg) {
    driving_flag_ = msg.data;
}


// Topic command
void RosCanSim::jointStateCallback(const sensor_msgs::JointStateConstPtr &msg) {
    joint_state_ = *msg;
    read_state_ = true;
}

// Topic command
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

// RPM = (60 * Omega) / 2 PI
double RosCanSim::angularToRPM(double angular_vel) {
    return (60 * angular_vel) / (2 * PI);
}

bool RosCanSim::spin() {
    ROS_INFO("robot_control::spin()");
    ros::Rate r(desired_freq_);

    while (!ros::isShuttingDown()) // Using ros::isShuttingDown to avoid restarting the node during a shutdown.
    {
        if (starting() == 0) {
            while (ros::ok() && nh_.ok()) {
                this->UpdateControl();
                this->publishWheelSpeeds();
                this->updateState();
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
