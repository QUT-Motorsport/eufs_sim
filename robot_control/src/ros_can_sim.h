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
 * @file ros_can_sim.h
 * @author EUFS
 * @date 09/06/2019
 * @copyright MIT License
 * @brief simulated ros_can for Gazebo
 *
 * @details Simulated IO functionality of the ros_can interfacing node
 * https://gitlab.com/eufs/ros_can . This is intended to be used with eufs_sim
 * and controls and interfaces with the ADS-DV car for the FSUK competition
 */

#ifndef ROBOT_CONTROL_ROS_CAN_SIM_H
#define ROBOT_CONTROL_ROS_CAN_SIM_H

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float64.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <ackermann_msgs/AckermannDriveStamped.h>

class RosCanSim {
public:
    RosCanSim();  ///< Constructor
    ~RosCanSim();  ///< Destoyer (forgot the proper word for it)

private:

    double PI = 3.14159265; ///< Value for pi

    ros::NodeHandle nh_;
    double desired_freq_;

    ros::Time last_time, current_time;

// Robot model
    std::string robot_model_;

// Velocity and position references to low level controllers
    ros::Publisher ref_vel_flw_;
    ros::Publisher ref_vel_frw_;
    ros::Publisher ref_vel_blw_;
    ros::Publisher ref_vel_brw_;
    ros::Publisher ref_pos_flw_;
    ros::Publisher ref_pos_frw_;

// Joint states published by the joint_state_controller of the Controller Manager
    ros::Subscriber joint_state_sub_;

// High level robot command
    ros::Subscriber cmd_sub_;

// Ackermann Topics - control action - traction - velocity
    std::string frw_vel_topic_;
    std::string flw_vel_topic_;
    std::string brw_vel_topic_;
    std::string blw_vel_topic_;

// Ackerman Topics - control action - steering - position
    std::string frw_pos_topic_;
    std::string flw_pos_topic_;

// Joint names - traction - velocity
    std::string joint_front_right_wheel;
    std::string joint_front_left_wheel;
    std::string joint_back_left_wheel;
    std::string joint_back_right_wheel;

// Joint names - steering - position
    std::string joint_front_right_steer;
    std::string joint_front_left_steer;

// Indexes to joint_states
    int frw_vel_, flw_vel_, blw_vel_, brw_vel_;
    int frw_pos_, flw_pos_;

// Robot Positions
    double x_pos_;
    double y_pos_;
    double theta_;
    double x_vel_;
    double y_vel_;
    double theta_vel_;

// Robot Joint States
    sensor_msgs::JointState joint_state_;

// Command reference
    ackermann_msgs::AckermannDriveStamped base_vel_msg_;

// External speed references
    double vel_ref_;
    double steering_ref_;
    double pos_ref_pan_;
    double pos_ref_tilt_;

// Flag to indicate if joint_state has been read
    bool read_state_; // Flag to indicate joint_state has been read

// Robot configuration parameters
    double wheel_diameter_;
    double wheelbase_;
    double max_speed_;
    double max_steering_;

// diagnostic frequencies
    double min_command_rec_freq_;
    double max_command_rec_freq_;

// accepted time deviation to process joint_sttate
    double joints_state_time_window_;

// Parameter that defines if odom tf is published or not
    bool publish_odom_tf_;

// Publisher for odom topic
    ros::Publisher odom_pub_;

// Broadcaster for odom tf
    tf::TransformBroadcaster odom_broadcaster;

    // TODO documentation
    int starting();

    void UpdateControl();

    void UpdateOdometry();

    void PublishOdometry();

    void stopping();

    void setCommand(const ackermann_msgs::AckermannDriveStamped &msg);

    void jointStateCallback(const sensor_msgs::JointStateConstPtr& msg);

    void commandCallback(const ackermann_msgs::AckermannDriveStamped::ConstPtr& msg);

    double saturation(double u, double min, double max);

    double radnorm( double value );

    double radnorm2( double value );

    bool spin();

};

#endif //ROBOT_CONTROL_ROS_CAN_SIM_H
