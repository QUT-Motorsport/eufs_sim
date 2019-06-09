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
    nh_.param<std::string>("joint_front_left_wheel",  joint_front_left_wheel,  "left_front_axle");
    nh_.param<std::string>("joint_back_left_wheel",   joint_back_left_wheel,   "left_rear_axle");
    nh_.param<std::string>("joint_back_right_wheel",  joint_back_right_wheel,  "right_rear_axle");

    // Ackermann configuration - direction - topics
    nh_.param<std::string>("frw_pos_topic", frw_pos_topic_, robot_model_ + "/right_steering_joint_controller/command");
    nh_.param<std::string>("flw_pos_topic", flw_pos_topic_, robot_model_ + "/left_steering_joint_controller/command");

    nh_.param<std::string>("joint_front_right_steer", joint_front_right_steer, "right_steering_joint");
    nh_.param<std::string>("joint_front_left_steer",  joint_front_left_steer,  "left_steering_joint");

    // car parameters
    nh_.param<double>("wheelbase", wheelbase_, 1.53);
    nh_.param<double>("wheel_diameter", wheel_diameter_, 0.505);
    nh_.param<double>("max_speed", max_speed_, 20);
    nh_.param<double>("max_steering", max_steering_, 0.523599); // 27.2 degrees

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

    // Robot state space control references
    vel_ref_ = 0.0;
    steering_ref_ = 0.0;

    // Subscribers
    joint_state_sub_ = nh_.subscribe<sensor_msgs::JointState>("joint_states", 1, &RosCanSim::jointStateCallback, this);
    cmd_sub_ = nh_.subscribe<ackermann_msgs::AckermannDriveStamped>("command", 1, &RosCanSim::commandCallback, this);

    // Advertise reference topics for the controllers
    ref_vel_frw_ = nh_.advertise<std_msgs::Float64>( frw_vel_topic_, 50);
    ref_vel_flw_ = nh_.advertise<std_msgs::Float64>( flw_vel_topic_, 50);
    ref_vel_blw_ = nh_.advertise<std_msgs::Float64>( blw_vel_topic_, 50);
    ref_vel_brw_ = nh_.advertise<std_msgs::Float64>( brw_vel_topic_, 50);
    ref_pos_frw_ = nh_.advertise<std_msgs::Float64>( frw_pos_topic_, 50);
    ref_pos_flw_ = nh_.advertise<std_msgs::Float64>( flw_pos_topic_, 50);

    // Flag to indicate joint_state has been read
    read_state_ = false;
}

    /// Controller startup in realtime
    // TODO
    int RosCanSim::starting() {
        // Initialize joint indexes according to joint names
        if (read_state_) {
            std::vector<std::string> joint_names = joint_state_.name;
            frw_vel_ = find (joint_names.begin(),joint_names.end(), std::string(joint_front_right_wheel)) - joint_names.begin();
            flw_vel_ = find (joint_names.begin(),joint_names.end(), std::string(joint_front_left_wheel)) - joint_names.begin();
            blw_vel_ = find (joint_names.begin(),joint_names.end(), std::string(joint_back_left_wheel)) - joint_names.begin();
            brw_vel_ = find (joint_names.begin(),joint_names.end(), std::string(joint_back_right_wheel)) - joint_names.begin();
            frw_pos_ = find (joint_names.begin(),joint_names.end(), std::string(joint_front_right_steer)) - joint_names.begin();
            flw_pos_ = find (joint_names.begin(),joint_names.end(), std::string(joint_front_left_steer)) - joint_names.begin();
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
        // Single steering
        double d1;
        double alfa_ref_left = 0.0;
        double alfa_ref_right = 0.0;
        if (steering_ref_!=0.0) {  // div/0
            d1 =  wheelbase_ / tan (steering_ref_);
            alfa_ref_left = atan2( wheelbase_, d1 - 0.105);
            alfa_ref_right = atan2( wheelbase_, d1 + 0.105);
            if (steering_ref_<0.0) {
                alfa_ref_left = alfa_ref_left - PI;
                alfa_ref_right = alfa_ref_right - PI;
            }
        } else {
            alfa_ref_left = 0.0;
            alfa_ref_right = 0.0;
        }

        // Angular position ref publish
        std_msgs::Float64 frw_ref_pos_msg;
        std_msgs::Float64 flw_ref_pos_msg;
        std_msgs::Float64 brw_ref_pos_msg;
        std_msgs::Float64 blw_ref_pos_msg;

        flw_ref_pos_msg.data = alfa_ref_left;
        frw_ref_pos_msg.data = alfa_ref_right;

        // Linear speed ref publish (could be improved by setting correct speed to each wheel according to turning state
        // w = v_mps / (PI * D);   w_rad = w * 2.0 * PI
        double ref_speed_joint = 2.0 * vel_ref_ / wheel_diameter_;

        std_msgs::Float64 frw_ref_vel_msg;
        std_msgs::Float64 flw_ref_vel_msg;
        std_msgs::Float64 brw_ref_vel_msg;
        std_msgs::Float64 blw_ref_vel_msg;
        frw_ref_vel_msg.data = -ref_speed_joint;
        flw_ref_vel_msg.data = -ref_speed_joint;
        brw_ref_vel_msg.data = -ref_speed_joint;
        blw_ref_vel_msg.data = -ref_speed_joint;

        // Publish msgs traction and direction
        ref_vel_frw_.publish( frw_ref_vel_msg );
        ref_vel_flw_.publish( flw_ref_vel_msg );
        ref_vel_blw_.publish( blw_ref_vel_msg );
        ref_vel_brw_.publish( brw_ref_vel_msg );
        ref_pos_frw_.publish( frw_ref_pos_msg );
        ref_pos_flw_.publish( flw_ref_pos_msg );
    }

    // Update robot odometry depending on kinematic configuration
    void RosCanSim::UpdateOdometry() {
        // Get angles
        double a1, a2;

        if( (ros::Time::now() - joint_state_.header.stamp).toSec() > joints_state_time_window_){
            ROS_WARN_THROTTLE(2, "RosCanSim::UpdateOdometry: joint_states are not being received");
            return;
        }

        a1 = joint_state_.position[frw_pos_];
        a2 = joint_state_.position[flw_pos_];

        // Linear speed of each wheel [mps]
        double v3, v4;
        // filtering noise from the Velocity controller when the speed is 0.0 (by using an open loop with desired speed)
        if( vel_ref_ == 0.0) {
            v3 = 0.0;
            v4 = 0.0;
        } else {
            v3 = joint_state_.velocity[blw_vel_] * (wheel_diameter_ / 2.0);
            v4 = joint_state_.velocity[brw_vel_] * (wheel_diameter_ / 2.0);
        }
        // Turning angle front
        double fBetaRads = (a1 + a2) / 2.0;

        // Linear speed
        double fSamplePeriod = 1.0 / desired_freq_;  // Default sample period
        double v_mps = -(v3 + v4) / 2.0;

        current_time = ros::Time::now();

        x_vel_ = v_mps * cos(theta_);
        y_vel_ = v_mps * sin(theta_);
        theta_vel_ = (tan(fBetaRads) * v_mps) / wheelbase_;

        double dt = current_time.toSec() - last_time.toSec();
        double delta_x = x_vel_ * dt;
        double delta_y = y_vel_ * dt;
        double delta_th = theta_vel_ * dt;

        x_pos_ += delta_x;
        y_pos_ += delta_y;
        theta_ += delta_th;

        last_time = current_time;
    }

    // Publish robot odometry tf and topic depending
    void RosCanSim::PublishOdometry() {
        //first, we'll publish the transform over tf
        // TODO change to tf_prefix
        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = current_time;
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = "base_footprint";

        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(theta_);

        odom_trans.transform.translation.x = x_pos_;
        odom_trans.transform.translation.y = y_pos_;
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation = odom_quat;

        // send the transform over /tf
        // activate / deactivate with param
        // this tf in needed when not using robot_pose_ekf
        if (publish_odom_tf_) odom_broadcaster.sendTransform(odom_trans);

        //next, we'll publish the odometry message over ROS
        nav_msgs::Odometry odom;
        odom.header.stamp = current_time;
        odom.header.frame_id = "odom";

        //set the position
        // Position
        odom.pose.pose.position.x = x_pos_;
        odom.pose.pose.position.y = y_pos_;
        odom.pose.pose.position.z = 0.0;
        // Orientation
        odom.pose.pose.orientation = odom_quat;
        // Pose covariance
        for(int i = 0; i < 6; i++)
            odom.pose.covariance[i*6+i] = 0.1;  // test 0.001

        //set the velocity
        odom.child_frame_id = "base_footprint";
        // Linear velocities
        odom.twist.twist.linear.x = x_vel_;
        odom.twist.twist.linear.y = y_vel_;
        odom.twist.twist.linear.z = 0.0;
        // Angular velocities
        odom.twist.twist.angular.z = theta_vel_;
        // Twist covariance
        for(int i = 0; i < 6; i++)
            odom.twist.covariance[6*i+i] = 0.1;  // test 0.001

        //publish the message
        odom_pub_.publish(odom);
    }

    /// Controller stopping
    void RosCanSim::stopping()
    {}


    // Set the base velocity command
    void RosCanSim::setCommand(const ackermann_msgs::AckermannDriveStamped &msg) {
        v_ref_ = saturation(msg.drive.speed, -max_speed_, max_speed_);
        alfa_ref_ = saturation(msg.drive.steering_angle, -max_steering_, max_steering_);
    }

    // Topic command
    void RosCanSim::jointStateCallback(const sensor_msgs::JointStateConstPtr& msg) {
        joint_state_ = *msg;
        read_state_ = true;
    }

    // Topic command
    void RosCanSim::commandCallback(const ackermann_msgs::AckermannDriveStamped::ConstPtr& msg) {
        base_vel_msg_ = *msg;
        this->setCommand(base_vel_msg_);
    }

    double RosCanSim::saturation(double u, double min, double max) {
        if (u>max) u=max;
        if (u<min) u=min;
        return u;
    }

    double RosCanSim::radnorm( double value ) {
        while (value > PI) value -= PI;
        while (value < -PI) value += PI;
        return value;
    }

    double RosCanSim::radnorm2( double value ) {
        while (value > 2.0*PI) value -= 2.0*PI;
        while (value < -2.0*PI) value += 2.0*PI;
        return value;
    }

    bool RosCanSim::spin()
    {
        ROS_INFO("robot_control::spin()");
        ros::Rate r(desired_freq_);

        while (!ros::isShuttingDown()) // Using ros::isShuttingDown to avoid restarting the node during a shutdown.
        {
            if (starting() == 0)
            {
                while(ros::ok() && nh_.ok()) {
                    UpdateControl();
                    UpdateOdometry();
                    PublishOdometry();
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

}; // Class RosCanSim

int main(int argc, char** argv) {
    ros::init(argc, argv, "robot_control");

    ros::NodeHandle n;
    RosCanSim scc(n);
    scc.spin();

    return (0);
}
