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

#include <chrono>
#include <thread>
#include <iostream>

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float64.h>
#include <tf/transform_broadcaster.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <eufs_msgs/WheelSpeeds.h>
#include <eufs_msgs/WheelSpeedsStamped.h>
#include <eufs_msgs/CanState.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <std_srvs/Trigger.h>


/**
* @class RosCanSim
* @brief simulated ros_can for Gazebo
*
* @details Simulated IO functionality of the ros_can interfacing node
* https://gitlab.com/eufs/ros_can . This is intended to be used with eufs_sim
* and controls and interfaces with the ADS-DV car for the FSUK competition
* Further specs: https://www.imeche.org/docs/default-source/1-oscar/formula-student/2019/fs-ai/ads-dv-software-interface-specification-v0-2.pdf?sfvrsn=2
*/

class RosCanSim {
public:
    RosCanSim();  ///< Constructor
    ~RosCanSim();  ///< Destructor

    bool spin();  ///< Main operational loop

private:
    ros::NodeHandle nh_;
    double PI = 3.14159265; ///< Value for pi

    // definition of internal state machine
    typedef enum e_as_state_type {
        AS_OFF = 0,
        AS_READY = 1,
        AS_DRIVING = 2,
        AS_EMERGENCY_BRAKE = 3,
        AS_FINISHED = 4,
    } as_state_type;

    as_state_type as_state_; ///< state machine state

    // definition of mission selection
    typedef enum e_ami_state_type {
        AMI_NOT_SELECTED = 10,
        AMI_ACCELERATION = 11,
        AMI_SKIDPAD = 12,
        AMI_AUTOCROSS = 13,
        AMI_TRACK_DRIVE = 14,
        AMI_BRAKE_TEST = 15,
        AMI_ADS_INSPECTION = 16,
        AMI_ADS_EBS = 17,
        AMI_DDT_INSPECTION_A = 18,
        AMI_DDT_INSPECTION_B = 19,
        AMI_MANUAL = 20
    } ami_state_type;

    ami_state_type ami_state_;  ///< mission status

    bool driving_flag_; ///< mission flag as per ADS-DV specs

    double desired_freq_;

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

    // Joint states published by the joint_state_controller of the Controller Manager
    ros::Subscriber joint_state_sub_;
    ros::Subscriber set_mission_sub_;

    // High level robot command
    ros::Subscriber cmd_sub_;
    ros::Subscriber flag_sub_;

    // Velocity and position references to low level controllers
    ros::Publisher ref_vel_flw_;
    ros::Publisher ref_vel_frw_;
    ros::Publisher ref_vel_blw_;
    ros::Publisher ref_vel_brw_;
    ros::Publisher ref_pos_flw_;
    ros::Publisher ref_pos_frw_;

    ros::Publisher wheel_speed_pub_;
    ros::Publisher state_pub_;
    ros::Publisher state_pub_str_;

    ros::ServiceServer reset_srv_;  ///< service to reset state machine
    ros::ServiceServer ebs_srv_; ///< service to request an emergency brake

    // Indexes to joint_states
    int frw_vel_, flw_vel_, blw_vel_, brw_vel_, frw_pos_, flw_pos_;

    // Robot Joint States
    sensor_msgs::JointState joint_state_;
    bool read_state_; ///< Flag to indicate joint_state has been read
    double joints_state_time_window_; ///< accepted time deviation to process joint_state

    // Reference control commands
    double vel_ref_;
    double steering_ref_;

    // Car parameters
    double steering_link_length_; ///< lengths from the axle of the car to the wheel
    double wheel_radius_;
    double wheelbase_;  ///< distance from front to rear axle
    double max_speed_;  ///< maximum allowed control velocity
    double max_steering_;


    /**
      * Waits for the joint_state_publisher node and the simulation to launch by periodically checking
      * for the topics they publish.
      */
    int starting();

    /**
      * Stores the joint_states
      */
    void jointStateCallback(const sensor_msgs::JointStateConstPtr &msg);

    /**
      * Stores the commands to the car
      */
    void commandCallback(const ackermann_msgs::AckermannDriveStamped::ConstPtr &msg);

    /**
      * Checks if a value is within a certain range. Returns the saturated version of that value if outside bound
      * @param value to check
      * @param minimum allowed value
      * @param maximum allowed value
      */
    double saturation(double u, double min, double max);


    /**
      * Publishes a eufs_msgs/WheelSpeeds message based on joint states
      */
    void publishWheelSpeeds();

    /**
      * Converts an angular velocity to RPM
      * @param angular velocity to be converted
      */
    double angularToRPM(double angular_vel);

    /**
      * Stores the state of the driving flag
      * @param message of the driving flag
      */
    void flagCallback(std_msgs::Bool msg);

    /**
      * Resets the state of the internal state machine
      */
    bool resetState(std_srvs::Trigger::Request &request, std_srvs::Trigger::Response &response);

    /**
      * Puts the car into EMERGENCY_BRAKE state and stops it
      */
    bool requestEBS(std_srvs::Trigger::Request &request, std_srvs::Trigger::Response &response);

    /**
      * Sets the mission of the car. Only available in simulation
      */
    void setMission(eufs_msgs::CanState state);

    /**
      * Controls the joints(motors) of the simulated car
      */
    void UpdateControl();

    /**
      * Loops through the internal state machine of the car
      * Based on: https://www.imeche.org/docs/default-source/1-oscar/formula-student/2019/fs-ai/ads-dv-software-interface-specification-v0-2.pdf?sfvrsn=2
      */
    void updateState();

    /**
      * Publishes internal state and mission in a eufs_msgs/CanState.msg format
      */
    void publishState();

    /**
      * Creates a std_msgs/String.msg version of the internal state and mission
      */
    std_msgs::String makeStateString(const eufs_msgs::CanState &state);

};

#endif //ROBOT_CONTROL_ROS_CAN_SIM_H
