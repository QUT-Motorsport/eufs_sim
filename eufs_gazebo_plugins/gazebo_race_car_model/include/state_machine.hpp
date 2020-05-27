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
 * @file state_machine.hpp
 * @author EUFS
 * @date 09/06/2019
 * @copyright MIT License
 * @brief simulated ros_can for Gazebo
 *
 * @details Simulated IO functionality of the ros_can interfacing node
 * https://gitlab.com/eufs/ros_can . This is intended to be used with eufs_sim
 * and controls and interfaces with the ADS-DV car for the FSUK competition
 */

#ifndef ROBOT_CONTROL_STATE_MACHINE_H
#define ROBOT_CONTROL_STATE_MACHINE_H

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
* @class StateMachine
* @brief simulated ros_can for Gazebo
*
* @details Simulated IO functionality of the ros_can interfacing node
* https://gitlab.com/eufs/ros_can . This is intended to be used with eufs_sim
* and controls and interfaces with the ADS-DV car for the FSUK competition
* Further specs: https://www.imeche.org/docs/default-source/1-oscar/formula-student/2019/fs-ai/ads-dv-software-interface-specification-v0-2.pdf?sfvrsn=2
*/

class StateMachine {
public:
    StateMachine(boost::shared_ptr<ros::NodeHandle> &nh);  ///< Constructor
    ~StateMachine();  ///< Destructor

    bool spinOnce();  ///< Main operational loop

    /**
     * Return if the car can drive based on the as_state
     */
    bool canDrive();

private:
    boost::shared_ptr<ros::NodeHandle> nh_;

    // TODO: Remove e_as_state_type enumerator, just use what is in the message
    // definition of internal state machine
    typedef enum e_as_state_type {
        AS_OFF = 0,
        AS_READY = 1,
        AS_DRIVING = 2,
        AS_EMERGENCY_BRAKE = 3,
        AS_FINISHED = 4,
    } as_state_type;

    as_state_type as_state_; ///< state machine state

    // TODO: Remove e_ami_state_type enumerator, just use what is in the message
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

    // Joint states published by the joint_state_controller of the Controller Manager
    ros::Subscriber set_mission_sub_;

    // High level robot command
    ros::Subscriber flag_sub_;

    ros::Publisher state_pub_;
    ros::Publisher state_pub_str_;

    ros::ServiceServer reset_srv_;  ///< service to reset state machine
    ros::ServiceServer ebs_srv_; ///< service to request an emergency brake


    /**
      * Stores the state of the driving flag
      * @param message of the driving flag
      */
    void flagCallback(std_msgs::Bool msg);

    /**
      * Sets the mission of the car. Only available in simulation
      */
    void setMission(eufs_msgs::CanState state);

    /**
      * Resets the state of the internal state machine
      */
    bool resetState(std_srvs::Trigger::Request &request, std_srvs::Trigger::Response &response);

    /**
      * Puts the car into EMERGENCY_BRAKE state and stops it
      */
    bool requestEBS(std_srvs::Trigger::Request &request, std_srvs::Trigger::Response &response);

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

#endif //ROBOT_CONTROL_STATE_MACHINE_H
