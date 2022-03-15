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

#ifndef EUFS_PLUGINS_GAZEBO_RACE_CAR_MODEL_INCLUDE_GAZEBO_RACE_CAR_MODEL_STATE_MACHINE_HPP_
#define EUFS_PLUGINS_GAZEBO_RACE_CAR_MODEL_INCLUDE_GAZEBO_RACE_CAR_MODEL_STATE_MACHINE_HPP_

// Gazebo Includes
#include <gazebo/common/Time.hh>

#include <chrono>  // NOLINT(build/c++11)
#include <iostream>
#include <thread>  // NOLINT(build/c++11)
#include <memory>

#include <rclcpp/rclcpp.hpp>
// ROS msgs
#include <eufs_msgs/msg/can_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>

// ROS  srvs
#include <std_srvs/srv/trigger.hpp>
#include <eufs_msgs/srv/set_can_state.hpp>



/**
 * @class StateMachine
 * @brief simulated ros_can for Gazebo
 *
 * @details Simulated IO functionality of the ros_can interfacing node
 * https://gitlab.com/eufs/ros_can . This is intended to be used with eufs_sim
 * and controls and interfaces with the ADS-DV car for the FSUK competition
 * Further specs:
 * https://www.imeche.org/docs/default-source/1-oscar/formula-student/2019/fs-ai/ads-dv-software-interface-specification-v0-2.pdf?sfvrsn=2
 */

namespace gazebo_plugins {
namespace eufs_plugins {

class StateMachine {
 public:
  explicit StateMachine(std::shared_ptr<rclcpp::Node> rosnode);  ///< Constructor
  ~StateMachine();                                      ///< Destructor

  void spinOnce(gazebo::common::Time current_time);  ///< Main operational loop

  /**
   * Return if the car can drive based on the as_state
   */
  bool canDrive();

 private:
  std::shared_ptr<rclcpp::Node> rosnode;

  uint16_t as_state_;  ///< state machine state

  uint16_t ami_state_;  ///< mission status

  bool mission_completed_;  ///< true only when selected mission has finished

  bool in_transition_;  ///< true when the state machine is currently transitioning from AS_READY to
                        ///< AS_DRIVING
  double transition_begin_;  ///< the world timestamp in which the transition from AS_READY to
                             ///< AS_DRIVING was begun


  // High level robot command
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr completed_sub_;

  rclcpp::Publisher<eufs_msgs::msg::CanState>::SharedPtr state_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_str_;

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr
      reset_srv_;  ///< service to reset state machine
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr
      ebs_srv_;  ///< service to request an emergency brake
  rclcpp::Service<eufs_msgs::srv::SetCanState>::SharedPtr
      set_mission_srv_;  ///< service to set mission

  /**
   * Stores the state of the mission complete flag
   * @param message of mission complete
   */
  void completedCallback(const std_msgs::msg::Bool::SharedPtr msg);

  /**
   * Sets the mission of the car. Only available in simulation
   */
  bool setMission(std::shared_ptr<eufs_msgs::srv::SetCanState::Request> request,
                  std::shared_ptr<eufs_msgs::srv::SetCanState::Response> response);

  /**
   * Resets the state of the internal state machine
   */
  bool resetState(std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                  std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  /**
   * Puts the car into EMERGENCY_BRAKE state and stops it
   */
  bool requestEBS(std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                  std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  /**
   * Loops through the internal state machine of the car
   * Based on:
   * https://www.imeche.org/docs/default-source/1-oscar/formula-student/2019/fs-ai/ads-dv-software-interface-specification-v0-2.pdf?sfvrsn=2
   */
  void updateState(gazebo::common::Time current_time);

  /**
   * Publishes internal state and mission in a eufs_msgs/msg/CanState.msg format
   */
  void publishState();

  /**
   * Creates a std_msgs/msg/String.msg version of the internal state and mission
   */
  std_msgs::msg::String makeStateString(const eufs_msgs::msg::CanState &state);
};

}  // namespace eufs_plugins
}  // namespace gazebo_plugins

#endif  // EUFS_PLUGINS_GAZEBO_RACE_CAR_MODEL_INCLUDE_GAZEBO_RACE_CAR_MODEL_STATE_MACHINE_HPP_
