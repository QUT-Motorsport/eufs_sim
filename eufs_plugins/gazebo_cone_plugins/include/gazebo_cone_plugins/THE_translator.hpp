/**
 * @file gazebo_cone_ground_truth.h
 * @author Niklas Burggraaff <s1902977@ed.ac.uk>
 * @date Mar 10, 2020
 * @copyright 2020 Edinburgh University Formula Student (EUFS)
 * @brief ground truth cone Gazebo plugin
 *
 * @details Provides ground truth cones in simulation in the form of
 *`eufs_msgs/ConeArrayWithCovariance`. Can also simulate the perception stack by publishing cones
 *with noise.
 **/

#ifndef EUFS_PLUGINS_GAZEBO_CONE_PLUGINS_INCLUDE_GAZEBO_CONE_PLUGINS_GAZEBO_GROUND_TRUTH_CONES_HPP_
#define EUFS_PLUGINS_GAZEBO_CONE_PLUGINS_INCLUDE_GAZEBO_CONE_PLUGINS_GAZEBO_GROUND_TRUTH_CONES_HPP_

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <cmath>
#include <eufs_msgs/msg/car_state.hpp>
#include <eufs_msgs/msg/cone_array.hpp>
#include <eufs_msgs/msg/cone_array_with_covariance.hpp>
#include <eufs_msgs/msg/cone_with_covariance.hpp>
#include <eufs_msgs/msg/point_array.hpp>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
#include <gazebo_ros/node.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <memory>
#include <string>
#include <utility>
#include <vector>
#include <map>

#include "rclcpp/rclcpp.hpp"
#include "yaml-cpp/yaml.h"

// ROS  srvs
#include <std_srvs/srv/trigger.hpp>










#endif 