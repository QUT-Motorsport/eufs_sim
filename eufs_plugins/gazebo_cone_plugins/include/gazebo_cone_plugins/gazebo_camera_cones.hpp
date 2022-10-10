/*MIT License
 *
 * Copyright (c) 2019 Edinburgh University Formula Student (EUFS)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 *         of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 *         to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *         copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 *         copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *         AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.*/

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

#ifndef EUFS_PLUGINS_GAZEBO_CONE_PLUGINS_INCLUDE_GAZEBO_CONE_PLUGINS_GAZEBO_CAMERA_CONES_HPP_
#define EUFS_PLUGINS_GAZEBO_CONE_PLUGINS_INCLUDE_GAZEBO_CONE_PLUGINS_GAZEBO_CAMERA_CONES_HPP_

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <cmath>
#include <eufs_msgs/msg/car_state.hpp>
#include <eufs_msgs/msg/cone_array_with_covariance.hpp>
#include <eufs_msgs/msg/cone_with_covariance.hpp>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
#include <gazebo_ros/node.hpp>
#include <memory>
#include <string>
#include <utility>
#include <vector>
#include <map>
#include <filesystem>

#include "rclcpp/rclcpp.hpp"
#include "yaml-cpp/yaml.h"

namespace gazebo_plugins {
namespace eufs_plugins {
class GazeboCameraCones : public gazebo::ModelPlugin {
 public:
  enum ConeType {blue, yellow, orange, big_orange};

  GazeboCameraCones();

  // Gazebo plugin functions
  void Load(gazebo::physics::ModelPtr _parent, sdf::ElementPtr _sdf);

  void UpdateChild();

  eufs_msgs::msg::ConeArrayWithCovariance processCones(
      eufs_msgs::msg::ConeArrayWithCovariance cones_to_process);

  std::vector<eufs_msgs::msg::ConeWithCovariance>
  fovCones(std::vector<eufs_msgs::msg::ConeWithCovariance> conesToCheck);

  // Add noise to the cone arrays
  eufs_msgs::msg::ConeArrayWithCovariance addConeNoise(
      eufs_msgs::msg::ConeArrayWithCovariance &cones_message);
  void addNoiseToConeArray(std::vector<eufs_msgs::msg::ConeWithCovariance> &cone_array);
  double GaussianKernel(double mu, double sigma);

  // Returns random cone colour with probability `weights`
  std::string pickColorWithProbability(const YAML::Node weights);
  std::map<std::string, std::vector<eufs_msgs::msg::ConeWithCovariance>> swapConeColors(
    std::map<std::string, std::vector<eufs_msgs::msg::ConeWithCovariance>> color_map);

  // Helper function for parameters
  bool getBoolParameter(sdf::ElementPtr _sdf, const char *element, bool default_value,
                        const char *default_description);
  double getDoubleParameter(sdf::ElementPtr _sdf, const char *element, double default_value,
                            const char *default_description);
  std::string getStringParameter(sdf::ElementPtr _sdf, const char *element,
                                 std::string default_value, const char *default_description);
  ignition::math::Pose3d getPose3dParameter(sdf::ElementPtr _sdf, const char *element,
                                                ignition::math::Pose3d default_value,
                                                const char *default_description);


  // Helper functions for determining whether a cone is in range
  bool inRangeOfCamera(eufs_msgs::msg::ConeWithCovariance cone);
  bool inFOVOfCamera(eufs_msgs::msg::ConeWithCovariance cone);

  // Publishers
  rclcpp::Publisher<eufs_msgs::msg::ConeArrayWithCovariance>::SharedPtr camera_cones_pub_;

  // Gazebo variables
  gazebo::physics::ModelPtr track_model;
  std::string car_frame_id;
  gazebo::physics::LinkPtr car_link;
  ignition::math::Pose3d car_pos;
  ignition::math::Pose3d camera_pos;


  // Parameters

  double camera_total_view_distance;
  double camera_min_view_distance;
  double camera_fov;
  double camera_a;
  double camera_b;
  YAML::Node recolor_config;

  double update_rate_;
  gazebo::common::Time time_last_published;


  // Required ROS gazebo plugin variables
  gazebo::physics::WorldPtr _world;
  gazebo::event::ConnectionPtr update_connection_;

  gazebo_ros::Node::SharedPtr rosnode_;

  unsigned int seed;
};

}  // namespace eufs_plugins
}  // namespace gazebo_plugins

#endif  // EUFS_PLUGINS_GAZEBO_CONE_PLUGINS_INCLUDE_GAZEBO_CONE_PLUGINS_GAZEBO_CAMERA_CONES_HPP_
