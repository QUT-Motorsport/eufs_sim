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
 * @file gazebo_ground_truth_cones.cpp
 * @author Niklas Burggraaff <s1902977@ed.ac.uk>
 * @date Mar 10, 2020
 * @copyright 2020 Edinburgh University Formula Student (EUFS)
 * @brief ground truth cone Gazebo plugin
 *
 * @details Provides ground truth cones in simulation in the form of
 *`eufs_msgs/msg/ConeArrayWithCovariance`. Can also simulate the perception stack by publishing
 *cones with noise.
 **/

#include "gazebo_cone_plugins/gazebo_camera_cones.hpp"
#include "gazebo_cone_plugins/ground_truth_getter.hpp"

namespace gazebo_plugins {
namespace eufs_plugins {

GZ_REGISTER_MODEL_PLUGIN(GazeboCameraCones)

GazeboCameraCones::GazeboCameraCones() { this->seed = 0; }

// Gazebo plugin functions

void GazeboCameraCones::Load(gazebo::physics::ModelPtr _parent, sdf::ElementPtr _sdf) {
  this->_world = _parent->GetWorld();
  this->rosnode_ = gazebo_ros::Node::Get(_sdf);

  RCLCPP_DEBUG(this->rosnode_->get_logger(), "Loading CameraConesPlugin");

  this->update_rate_ = getDoubleParameter(_sdf, "updateRate", 0,
                                          "0.0 (as fast as possible)");

  this->camera_total_view_distance = getDoubleParameter(_sdf, "cameraViewDistance",
                                                        10, "10");
  this->camera_min_view_distance = getDoubleParameter(_sdf, "cameraMinViewDistance",
                                                      1, "1");
  this->camera_fov = getDoubleParameter(_sdf, "cameraFOV", 1.918889,
                                        "1.918889  (110 degrees)");
  this->camera_a =
      getDoubleParameter(_sdf, "CameraDepthNoiseParameterA", 0.0184,
                         "0.0184");
  this->camera_b =
      getDoubleParameter(_sdf, "CameraDepthNoiseParameterB", 0.2106,
                         "0.2106");
  this->camera_pos =
      getPose3dParameter(_sdf, "cameraTF",
                         {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
                         "0.0, 0.0, 0.0, 0.0, 0.0, 0.0");

  std::string random_cone_color_yaml = "";
  std::filesystem::path config_path;
  if (!_sdf->HasElement("recolorConfig")) {
    RCLCPP_FATAL(this->rosnode_->get_logger(),
                 "camera cones plugin missing <recolorConfig>, cannot proceed");
    return;
  } else {
    std::filesystem::path plugins_share =
        ament_index_cpp::get_package_share_directory("eufs_plugins");
    random_cone_color_yaml = _sdf->GetElement("recolorConfig")->Get<std::string>();
    config_path = plugins_share / "config" / random_cone_color_yaml;
  }

  try {
    recolor_config = YAML::LoadFile(config_path);
  } catch (std::exception &e) {
    RCLCPP_FATAL(this->rosnode_->get_logger(), "Unable to load %s due to %s error.",
                 random_cone_color_yaml.c_str(), e.what());
    RCLCPP_FATAL(this->rosnode_->get_logger(),
    "Cone recoloring yaml will not load, cannot proceed");
  }

  // Set up the publishers
  // Camera cones publisher
  if (!_sdf->HasElement("cameraConesTopicName")) {
    RCLCPP_FATAL(this->rosnode_->get_logger(),
                 "camera cones plugin missing <cameraConesTopicName>, cannot proceed");
    return;
  } else {
    std::string topic_name_ = _sdf->GetElement("cameraConesTopicName")->Get<std::string>();
    this->camera_cones_pub_ =
        this->rosnode_->create_publisher<eufs_msgs::msg::ConeArrayWithCovariance>(
            topic_name_, 1);
  }

  this->time_last_published = _world->SimTime();
  this->track_model = _parent->GetWorld()->ModelByName("track");
  this->car_frame_id = "base_footprint";
  this->car_link = _parent->GetLink(this->car_frame_id);

  this->update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
      std::bind(&GazeboCameraCones::UpdateChild, this));
  RCLCPP_INFO(this->rosnode_->get_logger(), "CameraConesPlugin Loaded");
}  // GazeboCameraCones

void GazeboCameraCones::UpdateChild() {
  // Check if it is time to publish new data
  gazebo::common::Time cur_time = _world->SimTime();
  double dt = (cur_time - time_last_published).Double();
  if (dt < (1.0 / this->update_rate_)) {
    return;
  }

  // Update the time (This is here so that the time to publish all the messages does not affect the
  // update rate)
  this->time_last_published = cur_time;
  this->car_pos = this->car_link->WorldPose();

  // Get the track message
  eufs_msgs::msg::ConeArrayWithCovariance gazebo_cone_arrays_msg =
      cone_helpers::getGroundTruthCones(track_model, this->rosnode_->get_logger());
  eufs_msgs::msg::ConeArrayWithCovariance cone_arrays_msg =
      cone_helpers::translateToFrame(gazebo_cone_arrays_msg,
                                     this->car_pos,
                                     this->car_frame_id);
  cone_arrays_msg.header.stamp.sec = cur_time.sec;
  cone_arrays_msg.header.stamp.nanosec = cur_time.nsec;
  eufs_msgs::msg::ConeArrayWithCovariance ground_truth_cones_msg =
      processCones(cone_arrays_msg);

  // Publish the camera cones if it has subscribers
  if (this->camera_cones_pub_->get_subscription_count() > 0) {
    eufs_msgs::msg::ConeArrayWithCovariance camera_cones_msg =
        addConeNoise(ground_truth_cones_msg);
    this->camera_cones_pub_->publish(camera_cones_msg);
  }
}

eufs_msgs::msg::ConeArrayWithCovariance GazeboCameraCones::processCones(
    eufs_msgs::msg::ConeArrayWithCovariance cones) {

  std::vector<eufs_msgs::msg::ConeWithCovariance> new_blue;
  std::vector<eufs_msgs::msg::ConeWithCovariance> new_yellow;
  std::vector<eufs_msgs::msg::ConeWithCovariance> new_orange;
  std::vector<eufs_msgs::msg::ConeWithCovariance> new_big_orange;

  std::vector<eufs_msgs::msg::ConeWithCovariance> in_view;

  // blue
  in_view = GazeboCameraCones::fovCones(cones.blue_cones);
  new_blue.resize(new_blue.size() + in_view.size());
  copy(in_view.begin(), in_view.end(), new_blue.rbegin());

  // yellow
  in_view = GazeboCameraCones::fovCones(cones.yellow_cones);
  new_yellow.resize(new_yellow.size() + in_view.size());
  copy(in_view.begin(), in_view.end(), new_yellow.rbegin());

  // orange
  in_view = GazeboCameraCones::fovCones(cones.orange_cones);
  new_orange.resize(new_orange.size() + in_view.size());
  copy(in_view.begin(), in_view.end(), new_orange.rbegin());

  // big_orange
  in_view = GazeboCameraCones::fovCones(cones.big_orange_cones);
  new_big_orange.resize(new_big_orange.size() + in_view.size());
  copy(in_view.begin(), in_view.end(), new_big_orange.rbegin());

  cones.blue_cones = new_blue;
  cones.yellow_cones = new_yellow;
  cones.orange_cones = new_orange;
  cones.big_orange_cones = new_big_orange;

  return cones;
}

bool GazeboCameraCones::inRangeOfCamera(eufs_msgs::msg::ConeWithCovariance cone) {
  auto dist = std::pow(cone.point.x - this->camera_pos.Pos()[0], 2) +
              std::pow(cone.point.y - this->camera_pos.Pos()[1], 2);
  return camera_min_view_distance * camera_min_view_distance < dist &&
         dist < camera_total_view_distance * camera_total_view_distance;
}

bool GazeboCameraCones::inFOVOfCamera(eufs_msgs::msg::ConeWithCovariance cone) {
  float angle = atan2(cone.point.y - this->camera_pos.Pos()[1], cone.point.x -
                                                                    this->camera_pos.Pos()[0]);
  return abs(angle - this->camera_pos.Rot().Yaw()) < (this->camera_fov / 2);
}

std::vector<eufs_msgs::msg::ConeWithCovariance>
GazeboCameraCones::fovCones(std::vector<eufs_msgs::msg::ConeWithCovariance> conesToCheck) {
  std::vector<eufs_msgs::msg::ConeWithCovariance> cones_in_view;

  for (auto const &cone : conesToCheck) {
    bool camera_sees = inRangeOfCamera(cone) && inFOVOfCamera(cone);
    if (camera_sees) {cones_in_view.push_back(cone);}
  }
  return cones_in_view;
}

// Add noise to the cone arrays
eufs_msgs::msg::ConeArrayWithCovariance GazeboCameraCones::addConeNoise(
    eufs_msgs::msg::ConeArrayWithCovariance &cones_message) {
  eufs_msgs::msg::ConeArrayWithCovariance cones_message_with_noise = cones_message;

  addNoiseToConeArray(cones_message_with_noise.blue_cones);
  addNoiseToConeArray(cones_message_with_noise.yellow_cones);
  addNoiseToConeArray(cones_message_with_noise.orange_cones);
  addNoiseToConeArray(cones_message_with_noise.big_orange_cones);
  addNoiseToConeArray(cones_message_with_noise.unknown_color_cones);

  std::map<std::string, std::vector<eufs_msgs::msg::ConeWithCovariance>> color_map = {
    {"blue", cones_message_with_noise.blue_cones},
    {"yellow", cones_message_with_noise.yellow_cones},
    {"orange", cones_message_with_noise.orange_cones},
    {"big_orange", cones_message_with_noise.big_orange_cones},
    {"unknown_color", cones_message_with_noise.unknown_color_cones}};

  color_map = swapConeColors(color_map);

  cones_message_with_noise.blue_cones = color_map["blue"];
  cones_message_with_noise.yellow_cones = color_map["yellow"];
  cones_message_with_noise.orange_cones = color_map["orange"];
  cones_message_with_noise.big_orange_cones = color_map["big_orange"];
  cones_message_with_noise.unknown_color_cones = color_map["unknown_color"];


  cones_message_with_noise.header.frame_id = this->car_frame_id;
  cones_message_with_noise.header.stamp.sec = this->time_last_published.sec;
  cones_message_with_noise.header.stamp.nanosec = this->time_last_published.nsec;
  return cones_message_with_noise;
}

void GazeboCameraCones::addNoiseToConeArray(
    std::vector<eufs_msgs::msg::ConeWithCovariance> &cone_array) {
  for (unsigned int i = 0; i < cone_array.size(); i++) {
    // Camera noise
    auto dist = sqrt(cone_array[i].point.x * cone_array[i].point.x +
                     cone_array[i].point.y * cone_array[i].point.y);
    auto x_noise = camera_a * std::fmin(70.0, std::exp(camera_b * dist));
    auto y_noise = x_noise / 5;

    // Add noise in direction of cone position vector
    double par_x = cone_array[i].point.x / dist;
    double par_y = cone_array[i].point.y / dist;

    // Generate perpendicular unit vector
    auto perp_x = -1.0 * par_y;
    auto perp_y = par_x;

    // Create noise vector
    auto par_noise = GaussianKernel(0, x_noise);
    par_x *= par_noise;
    par_y *= par_noise;
    auto perp_noise = GaussianKernel(0, y_noise);
    perp_x *= perp_noise;
    perp_y *= perp_noise;

    // Add noise vector to cone pose
    cone_array[i].point.x += par_x + perp_x;
    cone_array[i].point.y += par_y + perp_y;
    cone_array[i].covariance = {x_noise, 0, 0, y_noise};
  }
}

double GazeboCameraCones::GaussianKernel(double mu, double sigma) {
  // using Box-Muller transform to generate two independent standard
  // normally distributed normal variables see wikipedia

  // normalized uniform random variable
  double U = static_cast<double>(rand_r(&this->seed)) / static_cast<double>(RAND_MAX);

  // normalized uniform random variable
  double V = static_cast<double>(rand_r(&this->seed)) / static_cast<double>(RAND_MAX);

  double X = sqrt(-2.0 * ::log(U)) * cos(2.0 * M_PI * V);
  // double Y = sqrt(-2.0 * ::log(U)) * sin(2.0*M_PI * V);

  // there are 2 indep. vars, we'll just use X
  // scale to our mu and sigma
  X = sigma * X + mu;
  return X;
}

// Returns pointer to cone array at random given weights
std::string GazeboCameraCones::pickColorWithProbability(
  const YAML::Node weights) {
    double rand = static_cast<double>(rand_r(&this->seed)) / static_cast<double>(RAND_MAX);
    float sum = 0.0;
    std::string color = "";
    for (YAML::const_iterator it = weights.begin(); it != weights.end(); it++) {
      sum += it->second.as<float>();
      if (rand <= sum && color.empty()) {
          color = it->first.as<std::string>();
        }
      }
    if (sum != 1.0f || color.empty()) {
      RCLCPP_WARN_ONCE(this->rosnode_->get_logger(),
      "Cone mis-coloring config invalid");
      RCLCPP_WARN_ONCE(this->rosnode_->get_logger(),
      "Total probability  %s config is %f", color.c_str(), sum);
    }
    return color;
}

std::map<std::string, std::vector<eufs_msgs::msg::ConeWithCovariance>>
GazeboCameraCones::swapConeColors(
  std::map<std::string, std::vector<eufs_msgs::msg::ConeWithCovariance>> color_map) {
  std::map<std::string, std::vector<eufs_msgs::msg::ConeWithCovariance>> new_map;
  for (auto const& [color, source] : color_map) {
    for (auto cone : source) {
      std::string rand_color = pickColorWithProbability(recolor_config[color]);
      if (rand_color != "undetected") {
        if (!new_map.count(rand_color)) {
          std::vector<eufs_msgs::msg::ConeWithCovariance> new_cones = {cone};
          new_map.insert({rand_color, new_cones});
        } else {
          new_map.find(rand_color)->second.push_back(cone);
        }
      }
    }
  }
  return new_map;
}

double GazeboCameraCones::getDoubleParameter(sdf::ElementPtr _sdf, const char *element,
                                                 double default_value,
                                                 const char *default_description) {
  if (!_sdf->HasElement(element)) {
    RCLCPP_DEBUG(this->rosnode_->get_logger(),
                 "camera cones plugin missing <%s>, defaults to %s", element,
                 default_description);
    return default_value;
  } else {
    return _sdf->GetElement(element)->Get<double>();
  }
}

ignition::math::Pose3d GazeboCameraCones::getPose3dParameter(
    sdf::ElementPtr _sdf, const char *element, ignition::math::Pose3d default_value,
    const char *default_description) {
  if (!_sdf->HasElement(element)) {
    RCLCPP_DEBUG(this->rosnode_->get_logger(),
                 "camera cones plugin missing <%s>, defaults to %s", element,
                 default_description);
    return default_value;
  } else {
    return _sdf->GetElement(element)->Get<ignition::math::Pose3d>();
  }
}

}  // namespace eufs_plugins
}  // namespace gazebo_plugins
