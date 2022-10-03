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
 * @file gazebo_cone_ground_truth.cpp
 * @author Khalid Hersafril <s2016510@ed.ac.uk>
 * @date Mar 04, 2022
 * @copyright 2020 Edinburgh University Formula Student (EUFS)
 * @brief simulate bounding boxes Gazebo plugin
 *
 * @details Provides simulated bounding boxes for the use of perception
 * It publishes to the /simulate_bounding_boxes_topic.
 **/

#include "gazebo_simulate_bounding_boxes/gazebo_simulate_bounding_boxes.hpp"
#include "yaml-cpp/yaml.h"
#include <random>

namespace gazebo_plugins {
namespace eufs_plugins {

GZ_REGISTER_MODEL_PLUGIN(BoundingBoxesPlugin)

BoundingBoxesPlugin::BoundingBoxesPlugin() { this->seed = 0; }

// Gazebo plugin functions
void BoundingBoxesPlugin::Load(gazebo::physics::ModelPtr _parent, sdf::ElementPtr _sdf) {
  this->_world = _parent->GetWorld();
  this->rosnode_ = gazebo_ros::Node::Get(_sdf);

  if (!_sdf->HasElement("publish_rate")) {
    _publish_rate = 50.0;
  } else {
    _publish_rate = _sdf->GetElement("publish_rate")->Get<double>();
  }

  // Load camera Info parameter
  std::string bounding_boxes_yaml = "";
  if (!_sdf->HasElement("config_file")) {
    RCLCPP_FATAL(rosnode_->get_logger(),
                 "gazebo_simulate_bounding_boxes plugin missing <config_file>, cannot proceed");
    return;
  } else {
    bounding_boxes_yaml = _sdf->GetElement("config_file")->Get<std::string>();
  }

  // Load Bounding Boxes YAML File
  YAML::Node bounding_boxes_settings;
  try {
      bounding_boxes_settings = YAML::LoadFile(bounding_boxes_yaml);
  }
  catch(std::exception &e) {
      RCLCPP_FATAL(this->rosnode_->get_logger(), "Unable to load %s due to %s error.",
      bounding_boxes_yaml.c_str(), e.what());
      RCLCPP_FATAL(this->rosnode_->get_logger(),
      "BoundingBoxes plugin will not load.");
      RCLCPP_INFO(this->rosnode_->get_logger(),
      "TIP: You may want to re-check the path to your yaml file in the share directory");
  }

  // Assign camera info parameter
  K = bounding_boxes_settings["camera_callibration"]["K"].as<std::array<double, 9>>();
  R = bounding_boxes_settings["camera_callibration"]["R"].as<std::array<double, 9>>();
  P = bounding_boxes_settings["camera_callibration"]["P"].as<std::array<double, 12>>();
  D = bounding_boxes_settings["camera_callibration"]["D"].as<std::vector<double>>();
  distortion_model_ =
      bounding_boxes_settings["camera_callibration"]["distortion_model_"].as<std::string>();

  // Gaussian noise settings for bounding boxes
  mean_bb = bounding_boxes_settings["gaussian_noise_bounding_box"]["mean"].as<double>();
  standard_deviation_bb =
      bounding_boxes_settings["gaussian_noise_bounding_box"]["standard_deviation"].as<double>();

  mean_width = bounding_boxes_settings["gaussian_noise_width"]["mean"].as<double>();
  standard_deviation_width =
      bounding_boxes_settings["gaussian_noise_width"]["standard_deviation"].as<double>();

  mean_height = bounding_boxes_settings["gaussian_noise_height"]["mean"].as<double>();
  standard_deviation_height =
      bounding_boxes_settings["gaussian_noise_height"]["standard_deviation"].as<double>();

  // Node parameter
  this->target_frame_ = getStringParameter(_sdf, "targetFrame",
                                        "zed_right_camera_optical_frame", "Frame we transform to");
  this->source_frame_ = getStringParameter(_sdf, "sourceFrame",
                                              "base_footprint", "Frame we transform from");

  // // Camera resolution
  this->camera_width_ = getIntParameter(_sdf, "width", 1280, "Width of camera");
  this->camera_height_ = getIntParameter(_sdf, "height", 720, "Height of camera");

  // For the use of image projection
  getTransform(target_frame_, source_frame_);
  setCameraInfo(custom_cam_info_pub);

  camera_model.fromCameraInfo(cam_info_);

  // gt ≡ ground truth
  std::string gt_bounding_boxes_topic = getStringParameter(_sdf, "gtBoundingBoxesTopic",
                                      "ground_truth_bounding_boxes", "Bounding Boxes Publisher");

  std::string noisy_bounding_boxes_topic = getStringParameter(_sdf, "noisyBoundingBoxesTopic",
                                      "noisy_bounding_boxes", "Bounding Boxes Publisher");

  std::string custom_camera_info_topic_ = getStringParameter(_sdf, "customCameraInfo",
                                      "custom_camera_info", "Custom Camera Info");

  // Declare publisher
  this->ground_truth_bounding_boxes_pub =
          this->rosnode_->create_publisher<eufs_msgs::msg::BoundingBoxes>(
          gt_bounding_boxes_topic, 10);

  this->bounding_boxes_with_noise_pub =
        this->rosnode_->create_publisher<eufs_msgs::msg::BoundingBoxes>(
        noisy_bounding_boxes_topic, 10);

  this->custom_cam_info_pub =
          this->rosnode_->create_publisher<sensor_msgs::msg::CameraInfo>(
          custom_camera_info_topic_, 10);

  // Declare subscriber
  this->cone_ground_truth_sub_ =
          this->rosnode_->create_subscription<eufs_msgs::msg::ConeArrayWithCovariance>(
          "/ground_truth/cones", 10,
          std::bind(&BoundingBoxesPlugin::cones_callback, this, std::placeholders::_1));

  this->time_last_published = _world->SimTime();
  _last_sim_time = _world->SimTime();

  RCLCPP_INFO(this->rosnode_->get_logger(), "BoundingBoxesPlugin Loaded");
}  // GazeboConeGroundTruth

void BoundingBoxesPlugin::cones_callback(
                      const eufs_msgs::msg::ConeArrayWithCovariance::SharedPtr msg) {
  gazebo::common::Time curTime = _world->SimTime();
  double dt = (curTime - _last_sim_time).Double();

  // Checks if we should publish
  if (dt < (1 / _publish_rate)) {
    return;
  }

  // Reset the last simulation time
  _last_sim_time = curTime;

  std::vector<eufs_msgs::msg::ConeWithCovariance> blue_cones = msg->blue_cones;
  std::vector<eufs_msgs::msg::ConeWithCovariance> yellow_cones = msg->yellow_cones;
  std::vector<eufs_msgs::msg::ConeWithCovariance> orange_cones = msg->orange_cones;
  std::vector<eufs_msgs::msg::ConeWithCovariance> big_orange_cones = msg->big_orange_cones;

  // Get the cone position of each color and find it's bounding boxes position
  auto [ground_truth_blue_bounding_boxes, noisy_blue_bounding_boxes] =
      projectToImagePlane(blue_cones, small_cones);
  auto [ground_truth_yellow_bounding_boxes, noisy_yellow_bounding_boxes] =
      projectToImagePlane(yellow_cones, small_cones);
  auto [ground_truth_orange_bounding_boxes, noisy_orange_bounding_boxes] =
      projectToImagePlane(orange_cones, small_cones);
  auto [ground_truth_big_orange_bounding_boxes, noisy_big_orange_bounding_boxes] =
      projectToImagePlane(big_orange_cones, big_cones);

  eufs_msgs::msg::BoundingBoxes ground_truth_bounding_boxes_msg;
  update_msg(ground_truth_bounding_boxes_msg, msg->header,
          ground_truth_blue_bounding_boxes, ground_truth_yellow_bounding_boxes,
          ground_truth_orange_bounding_boxes, ground_truth_big_orange_bounding_boxes);

  eufs_msgs::msg::BoundingBoxes noisy_bounding_boxes_msg;

  update_msg(noisy_bounding_boxes_msg, msg->header,
            noisy_blue_bounding_boxes,
            noisy_yellow_bounding_boxes,
            noisy_orange_bounding_boxes,
            noisy_big_orange_bounding_boxes);

  // Publish camera info
  custom_cam_info_pub->publish(cam_info_);

  // Publish msg
  this->ground_truth_bounding_boxes_pub->publish(ground_truth_bounding_boxes_msg);
  this->bounding_boxes_with_noise_pub->publish(noisy_bounding_boxes_msg);
  }

void BoundingBoxesPlugin::update_msg(
  eufs_msgs::msg::BoundingBoxes &bounding_boxes_msg, std_msgs::msg::Header header,
  std::vector<std::vector<double>> blue_cones,
  std::vector<std::vector<double>> yellow_cones,
  std::vector<std::vector<double>> orange_cones,
  std::vector<std::vector<double>> big_orange_cones) {
  bounding_boxes_msg.header = header;
  bounding_boxes_msg.image_header = header;
  push_back_bounding_boxes_msg(bounding_boxes_msg, blue_cones, "blue");
  push_back_bounding_boxes_msg(bounding_boxes_msg, yellow_cones, "yellow");
  push_back_bounding_boxes_msg(bounding_boxes_msg, orange_cones, "orange");
  push_back_bounding_boxes_msg(bounding_boxes_msg, big_orange_cones, "big-orange");
}

void BoundingBoxesPlugin::getTransform(
                                  std::string target_frame_,
                                  std::string source_frame_) {
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_ =
                      std::make_unique<tf2_ros::Buffer>(this->rosnode_->get_clock());
  std::shared_ptr<tf2_ros::TransformListener> transform_listener_ =
                      std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  geometry_msgs::msg::TransformStamped transformStamped;

  bool transform_done = false;

  // Get the transformation between base_footprint to the camera_optical_frame (right/left)
  while (!transform_done) {
    try {
      transformStamped = tf_buffer_->lookupTransform(
        target_frame_, source_frame_, tf2::TimePointZero);
      transform_done = true;
    }
  catch(tf2::TransformException & ex) {
      RCLCPP_DEBUG(
          this->rosnode_->get_logger(), "Could not transform %s to %s: %s",
          target_frame_.c_str(), source_frame_.c_str(), ex.what());
    }
  }
  tf2::fromMsg(transformStamped.transform, transform);
}

void BoundingBoxesPlugin::setCameraInfo(
          rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr custom_cam_info_pub) {
  // Customly created camera callibration
  cam_info_.set__width(camera_width_);
  cam_info_.set__height(camera_height_);
  cam_info_.set__d(D);
  cam_info_.set__k(K);
  cam_info_.set__r(R);
  cam_info_.set__p(P);
  cam_info_.set__distortion_model(distortion_model_);
}

std::tuple<std::vector<std::vector<double>>, std::vector<std::vector<double>>>
    BoundingBoxesPlugin::projectToImagePlane(std::vector<eufs_msgs::msg::ConeWithCovariance>
                  &cones_vector, const ConeInfo cone_info_) {
  std::vector<std::vector<double>> bounding_boxes_vector;
  std::vector<std::vector<double>> noisy_bounding_boxes_vector;

  for (size_t i = 0; i  < cones_vector.size(); i++) {
    double x = cones_vector[i].point.x;
    double y = cones_vector[i].point.y;
    double z = cones_vector[i].point.z;

    tf2::Vector3 new_point(x, y, z);
    new_point = transform * new_point;

    // Ground Truth Bounding Boxes
    double y_min, y_max, z_min, z_max;
    y_min = new_point.y();
    z_min = new_point.z();
    y_max = new_point.y() - 2 * cone_info_.radius;
    z_max = new_point.z() + cone_info_.height;

    // For some reason, defining this in the header file messes up the bounding boxes visualizer
    // but defining it here seems to work.
    std::vector<double> ground_truth_bounding_box;

    // For right cones
    if (new_point.y() < 0) {
    top_left_point = camera_model.project3dToPixel(
                                  cv::Point3d(-y_min, -z_max, new_point.x() + cone_info_.radius));
    bottom_right_point = camera_model.project3dToPixel(
                                  cv::Point3d(-y_max, -z_min, new_point.x() - cone_info_.radius));
    ground_truth_bounding_box.insert(ground_truth_bounding_box.end(), {bottom_right_point.x,
                                      top_left_point.y, top_left_point.x, bottom_right_point.y});
    } else {
    // For left cones
    top_right_point = camera_model.project3dToPixel(
                                  cv::Point3d(-y_max, -z_max, new_point.x() + cone_info_.radius));
    bottom_left_point = camera_model.project3dToPixel(
                                  cv::Point3d(-y_min, -z_min, new_point.x() - cone_info_.radius));
    ground_truth_bounding_box.insert(ground_truth_bounding_box.end(), {top_right_point.x,
                                     top_right_point.y, bottom_left_point.x, bottom_left_point.y});
    }

    // Noisy Bounding Boxes
    // Randomize seed to generate different normal distribution value
    std::normal_distribution<double> distNRadius(mean_width, standard_deviation_width);
    std::normal_distribution<double> distNHeight(mean_height, standard_deviation_height);
    std::default_random_engine seed_height = generate_seed();
    std::default_random_engine seed_width = generate_seed();
    std::default_random_engine seed_bounding_box = generate_seed();

    double gaussian_noise_width = static_cast<double>(distNRadius(seed_width));
    double gaussian_noise_height = static_cast<double>(distNHeight(seed_height));

    std::vector<double> noisy_bounding_box;

    // For right cones
    if (new_point.y() < 0) {
      noisy_bounding_box.insert(noisy_bounding_box.end(),
                                {bottom_right_point.x + gaussian_noise_width,
                                  top_left_point.y + gaussian_noise_height,
                                  top_left_point.x - gaussian_noise_width,
                                  bottom_right_point.y - gaussian_noise_height});
    } else {
    // For left cones
      noisy_bounding_box.insert(noisy_bounding_box.end(),
                                {top_right_point.x + gaussian_noise_width,
                                  top_right_point.y + gaussian_noise_height,
                                  bottom_left_point.x - gaussian_noise_width,
                                  bottom_left_point.y - gaussian_noise_height});
    }

    // Add noise to the bounding box
    add_gaussian_noise_to_bounding_box(noisy_bounding_box, seed_bounding_box);

    bounding_boxes_vector.push_back(ground_truth_bounding_box);
    noisy_bounding_boxes_vector.push_back(noisy_bounding_box);
  }

  return {bounding_boxes_vector, noisy_bounding_boxes_vector};
}

std::default_random_engine BoundingBoxesPlugin::generate_seed() {
    unsigned seed = std::chrono::steady_clock::now().time_since_epoch().count();
    std::default_random_engine seed_generator(seed);
    return seed_generator;
}

void BoundingBoxesPlugin::add_gaussian_noise_to_bounding_box(
        std::vector<double> &bounding_box_vector, std::default_random_engine seed) {
  std::vector<double> bounding_box_with_noise;

  // Define random generator with Gaussian distribution
  std::normal_distribution<double> distN(mean_bb, standard_deviation_bb);

  double gaussian_noise = static_cast<double>(distN(seed));

  for (size_t i = 0; i < bounding_box_vector.size(); i++) {
    bounding_box_vector[i] += gaussian_noise;
  }
}

void BoundingBoxesPlugin::push_back_bounding_boxes_msg(
  eufs_msgs::msg::BoundingBoxes &bounding_boxes_msg,
  std::vector<std::vector<double>> bounding_boxes_vector, std::string color) const {
  for (size_t i = 0; i < bounding_boxes_vector.size(); i++) {
  eufs_msgs::msg::BoundingBox bounding_box_msg;
  bounding_box_msg.color = color;
  bounding_box_msg.xmax = bounding_boxes_vector[i][0];
  bounding_box_msg.ymax = bounding_boxes_vector[i][3];
  bounding_box_msg.xmin = bounding_boxes_vector[i][2];
  bounding_box_msg.ymin = bounding_boxes_vector[i][1];
  bounding_boxes_msg.bounding_boxes.push_back(bounding_box_msg);
  }
}

std::string BoundingBoxesPlugin::getStringParameter(
                                            sdf::ElementPtr _sdf, const char *element,
                                            std::string default_value,
                                            const char *default_description) {
  if (!_sdf->HasElement(element)) {
    RCLCPP_DEBUG(this->rosnode_->get_logger(),
                 "state_ground_truth plugin missing <%s>, defaults to %s", element,
                 default_description);
    return default_value;
  } else {
    return _sdf->GetElement(element)->Get<std::string>();
  }
}

int BoundingBoxesPlugin::getIntParameter(sdf::ElementPtr _sdf, const char *element,
                                                    int default_value,
                                                    const char *default_description) {
if (!_sdf->HasElement(element)) {
  RCLCPP_DEBUG(this->rosnode_->get_logger(),
                "state_ground_truth plugin missing <%s>, defaults to %s", element,
                default_description);
  return default_value;
} else {
  return _sdf->GetElement(element)->Get<int>();
}
}

}  // namespace eufs_plugins
}  // namespace gazebo_plugins
