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
 * @author Niklas Burggraaff <s1902977@ed.ac.uk>
 * @date Mar 10, 2020
 * @copyright 2020 Edinburgh University Formula Student (EUFS)
 * @brief ground truth cone Gazebo plugin
 *
 * @details Provides ground truth cones in simulation in the form of
 *`eufs_msgs/msg/ConeArrayWithCovariance`. Can also simulate the perception stack by publishing
 *cones with noise.
 **/

#include "gazebo_cone_ground_truth/gazebo_cone_ground_truth.hpp"

namespace gazebo_plugins {
namespace eufs_plugins {

GZ_REGISTER_MODEL_PLUGIN(GazeboConeGroundTruth)

GazeboConeGroundTruth::GazeboConeGroundTruth() { this->seed = 0; }

// Gazebo plugin functions

void GazeboConeGroundTruth::Load(gazebo::physics::ModelPtr _parent, sdf::ElementPtr _sdf) {
  this->_world = _parent->GetWorld();
  this->rosnode_ = gazebo_ros::Node::Get(_sdf);

  RCLCPP_DEBUG(this->rosnode_->get_logger(), "Loading ConeGroundTruthPlugin");

  this->time_last_published = _world->SimTime();

  this->track_model = _parent->GetWorld()->ModelByName("track");
  this->car_link = _parent->GetLink("base_footprint");

  this->update_rate_ = getDoubleParameter(_sdf, "updateRate", 0, "0.0 (as fast as possible)");

  this->lidar_total_view_distance = getDoubleParameter(_sdf, "lidarViewDistance", 100, "100");
  this->camera_total_view_distance = getDoubleParameter(_sdf, "cameraViewDistance", 10, "10");
  this->lidar_min_view_distance = getDoubleParameter(_sdf, "lidarMinViewDistance", 1, "1");
  this->camera_min_view_distance = getDoubleParameter(_sdf, "cameraMinViewDistance", 1, "1");
  this->lidar_x_view_distance = getDoubleParameter(_sdf, "lidarXViewDistance", 20, "20");
  this->lidar_y_view_distance = getDoubleParameter(_sdf, "lidarYViewDistance", 10, "10");
  this->lidar_fov = getDoubleParameter(_sdf, "lidarFOV", 3.141593, "3.141593  (180 degrees)");
  this->camera_fov = getDoubleParameter(_sdf, "cameraFOV", 1.918889, "1.918889  (110 degrees)");
  this->camera_a =
      getDoubleParameter(_sdf, "perceptionCameraDepthNoiseParameterA", 0.0184, "0.0184");
  this->camera_b =
      getDoubleParameter(_sdf, "perceptionCameraDepthNoiseParameterB", 0.2106, "0.2106");
  this->camera_noise_percentage =
      getDoubleParameter(_sdf, "perceptionCameraNoisePercentage", 0.4, "0.4");
  this->lidar_on = getBoolParameter(_sdf, "lidarOn", true, "true");

  this->track_frame_ = getStringParameter(_sdf, "trackFrame", "map", "map");
  this->cone_frame_ = "base_footprint";

  this->simulate_perception_ = getBoolParameter(_sdf, "simulatePerception", false, "false");

  this->perception_lidar_noise_ =
      getVector3dParameter(_sdf, "perceptionNoise", {0.03, 0.03, 0.0}, "0.03, 0.03, 0.0");

  // Setup the publishers

  // Ground truth cone publisher
  if (!_sdf->HasElement("groundTruthConesTopicName")) {
    RCLCPP_FATAL(this->rosnode_->get_logger(),
                 "state_ground_truth plugin missing <groundTruthConesTopicName>, cannot proceed");
    return;
  } else {
    std::string topic_name_ = _sdf->GetElement("groundTruthConesTopicName")->Get<std::string>();
    this->ground_truth_cone_pub_ =
        this->rosnode_->create_publisher<eufs_msgs::msg::ConeArrayWithCovariance>(topic_name_, 1);
  }

  // Ground truth cone marker publisher
  if (!_sdf->HasElement("groundTruthConeMarkersTopicName")) {
    RCLCPP_FATAL(
        this->rosnode_->get_logger(),
        "state_ground_truth plugin missing <groundTruthConeMarkersTopicName>, cannot proceed");
    return;
  } else {
    std::string topic_name_ =
        _sdf->GetElement("groundTruthConeMarkersTopicName")->Get<std::string>();
    this->ground_truth_cone_marker_pub_ =
        this->rosnode_->create_publisher<visualization_msgs::msg::MarkerArray>(topic_name_, 1);
  }

  // Ground truth track publisher
  if (!_sdf->HasElement("groundTruthTrackTopicName")) {
    RCLCPP_FATAL(this->rosnode_->get_logger(),
                 "state_ground_truth plugin missing <groundTruthTrackTopicName>, cannot proceed");
    return;
  } else {
    std::string topic_name_ = _sdf->GetElement("groundTruthTrackTopicName")->Get<std::string>();
    this->ground_truth_track_pub_ =
        this->rosnode_->create_publisher<eufs_msgs::msg::ConeArrayWithCovariance>(topic_name_, 1);
    std::string viz_topic = topic_name_ + "/viz";
    this->ground_truth_track_viz_pub_ =
        this->rosnode_->create_publisher<visualization_msgs::msg::MarkerArray>(viz_topic, 1);
  }

  if (!_sdf->HasElement("pubGroundTruth")) {
    RCLCPP_FATAL(this->rosnode_->get_logger(),
                 "state_ground_truth plugin missing <pubGroundTruth>, cannot proceed");
    return;
  } else {
    pub_ground_truth = _sdf->GetElement("pubGroundTruth")->Get<bool>();
  }

  if (this->simulate_perception_) {
    // Camera cone publisher
    if (!_sdf->HasElement("perceptionConesTopicName")) {
      RCLCPP_FATAL(this->rosnode_->get_logger(),
                   "state_ground_truth plugin missing <perceptionConesTopicName>, cannot proceed");
      return;
    } else {
      std::string topic_name_ = _sdf->GetElement("perceptionConesTopicName")->Get<std::string>();
      this->perception_cone_pub_ =
          this->rosnode_->create_publisher<eufs_msgs::msg::ConeArrayWithCovariance>(topic_name_, 1);
    }

    // Camera cone marker publisher
    if (!_sdf->HasElement("perceptionConeMarkersTopicName")) {
      RCLCPP_FATAL(
          this->rosnode_->get_logger(),
          "state_ground_truth plugin missing <perceptionConeMarkersTopicName>, cannot proceed");
      return;
    } else {
      std::string topic_name_ =
          _sdf->GetElement("perceptionConeMarkersTopicName")->Get<std::string>();
      this->perception_cone_marker_pub_ =
          this->rosnode_->create_publisher<visualization_msgs::msg::MarkerArray>(topic_name_, 1);
    }
  }

  // Setup Services

  // Cone position reset service
  this->reset_cone_pos_srv = this->rosnode_->create_service<std_srvs::srv::Trigger>(
      "/ros_can/reset_cone_pos", std::bind(&GazeboConeGroundTruth::resetConePosition, this,
                                           std::placeholders::_1, std::placeholders::_2));

  std::string eufs_tracks_directory = ament_index_cpp::get_package_share_directory("eufs_tracks");
  cone_big_mesh_path = "file:///" + eufs_tracks_directory + "/meshes/cone_big.dae";
  cone_mesh_path = "file:///" + eufs_tracks_directory + "/meshes/cone.dae";

  this->initial_car_pos_ = this->car_link->WorldPose();

  this->update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
      std::bind(&GazeboConeGroundTruth::UpdateChild, this));

  //  Store initial track
  this->initial_track = this->getConeArraysMessage();

  RCLCPP_INFO(this->rosnode_->get_logger(), "ConeGroundTruthPlugin Loaded");
}  // GazeboConeGroundTruth

void GazeboConeGroundTruth::UpdateChild() {
  // Check if it is time to publish new data
  gazebo::common::Time cur_time = _world->SimTime();
  double dt = (cur_time - time_last_published).Double();
  if (dt < (1.0 / this->update_rate_)) {
    return;
  }

  // Update the time (This is here so that the time to publish all the messages does not affect the
  // update rate)
  this->time_last_published = cur_time;

  // Check if there is a reason to publish the data
  if (this->ground_truth_cone_pub_->get_subscription_count() == 0 &&
      this->ground_truth_cone_marker_pub_->get_subscription_count() == 0 &&
      this->ground_truth_track_pub_->get_subscription_count() == 0 &&
      this->ground_truth_track_viz_pub_->get_subscription_count() == 0 &&
      (!this->simulate_perception_ ||
       (this->perception_cone_pub_->get_subscription_count() == 0 &&
        this->perception_cone_marker_pub_->get_subscription_count() == 0))) {
    RCLCPP_DEBUG(this->rosnode_->get_logger(),
                 "Nobody is listening to cone_ground_truth. Doing nothing");
    return;
  }

  this->car_pos = this->car_link->WorldPose();

  // Get the track message
  eufs_msgs::msg::ConeArrayWithCovariance cone_arrays_message = getConeArraysMessage();

  eufs_msgs::msg::ConeArrayWithCovariance ground_truth_track_message;
  if (this->track_frame_ == "map") {
    ground_truth_track_message = translateMapFrame(cone_arrays_message);
  } else if (this->track_frame_ == "base_footprint") {
    ground_truth_track_message = translateBaseFootprintFrame(cone_arrays_message);
  } else {
    RCLCPP_WARN(this->rosnode_->get_logger(),
                "Can only publish track to \"map\" or \"base_footprint\" "
                "frame and not: \"%s\"",
                this->track_frame_.c_str());
    return;
  }

  // Publish the ground truth track if it has subscribers and is allowed to publish
  if (this->ground_truth_track_pub_->get_subscription_count() > 0 && pub_ground_truth) {
    this->ground_truth_track_pub_->publish(ground_truth_track_message);
  }

  // Publish the ground truth track markers if it has subscribers and is allowed to publish
  if (this->ground_truth_track_viz_pub_->get_subscription_count() > 0 && pub_ground_truth) {
    visualization_msgs::msg::MarkerArray ground_truth_track_marker_array_message =
        getConeMarkerArrayMessage(ground_truth_track_message);
    this->ground_truth_track_viz_pub_->publish(ground_truth_track_marker_array_message);
  }

  eufs_msgs::msg::ConeArrayWithCovariance ground_truth_cones_message =
      processCones(cone_arrays_message);

  // Publish the ground truth cones if it has subscribers and is allowed to publish
  if (this->ground_truth_cone_pub_->get_subscription_count() > 0 && pub_ground_truth) {
    this->ground_truth_cone_pub_->publish(ground_truth_cones_message);
  }

  // Publish the ground truth cone markers if it has subscribers and is allowed to publish
  if (this->ground_truth_cone_marker_pub_->get_subscription_count() > 0 && pub_ground_truth) {
    visualization_msgs::msg::MarkerArray ground_truth_cone_marker_array_message =
        getConeMarkerArrayMessage(ground_truth_cones_message);

    ground_truth_cone_markers_published = ground_truth_cone_marker_array_message.markers.size();
    this->removeExcessCones(ground_truth_cone_marker_array_message.markers,
                            ground_truth_cone_markers_published,
                            prev_ground_truth_cone_markers_published);
    this->ground_truth_cone_marker_pub_->publish(ground_truth_cone_marker_array_message);
    this->prev_ground_truth_cone_markers_published = ground_truth_cone_markers_published;
  }

  // Publish the simulated perception cones if it has subscribers
  if (this->simulate_perception_ &&
      (this->perception_cone_pub_->get_subscription_count() > 0 ||
       this->perception_cone_marker_pub_->get_subscription_count() > 0)) {
    eufs_msgs::msg::ConeArrayWithCovariance perception_cones_message =
        addNoisePerception(ground_truth_cones_message, perception_lidar_noise_);

    if (this->perception_cone_pub_->get_subscription_count() > 0) {
      this->perception_cone_pub_->publish(perception_cones_message);
    }

    if (this->perception_cone_marker_pub_->get_subscription_count() > 0) {
      visualization_msgs::msg::MarkerArray perception_cone_marker_array_message =
          getConeMarkerArrayMessage(perception_cones_message);

      perception_cone_markers_published = perception_cone_marker_array_message.markers.size();
      this->removeExcessCones(perception_cone_marker_array_message.markers,
                              perception_cone_markers_published,
                              prev_perception_cone_markers_published);
      this->perception_cone_marker_pub_->publish(perception_cone_marker_array_message);
      this->prev_perception_cone_markers_published = perception_cone_markers_published;
    }
  }
}

void GazeboConeGroundTruth::removeExcessCones(
    std::vector<visualization_msgs::msg::Marker> &marker_array, const int current_array_length,
    const int prev_array_length) {
  for (int i = current_array_length; i < prev_array_length; i++) {
    visualization_msgs::msg::Marker deleted_marker;
    deleted_marker.id = i;
    deleted_marker.action = deleted_marker.DELETE;
    marker_array.push_back(deleted_marker);
  }
}

// Getting the track
eufs_msgs::msg::ConeArrayWithCovariance GazeboConeGroundTruth::getConeArraysMessage() {
  eufs_msgs::msg::ConeArrayWithCovariance cone_arrays_message;
  cone_arrays_message.header.frame_id = "map";
  cone_arrays_message.header.stamp.sec = this->time_last_published.sec;
  cone_arrays_message.header.stamp.nanosec = this->time_last_published.nsec;

  if (this->track_model != nullptr) {
    gazebo::physics::Link_V links = this->track_model->GetLinks();
    for (unsigned int i = 0; i < links.size(); i++) {
      addConeToConeArray(cone_arrays_message, links[i]);
    }
  }

  return cone_arrays_message;
}

void GazeboConeGroundTruth::addConeToConeArray(
    eufs_msgs::msg::ConeArrayWithCovariance &ground_truth_cone_array,
    gazebo::physics::LinkPtr link) {
  geometry_msgs::msg::Point point;
  point.x = link->WorldPose().Pos().X();
  point.y = link->WorldPose().Pos().Y();
  point.z = 0;

  ConeType cone_type = this->getConeType(link);

  eufs_msgs::msg::ConeWithCovariance cone = eufs_msgs::msg::ConeWithCovariance();
  cone.point = point;
  cone.covariance = {0, 0, 0, 0};

  switch (cone_type) {
    case ConeType::blue:
      ground_truth_cone_array.blue_cones.push_back(cone);
      break;
    case ConeType::yellow:
      ground_truth_cone_array.yellow_cones.push_back(cone);
      break;
    case ConeType::orange:
      ground_truth_cone_array.orange_cones.push_back(cone);
      break;
    case ConeType::big_orange:
      ground_truth_cone_array.big_orange_cones.push_back(cone);
      break;
    case ConeType::unknown:
      ground_truth_cone_array.unknown_color_cones.push_back(cone);
      break;
  }
}

eufs_msgs::msg::ConeArrayWithCovariance GazeboConeGroundTruth::processCones(
    eufs_msgs::msg::ConeArrayWithCovariance cones_to_process) {
  eufs_msgs::msg::ConeArrayWithCovariance cones = translateBaseFootprintFrame(cones_to_process);

  std::vector<eufs_msgs::msg::ConeWithCovariance> new_blue;
  std::vector<eufs_msgs::msg::ConeWithCovariance> new_yellow;
  std::vector<eufs_msgs::msg::ConeWithCovariance> new_orange;
  std::vector<eufs_msgs::msg::ConeWithCovariance> new_big_orange;
  std::vector<eufs_msgs::msg::ConeWithCovariance> new_unknown;

  std::vector<eufs_msgs::msg::ConeWithCovariance> color, no_color;

  // blue
  std::tie(color, no_color) = GazeboConeGroundTruth::fovCones(cones.blue_cones);
  new_blue.resize(new_blue.size() + color.size());
  copy(color.begin(), color.end(), new_blue.rbegin());
  new_unknown.resize(new_unknown.size() + no_color.size());
  copy(no_color.begin(), no_color.end(), new_unknown.rbegin());

  // yellow
  std::tie(color, no_color) = GazeboConeGroundTruth::fovCones(cones.yellow_cones);
  new_yellow.resize(new_yellow.size() + color.size());
  copy(color.begin(), color.end(), new_yellow.rbegin());
  new_unknown.resize(new_unknown.size() + no_color.size());
  copy(no_color.begin(), no_color.end(), new_unknown.rbegin());

  // orange
  std::tie(color, no_color) = GazeboConeGroundTruth::fovCones(cones.orange_cones);
  new_orange.resize(new_orange.size() + color.size());
  copy(color.begin(), color.end(), new_orange.rbegin());
  new_unknown.resize(new_unknown.size() + no_color.size());
  copy(no_color.begin(), no_color.end(), new_unknown.rbegin());

  // big_orange
  std::tie(color, no_color) = GazeboConeGroundTruth::fovCones(cones.big_orange_cones);
  new_big_orange.resize(new_big_orange.size() + color.size());
  copy(color.begin(), color.end(), new_big_orange.rbegin());
  new_unknown.resize(new_unknown.size() + no_color.size());
  copy(no_color.begin(), no_color.end(), new_unknown.rbegin());

  // unknown
  std::tie(color, no_color) = GazeboConeGroundTruth::fovCones(cones.unknown_color_cones);
  new_unknown.resize(new_unknown.size() + color.size());
  copy(color.begin(), color.end(), new_unknown.rbegin());
  new_unknown.resize(new_unknown.size() + no_color.size());
  copy(no_color.begin(), no_color.end(), new_unknown.rbegin());

  cones.blue_cones = new_blue;
  cones.yellow_cones = new_yellow;
  cones.orange_cones = new_orange;
  cones.big_orange_cones = new_big_orange;
  cones.unknown_color_cones = new_unknown;

  return cones;
}

// Resets the position of cones to initial track model
bool GazeboConeGroundTruth::resetConePosition(
    std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
  (void)request;   // suppress unused parameter warning
  (void)response;  // suppress unused parameter warning

  gazebo::physics::Link_V links = this->track_model->GetLinks();

  // Initialise counters
  int blue_i = 0;
  int yellow_i = 0;
  int orange_i = 0;
  int big_orange_i = 0;
  int unknown_color_i = 0;

  // Loop through all cones
  for (unsigned int i = 0; i < links.size(); i++) {
    eufs_msgs::msg::ConeWithCovariance cone;
    ConeType cone_type = this->getConeType(links[i]);

    // sort by cone colour
    switch (cone_type) {
      case ConeType::blue:
        cone = this->initial_track.blue_cones[blue_i];
        blue_i++;
        break;
      case ConeType::yellow:
        cone = this->initial_track.yellow_cones[yellow_i];
        yellow_i++;
        break;
      case ConeType::orange:
        cone = this->initial_track.orange_cones[orange_i];
        orange_i++;
        break;
      case ConeType::big_orange:
        cone = this->initial_track.big_orange_cones[big_orange_i];
        big_orange_i++;
        break;
      case ConeType::unknown:
        cone = this->initial_track.unknown_color_cones[unknown_color_i];
        unknown_color_i++;
        break;
    }

    // Initial position and velocity variables
    const ignition::math::Pose3d pos(cone.point.x, cone.point.y, cone.point.z, 0.0, 0.0, 0.0);
    const ignition::math::Vector3d vel(0.0, 0.0, 0.0);
    const ignition::math::Vector3d angular(0.0, 0.0, 0.0);

    // Set cone position to initial position (and velocity)
    links[i]->SetWorldPose(pos);
    links[i]->SetAngularVel(vel);
    links[i]->SetLinearVel(angular);
  }

  return response->success;
}

bool GazeboConeGroundTruth::inRangeOfCamera(eufs_msgs::msg::ConeWithCovariance cone) {
  auto dist = (cone.point.x * cone.point.x) + (cone.point.y * cone.point.y);
  return camera_min_view_distance * camera_min_view_distance < dist &&
         dist < camera_total_view_distance * camera_total_view_distance;
}

bool GazeboConeGroundTruth::inFOVOfCamera(eufs_msgs::msg::ConeWithCovariance cone) {
  float angle = atan2(cone.point.y, cone.point.x);
  return abs(angle) < (this->camera_fov / 2);
}

bool GazeboConeGroundTruth::inRangeOfLidar(eufs_msgs::msg::ConeWithCovariance cone) {
  auto dist = (cone.point.x * cone.point.x) + (cone.point.y * cone.point.y);
  return lidar_min_view_distance * lidar_min_view_distance < dist &&
         dist < lidar_total_view_distance * lidar_total_view_distance &&
         abs(cone.point.x) < lidar_x_view_distance && abs(cone.point.y) < lidar_y_view_distance &&
         this->lidar_on;
}

bool GazeboConeGroundTruth::inFOVOfLidar(eufs_msgs::msg::ConeWithCovariance cone) {
  float angle = atan2(cone.point.y, cone.point.x);
  return abs(angle) < (this->lidar_fov / 2) && this->lidar_on;
}

std::vector<eufs_msgs::msg::ConeWithCovariance> GazeboConeGroundTruth::translateCones(
    std::vector<eufs_msgs::msg::ConeWithCovariance> cones, ignition::math::Pose3d frame) {
  std::vector<eufs_msgs::msg::ConeWithCovariance> translated_cones;
  for (auto const &cone : cones) {
    // Translate the position of the cone to be based on the car
    float x = cone.point.x - frame.Pos().X();
    float y = cone.point.y - frame.Pos().Y();

    // If the cone is withing viewing distance of lidar
    float yaw = frame.Rot().Yaw();

    // Rotate the points using the yaw of the car (x and y are the other way around)
    eufs_msgs::msg::ConeWithCovariance translated_cone;
    translated_cone.point.y = (cos(yaw) * y) - (sin(yaw) * x);
    translated_cone.point.x = (sin(yaw) * y) + (cos(yaw) * x);

    translated_cones.push_back(translated_cone);
  }
  return translated_cones;
}

eufs_msgs::msg::ConeArrayWithCovariance GazeboConeGroundTruth::translateMapFrame(
    eufs_msgs::msg::ConeArrayWithCovariance cones) {
  cones.header.frame_id = "map";
  cones.blue_cones = translateCones(cones.blue_cones, this->initial_car_pos_);
  cones.yellow_cones = translateCones(cones.yellow_cones, this->initial_car_pos_);
  cones.orange_cones = translateCones(cones.orange_cones, this->initial_car_pos_);
  cones.big_orange_cones = translateCones(cones.big_orange_cones, this->initial_car_pos_);
  cones.unknown_color_cones = translateCones(cones.unknown_color_cones, this->initial_car_pos_);
  return cones;
}

eufs_msgs::msg::ConeArrayWithCovariance GazeboConeGroundTruth::translateBaseFootprintFrame(
    eufs_msgs::msg::ConeArrayWithCovariance cones) {
  cones.header.frame_id = "base_footprint";
  cones.blue_cones = translateCones(cones.blue_cones, this->car_pos);
  cones.yellow_cones = translateCones(cones.yellow_cones, this->car_pos);
  cones.orange_cones = translateCones(cones.orange_cones, this->car_pos);
  cones.big_orange_cones = translateCones(cones.big_orange_cones, this->car_pos);
  cones.unknown_color_cones = translateCones(cones.unknown_color_cones, this->car_pos);
  return cones;
}

std::pair<std::vector<eufs_msgs::msg::ConeWithCovariance>,
          std::vector<eufs_msgs::msg::ConeWithCovariance>>
GazeboConeGroundTruth::fovCones(std::vector<eufs_msgs::msg::ConeWithCovariance> conesToCheck) {
  std::vector<eufs_msgs::msg::ConeWithCovariance> cones_in_view;
  std::vector<eufs_msgs::msg::ConeWithCovariance> cones_in_view_without_color;

  for (auto const &cone : conesToCheck) {
    bool lidar_sees = inRangeOfLidar(cone) && inFOVOfLidar(cone);
    bool camera_sees = inRangeOfCamera(cone) && inFOVOfCamera(cone);

    if ((lidar_sees && !camera_sees)) {
      cones_in_view_without_color.push_back(cone);
    } else if (camera_sees) {
      cones_in_view.push_back(cone);
    }
  }
  return std::make_pair(cones_in_view, cones_in_view_without_color);
}

GazeboConeGroundTruth::ConeType GazeboConeGroundTruth::getConeType(gazebo::physics::LinkPtr link) {
  std::string link_name = link->GetName();

  if (link_name.substr(0, 9) == "blue_cone") {
    return ConeType::blue;
  }
  if (link_name.substr(0, 11) == "yellow_cone") {
    return ConeType::yellow;
  }
  if (link_name.substr(0, 11) == "orange_cone") {
    return ConeType::orange;
  }
  if (link_name.substr(0, 8) == "big_cone") {
    return ConeType::big_orange;
  }

  RCLCPP_WARN_ONCE(this->rosnode_->get_logger(),
                   "Cannot get cone type from link in the track model in simulation: %s",
                   link_name.c_str());
  return ConeType::unknown;
}

// Getting the cone marker array

visualization_msgs::msg::MarkerArray GazeboConeGroundTruth::getConeMarkerArrayMessage(
    eufs_msgs::msg::ConeArrayWithCovariance &cones_message) {
  visualization_msgs::msg::MarkerArray ground_truth_cone_marker_array;
  std::string frame = cones_message.header.frame_id;

  int marker_id = 0;
  marker_id = addConeMarkers(ground_truth_cone_marker_array.markers, marker_id, frame,
                             cones_message.blue_cones, 0.2, 0.2, 1, false);
  marker_id = addConeMarkers(ground_truth_cone_marker_array.markers, marker_id, frame,
                             cones_message.yellow_cones, 1, 1, 0, false);
  marker_id = addConeMarkers(ground_truth_cone_marker_array.markers, marker_id, frame,
                             cones_message.orange_cones, 1, 0.549, 0, false);
  marker_id = addConeMarkers(ground_truth_cone_marker_array.markers, marker_id, frame,
                             cones_message.big_orange_cones, 1, 0.271, 0, true);
  marker_id = addConeMarkers(ground_truth_cone_marker_array.markers, marker_id, frame,
                             cones_message.unknown_color_cones, 0.7, 0.7, 0.7, false);

  return ground_truth_cone_marker_array;
}

int GazeboConeGroundTruth::addConeMarkers(
    std::vector<visualization_msgs::msg::Marker> &marker_array, int marker_id, std::string frame,
    std::vector<eufs_msgs::msg::ConeWithCovariance> cones, float red, float green, float blue,
    bool big) {
  int id = marker_id;
  for (unsigned int i = 0; i < cones.size(); i++) {
    visualization_msgs::msg::Marker marker;
    marker.header.stamp.sec = this->time_last_published.sec;
    marker.header.stamp.nanosec = this->time_last_published.nsec;
    marker.header.frame_id = frame;

    marker.id = id;

    marker.type = marker.MESH_RESOURCE;
    marker.action = marker.ADD;

    marker.pose.position.x = cones[i].point.x;
    marker.pose.position.y = cones[i].point.y;

    marker.scale.x = 1.5;
    marker.scale.y = 1.5;
    marker.scale.z = 1.5;

    if (big) {
      marker.mesh_resource = cone_big_mesh_path;
    } else {
      marker.mesh_resource = cone_mesh_path;
    }

    marker.color.r = red;
    marker.color.g = green;
    marker.color.b = blue;
    marker.color.a = 1.0;

    marker.lifetime = rclcpp::Duration::from_seconds(0.2);

    marker_array.push_back(marker);

    id++;
  }
  return id;
}

// Add noise to the cone arrays
eufs_msgs::msg::ConeArrayWithCovariance GazeboConeGroundTruth::addNoisePerception(
    eufs_msgs::msg::ConeArrayWithCovariance &cones_message, ignition::math::Vector3d noise) {
  eufs_msgs::msg::ConeArrayWithCovariance cones_message_with_noise = cones_message;

  addNoiseToConeArray(cones_message_with_noise.blue_cones, noise);
  addNoiseToConeArray(cones_message_with_noise.yellow_cones, noise);
  addNoiseToConeArray(cones_message_with_noise.orange_cones, noise);
  addNoiseToConeArray(cones_message_with_noise.big_orange_cones, noise);
  addNoiseToConeArray(cones_message_with_noise.unknown_color_cones, noise);

  cones_message_with_noise.header.frame_id = this->cone_frame_;
  cones_message_with_noise.header.stamp.sec = this->time_last_published.sec;
  cones_message_with_noise.header.stamp.nanosec = this->time_last_published.nsec;

  return cones_message_with_noise;
}

void GazeboConeGroundTruth::addNoiseToConeArray(
    std::vector<eufs_msgs::msg::ConeWithCovariance> &cone_array, ignition::math::Vector3d noise) {
  for (unsigned int i = 0; i < cone_array.size(); i++) {
    // Lidar noise
    auto lidar_x_noise = noise.X();
    auto lidar_y_noise = noise.Y();

    // Camera noise
    auto dist = sqrt(cone_array[i].point.x * cone_array[i].point.x +
                     cone_array[i].point.y * cone_array[i].point.y);
    auto camera_x_noise = camera_a * std::exp(camera_b * dist);
    auto camera_y_noise = camera_x_noise / 5;

    // Fuse noise
    auto x_noise = (camera_noise_percentage * camera_x_noise) +
                   ((1 - camera_noise_percentage) * lidar_x_noise);
    auto y_noise = (camera_noise_percentage * camera_y_noise) +
                   ((1 - camera_noise_percentage) * lidar_y_noise);

    // Apply noise
    cone_array[i].point.x += GaussianKernel(0, x_noise);
    cone_array[i].point.y += GaussianKernel(0, y_noise);
    cone_array[i].covariance = {x_noise, 0, 0, y_noise};
  }
}

double GazeboConeGroundTruth::GaussianKernel(double mu, double sigma) {
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

// Helper function for parameters
bool GazeboConeGroundTruth::getBoolParameter(sdf::ElementPtr _sdf, const char *element,
                                             bool default_value, const char *default_description) {
  if (!_sdf->HasElement(element)) {
    RCLCPP_DEBUG(this->rosnode_->get_logger(),
                 "state_ground_truth plugin missing <%s>, defaults to %s", element,
                 default_description);
    return default_value;
  } else {
    return _sdf->GetElement(element)->Get<bool>();
  }
}

double GazeboConeGroundTruth::getDoubleParameter(sdf::ElementPtr _sdf, const char *element,
                                                 double default_value,
                                                 const char *default_description) {
  if (!_sdf->HasElement(element)) {
    RCLCPP_DEBUG(this->rosnode_->get_logger(),
                 "state_ground_truth plugin missing <%s>, defaults to %s", element,
                 default_description);
    return default_value;
  } else {
    return _sdf->GetElement(element)->Get<double>();
  }
}

std::string GazeboConeGroundTruth::getStringParameter(sdf::ElementPtr _sdf, const char *element,
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

ignition::math::Vector3d GazeboConeGroundTruth::getVector3dParameter(
    sdf::ElementPtr _sdf, const char *element, ignition::math::Vector3d default_value,
    const char *default_description) {
  if (!_sdf->HasElement(element)) {
    RCLCPP_DEBUG(this->rosnode_->get_logger(),
                 "state_ground_truth plugin missing <%s>, defaults to %s", element,
                 default_description);
    return default_value;
  } else {
    return _sdf->GetElement(element)->Get<ignition::math::Vector3d>();
  }
}

eufs_msgs::msg::ConeArray GazeboConeGroundTruth::stripCovariance(
    eufs_msgs::msg::ConeArrayWithCovariance msg) {
  auto return_msg = eufs_msgs::msg::ConeArray();
  for (auto c : msg.blue_cones) {
    return_msg.blue_cones.push_back(c.point);
  }
  for (auto c : msg.yellow_cones) {
    return_msg.yellow_cones.push_back(c.point);
  }
  for (auto c : msg.orange_cones) {
    return_msg.orange_cones.push_back(c.point);
  }
  for (auto c : msg.big_orange_cones) {
    return_msg.big_orange_cones.push_back(c.point);
  }
  for (auto c : msg.unknown_color_cones) {
    return_msg.unknown_color_cones.push_back(c.point);
  }
  return return_msg;
}
}  // namespace eufs_plugins
}  // namespace gazebo_plugins
