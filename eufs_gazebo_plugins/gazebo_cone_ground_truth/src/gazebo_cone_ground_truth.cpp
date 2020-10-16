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
 * @details Provides ground truth cones in simulation in the form of `eufs_msgs/msg/ConeArrayWithCovariance`.
 * Can also simulate the perception stack by publishing cones with noise.
 **/

#include "../include/eufs_gazebo_plugins/gazebo_cone_ground_truth.h"

namespace gazebo_plugins {
namespace eufs {

  GZ_REGISTER_MODEL_PLUGIN(GazeboConeGroundTruth)

  GazeboConeGroundTruth::GazeboConeGroundTruth() {
    this->seed = 0;
  }

  // Gazebo plugin functions

  void GazeboConeGroundTruth::Load(gazebo::physics::ModelPtr _parent, sdf::ElementPtr _sdf) {
    this->rosnode_ = gazebo_ros::Node::Get(_sdf);

    this->time_last_published = rclcpp::Time(0);

#if GAZEBO_MAJOR_VERSION >= 8
    this->track_model = _parent->GetWorld()->ModelByName("track");
#else
    this->track_model = _parent->GetWorld()->GetModel("track");
#endif
    this->car_link = _parent->GetLink("base_footprint");

    this->update_rate_ = getDoubleParameter(_sdf, "updateRate", 0, "0.0 (as fast as possible)");

    this->lidar_total_view_distance = getDoubleParameter(_sdf, "lidarViewDistance", 100, "100");
    this->camera_total_view_distance = getDoubleParameter(_sdf, "cameraViewDistance", 10, "10");
    this->lidar_min_view_distance = getDoubleParameter(_sdf, "lidarMinViewDistance", 1, "1");
    this->camera_min_view_distance = getDoubleParameter(_sdf, "cameraMinViewDistance", 1, "1");
    this->lidar_x_view_distance = getDoubleParameter(_sdf, "lidarXViewDistance", 20, "20");
    this->lidar_y_view_distance = getDoubleParameter(_sdf, "lidarYViewDistance", 10, "10");
    this->lidar_fov = getDoubleParameter(_sdf, "lidarFOV", 6.283185, "6.283185  (360 degrees)");
    this->camera_fov = getDoubleParameter(_sdf, "cameraFOV", 1.918889, "1.918889  (110 degrees)");
    this->camera_a = getDoubleParameter(_sdf, "perceptionCameraDepthNoiseParameterA", 0.0184, "0.0184");
    this->camera_b = getDoubleParameter(_sdf, "perceptionCameraDepthNoiseParameterB", 0.2106, "0.2106");
    this->lidar_on = getBoolParameter(_sdf, "lidarOn", true, "true");

    this->cone_frame_ = getStringParameter(_sdf, "coneFrame", "base_footprint", "base_footprint");

    this->simulate_perception_ = getBoolParameter(_sdf, "simulatePerception", false, "false");

    this->perception_lidar_noise_ = getVector3dParameter(_sdf, "perceptionNoise", {0.03, 0.03, 0.0}, "0.03, 0.03, 0.0");

    // Setup the publishers

    // Ground truth cone publisher
    if (!_sdf->HasElement("groundTruthConesTopicName")) {
      RCLCPP_FATAL(this->rosnode_->get_logger(), "state_ground_truth plugin missing <groundTruthConesTopicName>, cannot proceed");
      return;
    } else {
      std::string topic_name_ = _sdf->GetElement("groundTruthConesTopicName")->Get<std::string>();
      this->ground_truth_cone_pub_ = this->rosnode_->create_publisher<eufs_msgs::msg::ConeArrayWithCovariance>(topic_name_, 1);
    }

    // Ground truth cone marker publisher
    if (!_sdf->HasElement("groundTruthConeMarkersTopicName")) {
      RCLCPP_FATAL(this->rosnode_->get_logger(), "state_ground_truth plugin missing <groundTruthConeMarkersTopicName>, cannot proceed");
      return;
    } else {
      std::string topic_name_ = _sdf->GetElement("groundTruthConeMarkersTopicName")->Get<std::string>();
      this->ground_truth_cone_marker_pub_ = this->rosnode_->create_publisher<visualization_msgs::msg::MarkerArray>(topic_name_, 1);
    }

    // Ground truth track publisher
    if (!_sdf->HasElement("groundTruthTrackTopicName")) {
      RCLCPP_FATAL(this->rosnode_->get_logger(), "state_ground_truth plugin missing <groundTruthTrackTopicName>, cannot proceed");
      return;
    } else {
      std::string topic_name_ = _sdf->GetElement("groundTruthTrackTopicName")->Get<std::string>();
      this->ground_truth_track_pub_ = this->rosnode_->create_publisher<eufs_msgs::msg::ConeArrayWithCovariance>(topic_name_, 1);
    }

    if (this->simulate_perception_) {
      // Camera cone publisher
      if (!_sdf->HasElement("perceptionConesTopicName")) {
        RCLCPP_FATAL(this->rosnode_->get_logger(), "state_ground_truth plugin missing <perceptionConesTopicName>, cannot proceed");
        return;
      } else {
        std::string topic_name_ = _sdf->GetElement("perceptionConesTopicName")->Get<std::string>();
        this->perception_cone_pub_ = this->rosnode_->create_publisher<eufs_msgs::msg::ConeArrayWithCovariance>(topic_name_, 1);
      }

      // Camera cone marker publisher
      if (!_sdf->HasElement("perceptionConeMarkersTopicName")) {
        RCLCPP_FATAL(this->rosnode_->get_logger(), "state_ground_truth plugin missing <perceptionConeMarkersTopicName>, cannot proceed");
        return;
      } else {
        std::string topic_name_ = _sdf->GetElement("perceptionConeMarkersTopicName")->Get<std::string>();
        this->perception_cone_marker_pub_ = this->rosnode_->create_publisher<visualization_msgs::msg::MarkerArray>(topic_name_, 1);
      }
    }

    std::string eufs_description_directory = ament_index_cpp::get_package_share_directory("eufs_description");
    cone_big_mesh_path = "file:///" + eufs_description_directory + "/meshes/cone_big.dae";
    cone_mesh_path = "file:///" + eufs_description_directory + "/meshes/cone.dae";

    this->update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(std::bind(&GazeboConeGroundTruth::UpdateChild, this));
  }  // GazeboConeGroundTruth

  void GazeboConeGroundTruth::UpdateChild() {
    // Check if it is time to pubilsh new data
    if (this->rosnode_->now().seconds() - time_last_published.seconds() < (1.0 / this->update_rate_)) {
      return;
    }

    // Update the time (This is here so that the time to publish all the messages does not affect the update rate)
    this->time_last_published = this->rosnode_->now();

    // Check if there is a reason to publish the data
    if (this->ground_truth_cone_pub_->get_subscription_count() == 0 && this->ground_truth_cone_marker_pub_->get_subscription_count() == 0
        && this->perception_cone_pub_->get_subscription_count() == 0 && this->perception_cone_marker_pub_->get_subscription_count() == 0) {
      RCLCPP_DEBUG(this->rosnode_->get_logger(), "Nobody is listening to cone_ground_truth. Doing nothing");
      return;
    }

#if GAZEBO_MAJOR_VERSION >= 8
    this->car_pos = this->car_link->WorldPose();
#else
    this->car_pos = this->car_link->GetWorldPose().Ign();
#endif

    // Get the track message
    eufs_msgs::msg::ConeArrayWithCovariance ground_truth_track_message = getTrackMessage();

    // Publish the ground truth track if it has subscribers
    if (this->ground_truth_track_pub_->get_subscription_count() > 0) {
      this->ground_truth_track_pub_->publish(ground_truth_track_message);
    }

    eufs_msgs::msg::ConeArrayWithCovariance ground_truth_cone_array_message = getConeArrayMessage(ground_truth_track_message);

    // Publish the ground truth cones if it has subscribers
    if (this->ground_truth_cone_pub_->get_subscription_count() > 0) {
      this->ground_truth_cone_pub_->publish(ground_truth_cone_array_message);
    }

    // Publish the ground truth cone markers if it has subscribers
    if (this->ground_truth_cone_marker_pub_->get_subscription_count() > 0) {
      visualization_msgs::msg::MarkerArray ground_truth_cone_marker_array_message = getConeMarkerArrayMessage(ground_truth_cone_array_message);
      this->ground_truth_cone_marker_pub_->publish(ground_truth_cone_marker_array_message);
    }

    // Publish the simulated perception cones if it has subscribers
    if (this->simulate_perception_ && (this->perception_cone_pub_->get_subscription_count() > 0 || this->perception_cone_marker_pub_->get_subscription_count() > 0)) {
      eufs_msgs::msg::ConeArrayWithCovariance perception_cone_array_message = getConeArrayMessageWithNoise(ground_truth_cone_array_message, perception_lidar_noise_);
      visualization_msgs::msg::MarkerArray perception_cone_marker_array_message = getConeMarkerArrayMessage(perception_cone_array_message);

      this->perception_cone_pub_->publish(perception_cone_array_message);
      this->perception_cone_marker_pub_->publish(perception_cone_marker_array_message);
    }
  }

  // Getting the track
  eufs_msgs::msg::ConeArrayWithCovariance GazeboConeGroundTruth::getTrackMessage() {
    eufs_msgs::msg::ConeArrayWithCovariance ground_truth_track_message;

    if (this->track_model != nullptr) {
      gazebo::physics::Link_V links = this->track_model->GetLinks();
      for (unsigned int i = 0; i < links.size(); i++) {
        addConeToConeArray(ground_truth_track_message, links[i]);
      }
    }

    return ground_truth_track_message;
  }

  // Getting the cone arrays
  eufs_msgs::msg::ConeArrayWithCovariance GazeboConeGroundTruth::getConeArrayMessage(eufs_msgs::msg::ConeArrayWithCovariance &track) {
    eufs_msgs::msg::ConeArrayWithCovariance ground_truth_cone_array_message = track;

    processCones(ground_truth_cone_array_message);

    ground_truth_cone_array_message.header.frame_id = "/" + this->cone_frame_;
    ground_truth_cone_array_message.header.stamp = this->rosnode_->now();

    return ground_truth_cone_array_message;
  }

  void GazeboConeGroundTruth::addConeToConeArray(eufs_msgs::msg::ConeArrayWithCovariance &ground_truth_cone_array, gazebo::physics::LinkPtr link) {
    geometry_msgs::msg::Point point;

#if GAZEBO_MAJOR_VERSION >= 8
    point.x = link->WorldPose().Pos().X();
    point.y = link->WorldPose().Pos().Y();
#else
    point.x = link->GetWorldPose().Ign().Pos().X();
    point.y = link->GetWorldPose().Ign().Pos().Y();
#endif
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

  void GazeboConeGroundTruth::processCones(eufs_msgs::msg::ConeArrayWithCovariance &cones) {
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
  }

  bool GazeboConeGroundTruth::inRangeOfCamera(eufs_msgs::msg::ConeWithCovariance cone)
  {
    auto dist = (cone.point.x * cone.point.x) + (cone.point.y * cone.point.y);
    return camera_min_view_distance * camera_min_view_distance < dist &&
           dist < camera_total_view_distance * camera_total_view_distance;
  }

  bool GazeboConeGroundTruth::inFOVOfCamera(eufs_msgs::msg::ConeWithCovariance cone)
  {
    float angle = atan2(cone.point.y, cone.point.x);
    return abs(angle) < (this->camera_fov / 2);
  }

  bool GazeboConeGroundTruth::inRangeOfLidar(eufs_msgs::msg::ConeWithCovariance cone)
  {
    auto dist = (cone.point.x * cone.point.x) + (cone.point.y * cone.point.y);
    return lidar_min_view_distance * lidar_min_view_distance < dist &&
           dist < lidar_total_view_distance * lidar_total_view_distance &&
           abs(cone.point.x) < lidar_x_view_distance &&
           abs(cone.point.y) < lidar_y_view_distance &&
           this->lidar_on;
  }

  bool GazeboConeGroundTruth::inFOVOfLidar(eufs_msgs::msg::ConeWithCovariance cone)
  {
    float angle = atan2(cone.point.y, cone.point.x);
    return abs(angle) < (this->lidar_fov / 2) && this->lidar_on;
  }

  std::pair<std::vector<eufs_msgs::msg::ConeWithCovariance>, std::vector<eufs_msgs::msg::ConeWithCovariance>>
      GazeboConeGroundTruth::fovCones(std::vector<eufs_msgs::msg::ConeWithCovariance> conesToCheck)
  {
    std::vector<eufs_msgs::msg::ConeWithCovariance> cones_in_view;
    std::vector<eufs_msgs::msg::ConeWithCovariance> cones_in_view_without_color;
    for (unsigned int i = 0; i < conesToCheck.size(); i++) {
      // Translate the position of the cone to be based on the car
      float x = conesToCheck[i].point.x - this->car_pos.Pos().X();
      float y = conesToCheck[i].point.y - this->car_pos.Pos().Y();

      // If the cone is withing viewing distance of lidar
      float yaw = this->car_pos.Rot().Yaw();

      // Rotate the points using the yaw of the car (x and y are the other way around)
      conesToCheck[i].point.y = (cos(yaw) * y) - (sin(yaw) * x);
      conesToCheck[i].point.x = (sin(yaw) * y) + (cos(yaw) * x);

      bool lidar_sees = inRangeOfLidar(conesToCheck[i]) && inFOVOfLidar(conesToCheck[i]);
      bool camera_sees = inRangeOfCamera(conesToCheck[i]) && inFOVOfCamera(conesToCheck[i]);

      if ((lidar_sees && !camera_sees)) {
        cones_in_view_without_color.push_back(conesToCheck[i]);
      }
      else if (camera_sees) {
        cones_in_view.push_back(conesToCheck[i]);
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

    RCLCPP_WARN_ONCE(this->rosnode_->get_logger(), "Cannot get cone type from link in the track model in simulation: %s", link_name.c_str());
    return ConeType::unknown;
  }

  // Getting the cone marker array

  visualization_msgs::msg::MarkerArray GazeboConeGroundTruth::getConeMarkerArrayMessage(eufs_msgs::msg::ConeArrayWithCovariance &ground_truth_cone_array_message) {
    visualization_msgs::msg::MarkerArray ground_truth_cone_marker_array;

    int marker_id = 0;
    marker_id = addConeMarkers(ground_truth_cone_marker_array.markers, marker_id, ground_truth_cone_array_message.blue_cones, 0.2, 0.2, 1, false);
    marker_id = addConeMarkers(ground_truth_cone_marker_array.markers, marker_id, ground_truth_cone_array_message.yellow_cones, 1, 1, 0, false);
    marker_id = addConeMarkers(ground_truth_cone_marker_array.markers, marker_id, ground_truth_cone_array_message.orange_cones, 1, 0.549, 0, false);
    marker_id = addConeMarkers(ground_truth_cone_marker_array.markers, marker_id, ground_truth_cone_array_message.big_orange_cones, 1, 0.271, 0, true);
    marker_id = addConeMarkers(ground_truth_cone_marker_array.markers, marker_id, ground_truth_cone_array_message.unknown_color_cones, 0.7, 0.7, 0.7, false);

    return ground_truth_cone_marker_array;
  }

  int GazeboConeGroundTruth::addConeMarkers(std::vector<visualization_msgs::msg::Marker> &marker_array, int marker_id, std::vector<eufs_msgs::msg::ConeWithCovariance> cones, float red, float green, float blue, bool big) {
    int id = marker_id;
    for (unsigned int i = 0; i < cones.size(); i++) {
      visualization_msgs::msg::Marker marker;

      marker.header.stamp = this->rosnode_->now();
      marker.header.frame_id = "/base_footprint";

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

      marker.lifetime = rclcpp::Duration(0.2);

      marker_array.push_back(marker);

      id++;
    }
    return id;
  }


  // Add noise to the cone arrays
  eufs_msgs::msg::ConeArrayWithCovariance GazeboConeGroundTruth::getConeArrayMessageWithNoise(eufs_msgs::msg::ConeArrayWithCovariance &ground_truth_cone_array_message, ignition::math::Vector3d noise) {
    eufs_msgs::msg::ConeArrayWithCovariance cone_array_message_with_noise = ground_truth_cone_array_message;

    addNoiseToConeArray(cone_array_message_with_noise.blue_cones, noise);
    addNoiseToConeArray(cone_array_message_with_noise.yellow_cones, noise);
    addNoiseToConeArray(cone_array_message_with_noise.orange_cones, noise);
    addNoiseToConeArray(cone_array_message_with_noise.big_orange_cones, noise);
    addNoiseToConeArray(cone_array_message_with_noise.unknown_color_cones, noise);

    cone_array_message_with_noise.header.frame_id = "/" + this->cone_frame_;
    cone_array_message_with_noise.header.stamp = this->rosnode_->now();

    return cone_array_message_with_noise;
  }

  void GazeboConeGroundTruth::addNoiseToConeArray(std::vector<eufs_msgs::msg::ConeWithCovariance> &cone_array, ignition::math::Vector3d noise) {
    for (unsigned int i = 0; i < cone_array.size(); i++) {
      // By default we use just lidar noise
      auto x_noise = noise.X();
      auto y_noise = noise.Y();

      // But if only the camera sees it, we use camera noise specifically
      if (!inFOVOfLidar(cone_array[i]) || !inRangeOfLidar(cone_array[i]))
      {
        auto dist = sqrt(
            cone_array[i].point.x * cone_array[i].point.x +
            cone_array[i].point.y * cone_array[i].point.y
        );
        x_noise = camera_a * std::exp(camera_b * dist);
        y_noise = x_noise / 5;
      }

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
    double U = static_cast<double>(rand_r(&this->seed)) /
               static_cast<double>(RAND_MAX);

    // normalized uniform random variable
    double V = static_cast<double>(rand_r(&this->seed)) /
               static_cast<double>(RAND_MAX);

    double X = sqrt(-2.0 * ::log(U)) * cos(2.0 * M_PI * V);
    // double Y = sqrt(-2.0 * ::log(U)) * sin(2.0*M_PI * V);

    // there are 2 indep. vars, we'll just use X
    // scale to our mu and sigma
    X = sigma * X + mu;
    return X;
  }

  // Helper function for parameters
  bool GazeboConeGroundTruth::getBoolParameter(sdf::ElementPtr _sdf, const char* element, bool default_value, const char* default_description) {
    if (!_sdf->HasElement(element)) {
      RCLCPP_DEBUG(this->rosnode_->get_logger(), "state_ground_truth plugin missing <%s>, defaults to %s", element, default_description);
      return default_value;
    } else {
      return  _sdf->GetElement(element)->Get<bool>();
    }
  }

  double GazeboConeGroundTruth::getDoubleParameter(sdf::ElementPtr _sdf, const char* element, double default_value, const char* default_description) {
    if (!_sdf->HasElement(element)) {
      RCLCPP_DEBUG(this->rosnode_->get_logger(), "state_ground_truth plugin missing <%s>, defaults to %s", element, default_description);
      return default_value;
    } else {
      return  _sdf->GetElement(element)->Get<double>();
    }
  }

  std::string GazeboConeGroundTruth::getStringParameter(sdf::ElementPtr _sdf, const char* element, std::string default_value, const char* default_description) {
    if (!_sdf->HasElement(element)) {
      RCLCPP_DEBUG(this->rosnode_->get_logger(), "state_ground_truth plugin missing <%s>, defaults to %s", element, default_description);
      return default_value;
    } else {
      return  _sdf->GetElement(element)->Get<std::string>();
    }
  }

  ignition::math::Vector3d GazeboConeGroundTruth::getVector3dParameter(sdf::ElementPtr _sdf, const char* element, ignition::math::Vector3d default_value, const char* default_description) {
    if (!_sdf->HasElement(element)) {
      RCLCPP_DEBUG(this->rosnode_->get_logger(), "state_ground_truth plugin missing <%s>, defaults to %s", element, default_description);
      return default_value;
    } else {
      return  _sdf->GetElement(element)->Get<ignition::math::Vector3d>();
    }
  }

  eufs_msgs::msg::ConeArray GazeboConeGroundTruth::stripCovariance(eufs_msgs::msg::ConeArrayWithCovariance msg)
  {
    auto return_msg = eufs_msgs::msg::ConeArray();
    for (auto c : msg.blue_cones)
    {
      return_msg.blue_cones.push_back(c.point);
    }
    for (auto c : msg.yellow_cones)
    {
      return_msg.yellow_cones.push_back(c.point);
    }
    for (auto c : msg.orange_cones)
    {
      return_msg.orange_cones.push_back(c.point);
    }
    for (auto c : msg.big_orange_cones)
    {
      return_msg.big_orange_cones.push_back(c.point);
    }
    for (auto c : msg.unknown_color_cones)
    {
      return_msg.unknown_color_cones.push_back(c.point);
    }
    return return_msg;
  }

} // namespace eufs
} // namespace gazebo_plugins