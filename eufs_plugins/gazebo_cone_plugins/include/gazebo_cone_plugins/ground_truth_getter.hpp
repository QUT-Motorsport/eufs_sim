#ifndef EUFS_PLUGINS_GAZEBO_CONE_PLUGINS_INCLUDE_GAZEBO_CONE_PLUGINS_GROUND_TRUTH_GETTER_HPP_
#define EUFS_PLUGINS_GAZEBO_CONE_PLUGINS_INCLUDE_GAZEBO_CONE_PLUGINS_GROUND_TRUTH_GETTER_HPP_

#include <vector>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/logger.hpp"
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/common/Time.hh>
#include <geometry_msgs/msg/point.hpp>

#include <eufs_msgs/msg/cone_with_covariance.hpp>
#include <eufs_msgs/msg/cone_array_with_covariance.hpp>


namespace internal {

enum ConeType {blue, yellow, orange, big_orange};

ConeType getConeType(gazebo::physics::LinkPtr link, rclcpp::Logger logger) {
  std::string link_name = link->GetName();

  if (link_name.substr(0, 9) == "blue_cone") {
    return ConeType::blue;
  } else if (link_name.substr(0, 11) == "yellow_cone") {
    return ConeType::yellow;
  } else if (link_name.substr(0, 11) == "orange_cone") {
    return ConeType::orange;
  } else if (link_name.substr(0, 8) == "big_cone") {
    return ConeType::big_orange;
  } else {
    RCLCPP_WARN_ONCE(logger,
                     "Cannot get cone type from link in the track model: %s",
                     link_name.c_str());
    return ConeType::orange;
  }
}

void addConeToConeArray(eufs_msgs::msg::ConeArrayWithCovariance &cone_array,
                        gazebo::physics::LinkPtr link,
                        rclcpp::Logger logger) {
  geometry_msgs::msg::Point point;
  point.x = link->WorldPose().Pos().X();
  point.y = link->WorldPose().Pos().Y();
  point.z = 0;

  ConeType cone_type = getConeType(link, logger);

  eufs_msgs::msg::ConeWithCovariance cone = eufs_msgs::msg::ConeWithCovariance();
  cone.point = point;
  cone.covariance = {0, 0, 0, 0};  // ground truth so zero covariance

  switch (cone_type) {
    case ConeType::blue:
      cone_array.blue_cones.push_back(cone);
      break;
    case ConeType::yellow:
      cone_array.yellow_cones.push_back(cone);
      break;
    case ConeType::orange:
      cone_array.orange_cones.push_back(cone);
      break;
    case ConeType::big_orange:
      cone_array.big_orange_cones.push_back(cone);
      break;
  }
}

std::vector<eufs_msgs::msg::ConeWithCovariance> translateCones(
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

}  // namespace internal


namespace gazebo_plugins {
namespace eufs_plugins {
namespace cone_helpers {

// IMPORTANT: This doesn't timestamp the ConeArray message, this is the responsibility of the plugin
eufs_msgs::msg::ConeArrayWithCovariance getGroundTruthCones(gazebo::physics::ModelPtr track_model,
                                                            rclcpp::Logger logger) {
  eufs_msgs::msg::ConeArrayWithCovariance cone_arrays_message;
  cone_arrays_message.header.frame_id = "gazebo";  // Gets cone coordinates in gazebo global FoV

  if (track_model != nullptr) {
    gazebo::physics::Link_V links = track_model->GetLinks();
    for (unsigned int i = 0; i < links.size(); i++) {
      internal::addConeToConeArray(cone_arrays_message, links[i], logger);
    }
  }
  return cone_arrays_message;
}

eufs_msgs::msg::ConeArrayWithCovariance translateToFrame(
    eufs_msgs::msg::ConeArrayWithCovariance cones,
    ignition::math::Pose3d frame_origin, std::string frame_id) {
  cones.header.frame_id = frame_id;
  cones.blue_cones = internal::translateCones(cones.blue_cones, frame_origin);
  cones.yellow_cones = internal::translateCones(cones.yellow_cones, frame_origin);
  cones.orange_cones = internal::translateCones(cones.orange_cones, frame_origin);
  cones.big_orange_cones = internal::translateCones(cones.big_orange_cones, frame_origin);
  cones.unknown_color_cones = internal::translateCones(cones.unknown_color_cones, frame_origin);
  return cones;
}

}  // namespace cone_helpers
}  // namespace eufs_plugins
}  // namespace gazebo_plugins

#endif  // EUFS_PLUGINS_GAZEBO_CONE_PLUGINS_INCLUDE_GAZEBO_CONE_PLUGINS_GROUND_TRUTH_GETTER_HPP_
