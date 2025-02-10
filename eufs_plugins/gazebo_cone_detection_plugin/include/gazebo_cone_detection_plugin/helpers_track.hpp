#pragma once

#include <driverless_msgs/msg/cone.hpp>
#include <driverless_msgs/msg/cone_detection_stamped.hpp>
// #include <gazebo/gazebo.hh>
// #include <gazebo/physics/Link.hh>
// #include <gazebo/physics/Model.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Entity.hh> 
#include <gz/sim/Util.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>
#include <rclcpp/rclcpp.hpp>
#include <string>   
#include <vector>

#include "rclcpp/rclcpp.hpp"

int get_cone_colour(const std::string &name, std::optional<const rclcpp::Logger> logger = {}) {
    if (name.find("blue_cone") != std::string::npos) {
        return driverless_msgs::msg::Cone::BLUE;
    }
    if (name.find("yellow_cone") != std::string::npos) {
        return driverless_msgs::msg::Cone::YELLOW;
    }
    if (name.find("orange_cone") != std::string::npos) {
        return driverless_msgs::msg::Cone::ORANGE_SMALL;
    }
    if (name.find("big_cone") != std::string::npos) {
        return driverless_msgs::msg::Cone::ORANGE_BIG;
    }

    if (logger) {
        RCLCPP_WARN_ONCE(*logger, "Unknown cone type: %s", name.c_str());
    }

    return driverless_msgs::msg::Cone::UNKNOWN;
}

driverless_msgs::msg::ConeWithCovariance get_cone_from_entity(const gz::sim::Entity &cone_entity,
                                                              gz::sim::EntityComponentManager &ecm,
                                                              const std::string &name,
                                                              const std::array<double, 4> &covariance,
                                                              std::optional<const rclcpp::Logger> logger = {}) {
    driverless_msgs::msg::ConeWithCovariance cone; 
    const auto *poseComp = ecm.Component<gz::sim::components::Pose>(cone_entity);
    if (!poseComp) {
        if (logger) {
            RCLCPP_ERROR(*logger, "No pose component found for cone: %lu", static_cast<unsigned long>(cone_entity));

        }
        return driverless_msgs::msg::ConeWithCovariance();
    }

    gz::math::Pose3d pose = poseComp->Data();
    cone.cone.location.x = pose.Pos().X();
    cone.cone.location.y = pose.Pos().Y();
    cone.cone.location.z = 0;
    cone.cone.color = get_cone_colour(name, logger);
    cone.covariance = covariance;

    return cone;
}

driverless_msgs::msg::ConeDetectionStamped get_ground_truth_track(gz::sim::Model track_model,
                                                                  gz::sim::EntityComponentManager &ecm,
                                                                  const std::string &track_frame_id,
                                                                  std::optional<const rclcpp::Logger> logger = {}) {
    driverless_msgs::msg::ConeDetectionStamped track;
    track.header.frame_id = track_frame_id;
    track.header.stamp = rclcpp::Clock().now();

    for (auto child : ecm.Descendants(track_model.Entity()))
    {
        if (!ecm.Component<gz::sim::components::Link>(child)) continue;
        std::string name = track_model.Name(ecm);
        track.cones_with_cov.push_back(get_cone_from_entity(child, ecm, name, {0, 0, 0, 0}, logger));
    }

    return track;
}
