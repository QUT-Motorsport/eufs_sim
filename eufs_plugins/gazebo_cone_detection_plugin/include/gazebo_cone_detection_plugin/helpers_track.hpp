#pragma once

#include <driverless_msgs/msg/cone.hpp>
#include <driverless_msgs/msg/cone_detection_stamped.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Model.hh>
#include <string>

#include "rclcpp/rclcpp.hpp"

int get_cone_colour(gazebo::physics::LinkPtr cone_link, std::optional<const rclcpp::Logger> logger = {}) {
    std::string link_name = cone_link->GetName();

    if (link_name.substr(0, 9) == "blue_cone") {
        return driverless_msgs::msg::Cone::BLUE;
    }
    if (link_name.substr(0, 11) == "yellow_cone") {
        return driverless_msgs::msg::Cone::YELLOW;
    }
    if (link_name.substr(0, 11) == "orange_cone") {
        return driverless_msgs::msg::Cone::ORANGE_SMALL;
    }
    if (link_name.substr(0, 8) == "big_cone") {
        return driverless_msgs::msg::Cone::ORANGE_BIG;
    }

    if (logger) {
        RCLCPP_WARN_ONCE(*logger, "Cannot get cone type from link in the track model in simulation: %s",
                         link_name.c_str());
    }

    return driverless_msgs::msg::Cone::UNKNOWN;
}

driverless_msgs::msg::ConeWithCovariance get_cone_from_link(gazebo::physics::LinkPtr cone_link,
                                                            std::array<double, 4UL> covariance,
                                                            std::optional<const rclcpp::Logger> logger = {}) {
    driverless_msgs::msg::ConeWithCovariance cone;
    cone.cone.location.x = cone_link->WorldPose().Pos().X();
    cone.cone.location.y = cone_link->WorldPose().Pos().Y();
    cone.cone.location.z = 0;
    cone.cone.color = get_cone_colour(cone_link, logger);
    cone.covariance = covariance;

    return cone;
}

driverless_msgs::msg::ConeDetectionStamped get_ground_truth_track(gazebo::physics::ModelPtr track_model,
                                                                  gazebo::common::Time now, std::string track_frame_id,
                                                                  std::optional<const rclcpp::Logger> logger = {}) {
    driverless_msgs::msg::ConeDetectionStamped track;

    track.header.frame_id = track_frame_id;
    track.header.stamp.sec = now.sec;
    track.header.stamp.nanosec = now.nsec;

    gazebo::physics::Link_V links = track_model->GetLinks();
    for (auto const &link : links) {
        track.cones_with_cov.push_back(get_cone_from_link(link, {0, 0, 0, 0}, logger));
    }

    return track;
}
