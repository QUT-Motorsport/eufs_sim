#pragma once

#include <gazebo/gazebo.hh>

#include <driverless_msgs/msg/cone.hpp>
#include <driverless_msgs/msg/cone_detection_stamped.hpp>


driverless_msgs::msg::ConeDetectionStamped get_lidar_detection(
    ignition::math::Pose3d car_pose,
    driverless_msgs::msg::ConeDetectionStamped ground_truth_track
) {
    // TODO
    return ground_truth_track;
}


driverless_msgs::msg::ConeDetectionStamped get_vision_detection(
    ignition::math::Pose3d car_pose,
    driverless_msgs::msg::ConeDetectionStamped ground_truth_track
) {
    // TODO
    return ground_truth_track;
}


driverless_msgs::msg::ConeDetectionStamped get_slam_global_map(
    ignition::math::Pose3d car_pose,
    driverless_msgs::msg::ConeDetectionStamped ground_truth_track
) {
    // TODO
    return ground_truth_track;
}


driverless_msgs::msg::ConeDetectionStamped get_slam_local_map(
    ignition::math::Pose3d car_pose,
    driverless_msgs::msg::ConeDetectionStamped ground_truth_track
) {
    // TODO
    return ground_truth_track;
}
