#pragma once

#include <driverless_msgs/msg/cone.hpp>
#include <driverless_msgs/msg/cone_detection_stamped.hpp>
#include <gazebo/gazebo.hh>
#include <string>

#include "helpers_gazebo.hpp"

unsigned int seed = 0;

typedef struct SensorConfig {
    std::string frame_id;
    double min_view_distance;
    double max_view_distance;
    double fov;
    double range_noise;
    double bearing_noise;
    bool detects_colour;
    double offset_x;
} SensorConfig_t;

SensorConfig_t populate_sensor_config(std::string sensor_prefix, gazebo_ros::Node::SharedPtr node) {
    std::string frame_id = node->declare_parameter(sensor_prefix + "_frame_id", "sensor");
    double min_view_distance = node->declare_parameter(sensor_prefix + "_min_view_distance", 0.0);
    double max_view_distance = node->declare_parameter(sensor_prefix + "_max_view_distance", 0.0);
    double fov = node->declare_parameter(sensor_prefix + "_fov", 0.0);
    double range_noise = node->declare_parameter(sensor_prefix + "_range_noise", 0.0);
    double bearing_noise = node->declare_parameter(sensor_prefix + "_bearing_noise", 0.0);
    bool detects_colour = node->declare_parameter(sensor_prefix + "_detects_colour", true);
    double offset_x = node->declare_parameter(sensor_prefix + "_offset_x", 0.0);

    return {frame_id, min_view_distance, max_view_distance, fov, range_noise, bearing_noise, detects_colour, offset_x};
}

driverless_msgs::msg::Cone convert_cone_to_car_frame(const ignition::math::Pose3d car_pose,
                                                     const driverless_msgs::msg::Cone cone, const double offset_x = 0) {
    driverless_msgs::msg::Cone translated_cone = cone;

    double x = cone.location.x - car_pose.Pos().X();
    double y = cone.location.y - car_pose.Pos().Y();
    double yaw = car_pose.Rot().Yaw();

    // Rotate the points using the yaw of the car (x and y are the other way around)
    translated_cone.location.y = (cos(yaw) * y) - (sin(yaw) * x);
    translated_cone.location.x = (sin(yaw) * y) + (cos(yaw) * x) - offset_x;

    return translated_cone;
}

double cone_dist(driverless_msgs::msg::Cone cone) {
    return sqrt(cone.location.x * cone.location.x + cone.location.y * cone.location.y);
}

double cone_angle(driverless_msgs::msg::Cone cone) { return atan2(cone.location.y, cone.location.x); }

double GaussianKernel(double mu, double sigma) {
    // using Box-Muller transform to generate two independent standard
    // normally distributed normal variables see wikipedia

    // normalized uniform random variable
    double U = static_cast<double>(rand_r(&seed)) / static_cast<double>(RAND_MAX);

    // normalized uniform random variable
    double V = static_cast<double>(rand_r(&seed)) / static_cast<double>(RAND_MAX);

    double X = sqrt(-2.0 * ::log(U)) * cos(2.0 * M_PI * V);
    // double Y = sqrt(-2.0 * ::log(U)) * sin(2.0*M_PI * V);

    // there are 2 indep. vars, we'll just use X
    // scale to our mu and sigma
    X = sigma * X + mu;
    return X;
}

driverless_msgs::msg::ConeWithCovariance make_noisy_range_bearing_cone(driverless_msgs::msg::Cone cone,
                                                                       double range_noise, double bearing_noise) {
    driverless_msgs::msg::ConeWithCovariance noisy_cone;

    double range = cone_dist(cone) + GaussianKernel(0, range_noise);
    double bearing = cone_angle(cone) + GaussianKernel(0, bearing_noise);

    noisy_cone.cone = cone;
    noisy_cone.cone.location.x = range * cos(bearing);
    noisy_cone.cone.location.y = range * sin(bearing);

    // TODO - how to convert this back to x, y noise?
    noisy_cone.covariance = {range_noise * range_noise, 0, 0, bearing_noise * bearing_noise};

    return noisy_cone;
}

driverless_msgs::msg::ConeWithCovariance make_noisy_x_y_cone(driverless_msgs::msg::Cone cone, double x_noise,
                                                             double y_noise) {
    driverless_msgs::msg::ConeWithCovariance noisy_cone;
    noisy_cone.cone = cone;
    noisy_cone.cone.location.x += GaussianKernel(0, x_noise);
    noisy_cone.cone.location.y += GaussianKernel(0, y_noise);
    noisy_cone.covariance = {x_noise * x_noise, 0, 0, y_noise * y_noise};

    return noisy_cone;
}

driverless_msgs::msg::ConeDetectionStamped get_sensor_detection(
    SensorConfig_t sensor_config, ignition::math::Pose3d car_pose,
    driverless_msgs::msg::ConeDetectionStamped ground_truth_track) {
    driverless_msgs::msg::ConeDetectionStamped sensor_detection;
    sensor_detection.header = ground_truth_track.header;
    sensor_detection.header.frame_id = sensor_config.frame_id;

    for (auto const &cone : ground_truth_track.cones_with_cov) {
        auto translated_cone = convert_cone_to_car_frame(car_pose, cone.cone, sensor_config.offset_x);

        double dist = cone_dist(translated_cone);
        if (dist < sensor_config.min_view_distance || dist > sensor_config.max_view_distance) {
            continue;
        }

        double angle = cone_angle(translated_cone);
        if (abs(angle) > sensor_config.fov / 2) {
            continue;
        }

        if (!sensor_config.detects_colour) {
            translated_cone.color = driverless_msgs::msg::Cone::UNKNOWN;
        }

        driverless_msgs::msg::ConeWithCovariance noisy_cone =
            make_noisy_range_bearing_cone(translated_cone, sensor_config.range_noise, sensor_config.bearing_noise);
        sensor_detection.cones_with_cov.push_back(noisy_cone);
        sensor_detection.cones.push_back(noisy_cone.cone);
    }

    return sensor_detection;
}

driverless_msgs::msg::ConeDetectionStamped get_track_centered_on_car_inital_pose(
    ignition::math::Pose3d car_inital_pose, driverless_msgs::msg::ConeDetectionStamped track) {
    driverless_msgs::msg::ConeDetectionStamped centered_track;
    centered_track.header = track.header;

    for (auto const &cone : track.cones_with_cov) {
        auto translated_cone = cone;
        translated_cone.cone = convert_cone_to_car_frame(car_inital_pose, cone.cone);
        centered_track.cones_with_cov.push_back(translated_cone);
        centered_track.cones.push_back(translated_cone.cone);
    }

    return centered_track;
}

typedef struct SLAMConfig {
    std::string frame_id;
    double x_noise;
    double y_noise;
    std::string local_frame_id;
    double local_range_x;
    double local_range_y;
} SLAMConfig_t;

SLAMConfig_t populate_slam_config(gazebo_ros::Node::SharedPtr node) {
    std::string frame_id = node->declare_parameter("slam_frame_id", "map");
    double x_noise = node->declare_parameter("slam_x_noise", 0.0);
    double y_noise = node->declare_parameter("slam_y_noise", 0.0);
    std::string local_frame_id = node->declare_parameter("slam_local_frame_id", "base_link");
    double local_range_x = node->declare_parameter("slam_local_range_x", 0.0);
    double local_range_y = node->declare_parameter("slam_local_range_y", 0.0);

    return {frame_id, x_noise, y_noise, local_frame_id, local_range_x, local_range_y};
}

driverless_msgs::msg::ConeDetectionStamped get_noisy_global_map(
    SLAMConfig_t slam_config, driverless_msgs::msg::ConeDetectionStamped ground_truth_track) {
    driverless_msgs::msg::ConeDetectionStamped noisy_global_map;
    noisy_global_map.header = ground_truth_track.header;
    for (auto const &cone : ground_truth_track.cones_with_cov) {
        noisy_global_map.cones_with_cov.push_back(
            make_noisy_x_y_cone(cone.cone, slam_config.x_noise, slam_config.y_noise));
        noisy_global_map.cones.push_back(make_noisy_x_y_cone(cone.cone, slam_config.x_noise, slam_config.y_noise).cone);
    }

    return noisy_global_map;
}

driverless_msgs::msg::ConeDetectionStamped get_noisy_local_map(
    SLAMConfig_t slam_config, ignition::math::Pose3d car_pose,
    driverless_msgs::msg::ConeDetectionStamped noisy_global_map) {
    driverless_msgs::msg::ConeDetectionStamped noisy_local_map;
    noisy_local_map.header = noisy_global_map.header;
    noisy_local_map.header.frame_id = slam_config.local_frame_id;

    for (auto const &cone : noisy_global_map.cones_with_cov) {
        auto translated_cone = cone;
        translated_cone.cone = convert_cone_to_car_frame(car_pose, cone.cone);
        bool within_x_range =
            0 < translated_cone.cone.location.x && translated_cone.cone.location.x < slam_config.local_range_x / 2;
        bool within_y_range = abs(translated_cone.cone.location.y) < slam_config.local_range_y;
        if (within_x_range && within_y_range) {
            noisy_local_map.cones_with_cov.push_back(translated_cone);
            noisy_local_map.cones.push_back(translated_cone.cone);
        }
    }

    return noisy_local_map;
}
