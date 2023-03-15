#pragma once

#include <string>

#include <gazebo/gazebo.hh>

#include <driverless_msgs/msg/cone.hpp>
#include <driverless_msgs/msg/cone_detection_stamped.hpp>

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
} SensorConfig_t;


SensorConfig_t populate_sensor_config(
    std::string sensor_prefix,
    sdf::ElementPtr sdf,
    std::optional<const rclcpp::Logger> logger = {}
) {
    return {
        get_string_parameter(sdf, sensor_prefix + "FrameId", "", ""),
        get_double_parameter(sdf, sensor_prefix + "MinViewDistance", 0, "0"),
        get_double_parameter(sdf, sensor_prefix + "MaxViewDistance", 0, "0"),
        get_double_parameter(sdf, sensor_prefix + "FOV", 0, "0"),
        get_double_parameter(sdf, sensor_prefix + "RangeNoise", 0, "0"),
        get_double_parameter(sdf, sensor_prefix + "BearingNoise", 0, "0"),
        get_bool_parameter(sdf, sensor_prefix + "DetectsColour", true, "true"),
    };
}

driverless_msgs::msg::Cone convert_cone_to_car_frame(
    const ignition::math::Pose3d car_pose,
    const driverless_msgs::msg::Cone cone
) {
    driverless_msgs::msg::Cone translated_cone = cone;
    
    double x = cone.location.x - car_pose.Pos().X();
    double y = cone.location.y - car_pose.Pos().Y();
    double yaw = car_pose.Rot().Yaw();

    // Rotate the points using the yaw of the car (x and y are the other way around)
    translated_cone.location.y = (cos(yaw) * y) - (sin(yaw) * x);
    translated_cone.location.x = (sin(yaw) * y) + (cos(yaw) * x);

    return translated_cone;
}

double cone_dist(driverless_msgs::msg::Cone cone) {
    return sqrt(cone.location.x*cone.location.x + cone.location.y*cone.location.y);
}

double cone_angle(driverless_msgs::msg::Cone cone) {
    return atan2(cone.location.y, cone.location.x);
}

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

driverless_msgs::msg::ConeWithCovariance make_noisy_cone(
    driverless_msgs::msg::Cone cone,
    double range_noise,
    double bearing_noise
) {
    driverless_msgs::msg::ConeWithCovariance noisy_cone;
    
    double range = cone_dist(cone) + GaussianKernel(0, range_noise);
    double bearing = cone_angle(cone) + GaussianKernel(0, bearing_noise);

    noisy_cone.cone = cone;
    noisy_cone.cone.location.x = range*cos(bearing);
    noisy_cone.cone.location.y = range*sin(bearing);

    // TODO - how to convert this back to x, y noise?
    noisy_cone.covariance = {range_noise*range_noise, 0, 0, bearing_noise*bearing_noise};

    return noisy_cone;   
}

driverless_msgs::msg::ConeDetectionStamped get_sensor_detection(
    SensorConfig_t sensor_config,
    ignition::math::Pose3d car_pose,
    driverless_msgs::msg::ConeDetectionStamped ground_truth_track
) {
    driverless_msgs::msg::ConeDetectionStamped sensor_detection;
    sensor_detection.header = ground_truth_track.header;
    sensor_detection.header.frame_id = sensor_config.frame_id;

    for (auto const &cone : ground_truth_track.cones_with_cov) {
        auto translated_cone = convert_cone_to_car_frame(car_pose, cone.cone);

        double dist = cone_dist(translated_cone);
        if (dist < sensor_config.min_view_distance || dist > sensor_config.max_view_distance) {
            continue;
        }

        double angle = cone_angle(translated_cone);
        if (abs(angle) > sensor_config.fov/2) {
            continue;
        }

        if (!sensor_config.detects_colour) {
            translated_cone.color = driverless_msgs::msg::Cone::UNKNOWN;
        }

        sensor_detection.cones_with_cov.push_back(
            make_noisy_cone(
                translated_cone,
                sensor_config.range_noise,
                sensor_config.bearing_noise
            )
        );
    }

    return sensor_detection;
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


driverless_msgs::msg::ConeDetectionStamped get_ground_truth_track_centered_on_car_inital_pose(
    ignition::math::Pose3d car_inital_pose,
    driverless_msgs::msg::ConeDetectionStamped ground_truth_track
) {
    driverless_msgs::msg::ConeDetectionStamped centered_ground_truth;
    centered_ground_truth.header = ground_truth_track.header;

    for (auto const &cone : ground_truth_track.cones_with_cov) {
        auto translated_cone = cone;
        translated_cone.cone = convert_cone_to_car_frame(car_inital_pose, cone.cone);
        centered_ground_truth.cones_with_cov.push_back(translated_cone);
    }

    return centered_ground_truth;
}
