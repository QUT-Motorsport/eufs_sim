#pragma once

// Include custom driverless messages for cones and cone detections
#include <driverless_msgs/msg/cone.hpp>
#include <driverless_msgs/msg/cone_detection_stamped.hpp>
// Gazebo math for Pose3 (similar to ignition::math::Pose3d)
#include <gz/math/Pose3.hh>
#include <string>
#include <rclcpp/rclcpp.hpp>

#include "helpers_gazebo.hpp"

// Global seed for generating noise
unsigned int seed = 0;

// Structure to hold sensor configuration parameters
struct SensorConfig {
    std::string frame_id;         // The frame where the sensor is mounted
    double min_view_distance;     // Minimum distance the sensor can view
    double max_view_distance;     // Maximum viewing distance
    double fov;                   // Field of view (radians)
    double range_noise;           // Noise to apply to range measurements
    double bearing_noise;         // Noise to apply to bearing measurements
    bool detects_colour;          // Whether the sensor detects cone color
    double offset_x;              // Optional x offset (e.g., sensor mount offset)
};

using SensorConfig_t = SensorConfig;

// Populate sensor configuration parameters from ROS parameters
SensorConfig_t populate_sensor_config(std::string sensor_prefix, rclcpp::Node::SharedPtr node) {
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

// Convert a cone's coordinates from world frame to car frame using the car's pose.
// Optionally applies an offset in the x-direction.
driverless_msgs::msg::Cone convert_cone_to_car_frame(const gz::math::Pose3d &car_pose,
                                                     const driverless_msgs::msg::Cone &cone,
                                                     double offset_x = 0) {
    driverless_msgs::msg::Cone translated_cone = cone;

    // Compute relative x,y difference from car position
    double x = cone.location.x - car_pose.Pos().X();
    double y = cone.location.y - car_pose.Pos().Y();
    double yaw = car_pose.Rot().Yaw();

    // Rotate the relative coordinates by the car's yaw and apply offset
    translated_cone.location.y = (cos(yaw) * y) - (sin(yaw) * x);
    translated_cone.location.x = (sin(yaw) * y) + (cos(yaw) * x) - offset_x;

    return translated_cone;
}

// Calculate Euclidean distance of the cone from the origin (x, y)
double cone_dist(driverless_msgs::msg::Cone &cone) {
    return sqrt(cone.location.x * cone.location.x + cone.location.y * cone.location.y);
}

// Calculate the angle (bearing) of the cone from the origin
double cone_angle(driverless_msgs::msg::Cone &cone) { 
    return atan2(cone.location.y, cone.location.x); 
}

// Generate a normally distributed random value with mean mu and standard deviation sigma
double GaussianKernel(double mu, double sigma) {
    // Using Box-Muller transform
    double U = static_cast<double>(rand_r(&seed)) / static_cast<double>(RAND_MAX);
    double V = static_cast<double>(rand_r(&seed)) / static_cast<double>(RAND_MAX);
    double X = sqrt(-2.0 * ::log(U)) * cos(2.0 * M_PI * V);
    return sigma * X + mu; // What the sigma?
}

// Create a cone with noise applied in range and bearing based on provided noise parameters
driverless_msgs::msg::ConeWithCovariance make_noisy_range_bearing_cone(driverless_msgs::msg::Cone cone,
                                                                       double range_noise, double bearing_noise) {
    driverless_msgs::msg::ConeWithCovariance noisy_cone;

    double range = cone_dist(cone) + GaussianKernel(0, range_noise);
    double bearing = cone_angle(cone) + GaussianKernel(0, bearing_noise);

    noisy_cone.cone = cone;
    noisy_cone.cone.location.x = range * cos(bearing);
    noisy_cone.cone.location.y = range * sin(bearing);

    // Set covariance (simplified: variance for range and bearing)
    noisy_cone.covariance = {range_noise * range_noise, 0, 0, bearing_noise * bearing_noise};

    return noisy_cone;
}

// Create a cone with noise applied directly to x and y coordinates
driverless_msgs::msg::ConeWithCovariance make_noisy_x_y_cone(driverless_msgs::msg::Cone cone, double x_noise,
                                                             double y_noise) {
    driverless_msgs::msg::ConeWithCovariance noisy_cone;
    noisy_cone.cone = cone;
    noisy_cone.cone.location.x += GaussianKernel(0, x_noise);
    noisy_cone.cone.location.y += GaussianKernel(0, y_noise);
    noisy_cone.covariance = {x_noise * x_noise, 0, 0, y_noise * y_noise};
    return noisy_cone;
}

// Given a sensor configuration, the car's pose, and a ground truth track,
// compute the sensor's detected cones (with noise applied and within sensor limits).
driverless_msgs::msg::ConeDetectionStamped get_sensor_detection(
    SensorConfig_t sensor_config, gz::math::Pose3d car_pose,
    driverless_msgs::msg::ConeDetectionStamped ground_truth_track) {
    driverless_msgs::msg::ConeDetectionStamped sensor_detection;
    sensor_detection.header = ground_truth_track.header;
    sensor_detection.header.frame_id = sensor_config.frame_id;

    // Process each cone in the ground truth track
    for (auto const &cone : ground_truth_track.cones_with_cov) {
        auto translated_cone = convert_cone_to_car_frame(car_pose, cone.cone, sensor_config.offset_x);

        double dist = cone_dist(translated_cone);
        // Skip cones outside of sensor's view distance
        if (dist < sensor_config.min_view_distance || dist > sensor_config.max_view_distance) {
            continue;
        }

        double angle = cone_angle(translated_cone);
        // Skip cones outside of sensor's field of view
        if (abs(angle) > sensor_config.fov / 2) {
            continue;
        }

        // Optionally ignore cone colour information if sensor does not detect it
        if (!sensor_config.detects_colour) {
            translated_cone.color = driverless_msgs::msg::Cone::UNKNOWN;
        }

        // Apply noise and add the cone to the detection results
        driverless_msgs::msg::ConeWithCovariance noisy_cone =
            make_noisy_range_bearing_cone(translated_cone, sensor_config.range_noise, sensor_config.bearing_noise);
        sensor_detection.cones_with_cov.push_back(noisy_cone);
        sensor_detection.cones.push_back(noisy_cone.cone);
    }

    return sensor_detection;
}

// Center the track based on the car's initial pose. This translates all cone positions.
driverless_msgs::msg::ConeDetectionStamped get_track_centered_on_car_inital_pose(
    gz::math::Pose3d car_inital_pose, driverless_msgs::msg::ConeDetectionStamped track) {
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

// Structure to hold SLAM configuration parameters
typedef struct SLAMConfig {
    std::string frame_id;        // Global frame for SLAM
    double x_noise;              // Noise on x-axis for SLAM
    double y_noise;              // Noise on y-axis for SLAM
    std::string local_frame_id;  // Local frame for SLAM results
    double local_range_x;        // Local x-range for filtering cones
    double local_range_y;        // Local y-range for filtering cones
} SLAMConfig_t;

 // Helper function to load SLAM configuration parameters from ROS
SLAMConfig_t populate_slam_config(rclcpp::Node::SharedPtr node) {
    std::string frame_id = node->declare_parameter("slam_frame_id", "map");
    double x_noise = node->declare_parameter("slam_x_noise", 0.0);
    double y_noise = node->declare_parameter("slam_y_noise", 0.0);
    std::string local_frame_id = node->declare_parameter("slam_local_frame_id", "base_link");
    double local_range_x = node->declare_parameter("slam_local_range_x", 0.0);
    double local_range_y = node->declare_parameter("slam_local_range_y", 0.0);

    return {frame_id, x_noise, y_noise, local_frame_id, local_range_x, local_range_y};
}

// Generate a noisy global map by applying noise to each cone in the ground truth track.
driverless_msgs::msg::ConeDetectionStamped get_noisy_global_map(
    SLAMConfig_t slam_config, driverless_msgs::msg::ConeDetectionStamped ground_truth_track) {
    driverless_msgs::msg::ConeDetectionStamped noisy_global_map;
    noisy_global_map.header = ground_truth_track.header;
    for (auto const &cone : ground_truth_track.cones_with_cov) {
        noisy_global_map.cones_with_cov.push_back(
            make_noisy_x_y_cone(cone.cone, slam_config.x_noise, slam_config.y_noise));
        noisy_global_map.cones.push_back(
            make_noisy_x_y_cone(cone.cone, slam_config.x_noise, slam_config.y_noise).cone);
    }

    return noisy_global_map;
}

// Generate a noisy local map by centering the noisy global map on the car's current pose
// and filtering cones based on a local range.
driverless_msgs::msg::ConeDetectionStamped get_noisy_local_map(
    SLAMConfig_t slam_config, gz::math::Pose3d car_pose,
    driverless_msgs::msg::ConeDetectionStamped noisy_global_map) {
    driverless_msgs::msg::ConeDetectionStamped noisy_local_map;
    noisy_local_map.header = noisy_global_map.header;
    // Set the frame to the local frame
    noisy_local_map.header.frame_id = slam_config.local_frame_id;

    for (auto const &cone : noisy_global_map.cones_with_cov) {
        auto translated_cone = cone;
        translated_cone.cone = convert_cone_to_car_frame(car_pose, cone.cone);
        // Filter cones within the specified local x and y ranges
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
