#include "gazebo_cone_detection_plugin/gazebo_ros_cone_detection.hpp"

#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Dense"

#include "helpers_gazebo.hpp"
#include "helpers_track.hpp"
#include "helpers_ros.hpp"
#include "helpers_detection.hpp"

namespace gazebo_plugins {
namespace eufs_plugins {

GZ_REGISTER_MODEL_PLUGIN(ConeDetectionPlugin)

ConeDetectionPlugin::ConeDetectionPlugin() { }

void ConeDetectionPlugin::Load(gazebo::physics::ModelPtr parent, sdf::ElementPtr sdf) {
    ros_node = gazebo_ros::Node::Get(sdf);

    RCLCPP_INFO(ros_node->get_logger(), "Initalising Cone Detection Plugin.");

    world = parent->GetWorld();
    track_model = get_model(world, "track", ros_node->get_logger());
    update_rate = get_double_parameter(sdf, "updateRate", 0, "0.0 (as fast as possible)", ros_node->get_logger());

    bool publish_ground_truth = get_bool_parameter(sdf, "publishGroundTruth", true, "true");
    bool simulate_perception = get_bool_parameter(sdf, "simulatePerception", true, "true");
    bool simulate_SLAM = get_bool_parameter(sdf, "simulateSLAM", true, "true");

    if (publish_ground_truth) {
        ground_truth_pub = ros_node->create_publisher<driverless_msgs::msg::ConeDetectionStamped>(("ground_truth/global_map"), 1);
    }

    if (simulate_perception) {
        lidar_detection_pub = ros_node->create_publisher<driverless_msgs::msg::ConeDetectionStamped>(("lidar/cone_detection"), 1);
        vision_detection_pub = ros_node->create_publisher<driverless_msgs::msg::ConeDetectionStamped>(("vision/cone_detection"), 1);
    }

    if (simulate_SLAM) {
        slam_global_pub = ros_node->create_publisher<driverless_msgs::msg::ConeDetectionStamped>(("slam/global_map"), 1);
        slam_local_pub = ros_node->create_publisher<driverless_msgs::msg::ConeDetectionStamped>(("slam/local_map"), 1);
    }

    last_update = world->SimTime();
    update_connection = gazebo::event::Events::ConnectWorldUpdateBegin(std::bind(&ConeDetectionPlugin::UpdateChild, this));
}

void ConeDetectionPlugin::UpdateChild() {
    auto curr_time = world->SimTime();
    if (calc_dt(last_update, curr_time) < (1.0 / update_rate)) {
        return;
    }
    last_update = curr_time;
    
    ignition::math::Pose3d car_pose;
    driverless_msgs::msg::ConeDetectionStamped ground_truth_track = get_ground_truth_track(track_model, curr_time, ros_node->get_logger());

    if (has_subscribers(ground_truth_pub)) {
        ground_truth_pub->publish(ground_truth_track);
    }

    if (has_subscribers(lidar_detection_pub)) {
        driverless_msgs::msg::ConeDetectionStamped lidar_detection = get_lidar_detection(car_pose, ground_truth_track);
        lidar_detection_pub->publish(lidar_detection);
    }

    if (has_subscribers(vision_detection_pub)) {
        driverless_msgs::msg::ConeDetectionStamped vision_detection = get_vision_detection(car_pose, ground_truth_track);
        vision_detection_pub->publish(vision_detection);
    }
    
    if (has_subscribers(slam_global_pub)) {
        driverless_msgs::msg::ConeDetectionStamped slam_global_map = get_slam_global_map(car_pose, ground_truth_track);
        slam_global_pub->publish(slam_global_map);
    }

    if (has_subscribers(slam_local_pub)) {
        driverless_msgs::msg::ConeDetectionStamped slam_local_map = get_slam_local_map(car_pose, ground_truth_track);
        slam_local_pub->publish(slam_local_map);
    }
}

}  // namespace eufs_plugins
}  // namespace gazebo_plugins
