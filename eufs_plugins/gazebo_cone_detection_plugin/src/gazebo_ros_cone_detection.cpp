#pragma once

#include "gazebo_cone_detection_plugin/gazebo_ros_cone_detection.hpp"
#include <gz/plugin/Register.hh>

namespace gazebo_plugins {
namespace eufs_plugins {

// Default constructor and destructor.
ConeDetectionPlugin::ConeDetectionPlugin() = default;
ConeDetectionPlugin::~ConeDetectionPlugin() = default;

/**
 * @brief Configure the plugin.
 * 
 * This method initializes the plugin, creates the ROS node, sets up publishers,
 * and initializes parameters from ROS.
 */
void ConeDetectionPlugin::Configure(const gz::sim::Entity &entity,
                   const std::shared_ptr<const sdf::Element> &sdf,
                   gz::sim::EntityComponentManager &ecm,
                   gz::sim::EventManager &eventMgr)
{
    (void)sdf; // not used
    (void)eventMgr; // also not used

    // Stores simulation objects
    _ecm = &ecm;
    _entity = entity;
    _model = gz::sim::Model(entity);

    if (!_model.Valid(ecm)) {
        std::cerr << "Invalid model entity, plugin won't run." << std::endl;
        return;
    }

    // get world handle
    _world = gz::sim::World(_model.Entity());

    // Create ros node
    _ros_node = std::make_shared<rclcpp::Node>("cone_detection_plugin");
    RCLCPP_DEBUG(_ros_node->get_logger(), "Loading SVCheats");
    RCLCPP_DEBUG(_ros_node->get_logger(), ".noclip");
    // Gmodders are obviously not bound to the laws of Physics. Along with their shapeshifting capabilities, 
    // they are able to control their density and weight. They are able to make the gaps between their 
    // electrons so wide to the point that they go through solid objects, and they can make themselves so 
    // light where they are able to fly through the air. 

    // Initialize parameters
    initParams(ecm);

    // Setup ros publishers with their parameters.
    if (_pub_gt) {
        _ground_truth_pub =
            _ros_node->create_publisher<driverless_msgs::msg::ConeDetectionStamped>(("ground_truth/global_map"), 1);
    }
    if (_simulate_perception) {
        _lidar_detection_pub =
            _ros_node->create_publisher<driverless_msgs::msg::ConeDetectionStamped>(("lidar/cone_detection"), 1);
        _vision_detection_pub =
            _ros_node->create_publisher<driverless_msgs::msg::ConeDetectionStamped>(("vision/cone_detection"), 1);
    }

    if (_simulate_slam) {
        _slam_global_pub =
            _ros_node->create_publisher<driverless_msgs::msg::ConeDetectionStamped>(("slam/global_map"), 1);
        _slam_local_pub =
            _ros_node->create_publisher<driverless_msgs::msg::ConeDetectionStamped>(("slam/local_map"), 1);
    }

    // Initialise time stamps.
    _last_gt_update = _last_lidar_update = _last_camera_update = _last_slam_update = std::chrono::steady_clock::now();

    //  Store initial track and GTT
    _initial_track = get_ground_truth_track(_track_model, ecm, _map_frame, _ros_node->get_logger());

    RCLCPP_INFO(_ros_node->get_logger(), "ConeDetectionPlugin Loaded");
}

/**
 * @brief Initialize plugin parameters from ROS.
 */
void ConeDetectionPlugin::initParams(gz::sim::EntityComponentManager &ecm) {
    _map_frame = _ros_node->declare_parameter("map_frame", "map");
    _base_frame = _ros_node->declare_parameter("base_frame", "base_link");
    std::string track_model_name = _ros_node->declare_parameter("track_model", "track");

    // Get track from world model
    _track_model = eufs_plugins::getModel(_world, *_ecm, "track", _ros_node->get_logger());
    _car_link = eufs_plugins::get_link(_model, *_ecm, _base_frame, _ros_node->get_logger());
    _car_inital_pose = gz::sim::worldPose(_entity, ecm);

    // Gets sensor update rates.
    _lidar_update_rate = _ros_node->declare_parameter("lidar_update_rate", 1.0);
    _camera_update_rate = _ros_node->declare_parameter("camera_update_rate", 1.0);
    _slam_update_rate = _ros_node->declare_parameter("slam_update_rate", 1.0);
    _gt_update_rate = _ros_node->declare_parameter("gt_update_rate", 1.0);

    // Plugin behaviour flags.
    _pub_gt = _ros_node->declare_parameter("publish_ground_truth", false);
    _simulate_perception = _ros_node->declare_parameter("simulate_perception", false);
    _simulate_slam = _ros_node->declare_parameter("simulate_slam", false);

    // Does what it says it does based on information in the ros node
    _lidar_config = populate_sensor_config("lidar", _ros_node);
    _camera_config = populate_sensor_config("camera", _ros_node);
    _slam_config = populate_slam_config(_ros_node);
}


/**
 * @brief Called before each simulation update.
 *
 * Publishes ground truth, LiDAR, camera, and SLAM detections if appropriate.
 */
void ConeDetectionPlugin::PreUpdate(const gz::sim::UpdateInfo &info,
                                    gz::sim::EntityComponentManager &ecm)
{
    if (info.paused)
        return;

    publishGTTrack(ecm);
    publishLiDARDetection(ecm);
    publishCameraDetection(ecm);
    publishSLAM(ecm);
}

/**
 * @brief Publish the ground truth track if there are subscribers.
 */
void ConeDetectionPlugin::publishGTTrack(gz::sim::EntityComponentManager &ecm) {
    auto curr_time = std::chrono::steady_clock::now();
    if (std::chrono::duration<double>(curr_time - _last_gt_update).count() < (1.0 / _gt_update_rate)) {
        return;
    }
    _last_gt_update = curr_time;

    auto ground_truth_track = get_ground_truth_track(_track_model, ecm, _map_frame, _ros_node->get_logger());

    if (has_subscribers(_ground_truth_pub)) {
        auto centered_ground_truth = get_track_centered_on_car_inital_pose(_car_inital_pose, ground_truth_track);
        _ground_truth_pub->publish(centered_ground_truth);
    }
}

/**
 * @brief Publish LiDAR detection data if there are subscribers.
 */
void ConeDetectionPlugin::publishLiDARDetection(gz::sim::EntityComponentManager &ecm) {
    auto curr_time = std::chrono::steady_clock::now();
    if (std::chrono::duration<double>(curr_time - _last_lidar_update).count() < (1.0 / _lidar_update_rate)) {
        return;
    }
    _last_lidar_update = curr_time;

    auto ground_truth_track = get_ground_truth_track(_track_model, ecm, _map_frame, _ros_node->get_logger());

    if (has_subscribers(_lidar_detection_pub)) {
        auto lidar_detection = get_sensor_detection(_lidar_config, _car_link.WorldPose(ecm).value(), ground_truth_track);
        _lidar_detection_pub->publish(lidar_detection);
    }
}

/**
 * @brief Publish camera-based detection data if there are subscribers.
 */
void ConeDetectionPlugin::publishCameraDetection(gz::sim::EntityComponentManager &ecm) {
    auto curr_time = std::chrono::steady_clock::now();
        if (std::chrono::duration<double>(curr_time - _last_camera_update).count() < (1.0 / _camera_update_rate))
            return;
    
    _last_camera_update = curr_time;

    auto ground_truth_track = get_ground_truth_track(_track_model, ecm, _map_frame, _ros_node->get_logger());

    if (has_subscribers(_vision_detection_pub)) {
        auto vision_detection = get_sensor_detection(_camera_config, _car_link.WorldPose(ecm).value(), ground_truth_track);
        _vision_detection_pub->publish(vision_detection);
    }
}

/**
 * @brief Publish SLAM detection data if there are subscribers.
 */
void ConeDetectionPlugin::publishSLAM(gz::sim::EntityComponentManager &ecm) {
    auto curr_time = std::chrono::steady_clock::now();
    if (std::chrono::duration<double>(curr_time - _last_slam_update).count() < (1.0 / _slam_update_rate)) {
        return;
    }
    _last_slam_update = curr_time;

    auto ground_truth_track = get_ground_truth_track(_track_model, ecm, _map_frame, _ros_node->get_logger());

    if (has_subscribers(_slam_global_pub) || has_subscribers(_slam_local_pub)) {
        if (_initial_slam.cones_with_cov.empty()) {
            _initial_slam = get_noisy_global_map(_slam_config, ground_truth_track);
        }

        if (has_subscribers(_slam_global_pub)) {
            auto slam_global_map = get_track_centered_on_car_inital_pose(_car_inital_pose, _initial_slam);
            _slam_global_pub->publish(slam_global_map);
        }

        if (has_subscribers(_slam_local_pub)) {
            auto noisy_local_map = get_noisy_local_map(_slam_config, _car_link.WorldPose(ecm).value(), _initial_slam);
            _slam_local_pub->publish(noisy_local_map);
        }
    }
}

/**
 * @brief Reset all cones to their initial positions.
 *
 * This service resets the position and velocities of all cones in the track.
 */
bool ConeDetectionPlugin::resetConePosition(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                            std::shared_ptr<std_srvs::srv::Trigger::Response> response,
                                            gz::sim::EntityComponentManager &ecm) 
{
    (void)request; // not used :)        

    std::vector<gz::sim::Entity> links = _track_model.Links(ecm);

    // Loop through all cones
    for (unsigned int i = 0; i < links.size(); i++) {
        driverless_msgs::msg::ConeWithCovariance cone = _initial_track.cones_with_cov[i];

        // Initial position and velocity variables
        gz::math::Pose3d pos(cone.cone.location.x, cone.cone.location.y, cone.cone.location.z, 0.0, 0.0, 0.0);
        gz::math::Vector3d vel(0.0, 0.0, 0.0);
        gz::math::Vector3d angular(0.0, 0.0, 0.0);

        RCLCPP_INFO(_ros_node->get_logger(), "Resetting cone %d to position (%f, %f, %f)",
                    i, pos.Pos().X(),pos.Pos().Y(), pos.Pos().Z());

        // Set cone position to initial position (and velocity)
        gz::sim::Link link(links[i]);
        // link.SetWorldPose(pos);
        link.SetAngularVelocity(ecm, vel);
        link.SetLinearVelocity(ecm, angular);
    }

    response->success = true;
    response->message = "Cones reset";
    return true;
    }

GZ_ADD_PLUGIN(eufs_plugins::ConeDetectionPlugin, 
              ::gz::sim::System,
              eufs_plugins::ConeDetectionPlugin::ISystemConfigure,
              eufs_plugins::ConeDetectionPlugin::ISystemPreUpdate)


}  // namespace eufs_plugins
}  // namespace gazebo_plugins
