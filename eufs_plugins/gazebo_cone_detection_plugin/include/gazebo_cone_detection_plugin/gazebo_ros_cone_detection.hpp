#pragma once

// -----------------------------------------------------------------------------
// Gazebo Includes
// These headers provide access to Gazebo's simulation system, including
// the world, models, entities, components, and event management.
// -----------------------------------------------------------------------------
#include <gz/sim/System.hh>
#include <gz/sim/Entity.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/World.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/Link.hh>
#include <gz/transport/Node.hh>
#include <gz/sim/EventManager.hh>

// -----------------------------------------------------------------------------
// ROS Includes
// These headers provide access to ROS2 services and node functionality.
// -----------------------------------------------------------------------------
#include <std_srvs/srv/trigger.hpp>
#include <rclcpp/rclcpp.hpp>

// -----------------------------------------------------------------------------
// QUTMS Driverless Includes
// Custom messages defined for the driverless system, in this case, Cone and
// ConeDetectionStamped messages which are used for representing cone detections.
// Part of a seperate repo: https://github.com/QUT-Motorsport/QUTMS_Driverless
// -----------------------------------------------------------------------------
#include <driverless_msgs/msg/cone.hpp>
#include <driverless_msgs/msg/cone_detection_stamped.hpp>

// -----------------------------------------------------------------------------
// Eufs Sim Includes
// These helper functions custom wrriten to assist with converting and
// handling sensor data, Gazebo-specific utilities, ROS conversions, and track data.
// -----------------------------------------------------------------------------
#include "helpers_detection.hpp"
#include "helpers_gazebo.hpp"
#include "helpers_ros.hpp"
#include "helpers_track.hpp"

namespace gazebo_plugins {
namespace eufs_plugins {

// -----------------------------------------------------------------------------
// ConeDetectionPlugin Class
//
// This plugin integrates with Gazebos simulation system and is responsible for
// detecting cones on the track and publishing the detection data over ROS.
// It implements Gazebo's system interfaces to configure the plugin and perform
// periodic updates before each simulation iteration.
// -----------------------------------------------------------------------------
class ConeDetectionPlugin : public gz::sim::System,        // Base class for systems in Gazebo
                            public gz::sim::ISystemConfigure, // Interface for configuration (replacement for Load)
                            public gz::sim::ISystemPreUpdate // Interface for pre-update logic (replacement for update)
{
   public:
    // Constructor and destructor
    ConeDetectionPlugin();
    ~ConeDetectionPlugin();

    // -------------------------------------------------------------------------
    // Configure
    // This method is called once when the plugin is loaded.
    // It receives the entity handle of the model, the SDF element,
    // the EntityComponentManager (for accessing simulation components), and the EventManager.
    // It is used to initialize the plugin, create the ROS node, and set up publishers,
    // services, and internal state.
    // Event manager and SDF are currently unused but could be useful in the future
    // So they were left in.
    // -------------------------------------------------------------------------
    void Configure(const gz::sim::Entity &entity,
                   const std::shared_ptr<const sdf::Element> &sdf,
                   gz::sim::EntityComponentManager &ecm,
                   gz::sim::EventManager &eventMgr) override;

    // -------------------------------------------------------------------------
    // PreUpdate
    // This method is called before each simulation iteration.
    // It is used to perform tasks such as checking for new sensor data,
    // publishing updated cone detection information, and processing sensor updates.
    // Better than PostUpdate as you can modify components through the ECM
    // -------------------------------------------------------------------------
    void PreUpdate(const gz::sim::UpdateInfo &info,
                   gz::sim::EntityComponentManager &ecm) override;

   private:
    // -------------------------------------------------------------------------
    // Helper Methods
    // These private functions are used internally by the plugin for:
    // - Initializing parameters (from ROS and SDF)
    // - Publishing ground-truth track information
    // - Publishing LiDAR-based cone detection data
    // - Publishing camera-based cone detection data
    // - Publishing SLAM-based cone detection data
    // - Resetting cone positions (via a ROS service)   
    // -------------------------------------------------------------------------
    void initParams(gz::sim::EntityComponentManager &ecm);
    void publishGTTrack(gz::sim::EntityComponentManager &ecm);
    void publishLiDARDetection(gz::sim::EntityComponentManager &ecm);
    void publishCameraDetection(gz::sim::EntityComponentManager &ecm);
    void publishSLAM(gz::sim::EntityComponentManager &ecm);
    bool resetConePosition(std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                           std::shared_ptr<std_srvs::srv::Trigger::Response> response,
                           gz::sim::EntityComponentManager &ecm);

   private:
    // -------------------------------------------------------------------------
    // Simulation Objects
    // These members store handles to various simulation elements.
    // - _world: Handle to the Gazebo world.
    // - _model: Handle to the robot model this plugin is attached to.
    // - _entity: The raw entity handle.
    // - _ecm: Pointer to the Entity Component Manager for querying/updating simulation data.
    // - _track_model: Handle to the model representing the track (e.g., for ground truth).
    // - _car_link: The specific link on the car (used for relative pose computations).
    // - _car_inital_pose: The initial pose of the car in the world.
    // -------------------------------------------------------------------------
    gz::sim::World _world;
    gz::sim::Model _model;
    gz::sim::Entity _entity;
    gz::sim::EntityComponentManager* _ecm;  // Pointer stored for use in helper functions
    gz::sim::Model _track_model;
    gz::sim::Link _car_link;
    gz::math::Pose3d _car_inital_pose;  

    // -------------------------------------------------------------------------
    // ROS Node and Communication
    // The plugin creates its own ROS node to publish sensor data and provide services.
    // - _ros_node: The ROS2 node used for communication.
    // - _map_frame and _base_frame: Frame identifiers used for coordinate transformations.
    // -------------------------------------------------------------------------
    std::shared_ptr<rclcpp::Node> _ros_node;
    std::string _map_frame;
    std::string _base_frame;

    // -------------------------------------------------------------------------
    // Plugin Behavior Flags and Update Rates
    // These booleans control whether the plugin publishes ground truth, simulates perception,
    // or simulates SLAM data. The update rates determine how frequently each type of data is published.
    // -------------------------------------------------------------------------
    bool _pub_gt;
    bool _simulate_perception;
    bool _simulate_slam;
    double _gt_update_rate;
    double _lidar_update_rate;
    double _camera_update_rate;
    double _slam_update_rate;

    // -------------------------------------------------------------------------
    // Timing Information
    // These time points record the last time each type of sensor data was published.
    // They are used to enforce the desired update rates.
    // All of the time calls changed with the switch to Gazebo so these are quite
    // different from previous versions.
    // -------------------------------------------------------------------------
    std::chrono::time_point<std::chrono::steady_clock> _last_gt_update;
    std::chrono::time_point<std::chrono::steady_clock> _last_lidar_update;
    std::chrono::time_point<std::chrono::steady_clock> _last_camera_update;
    std::chrono::time_point<std::chrono::steady_clock> _last_slam_update;

    // -------------------------------------------------------------------------
    // Initial Data Storage
    // These messages store the initial ground truth track and SLAM map that the plugin uses
    // for computing relative detections.
    // -------------------------------------------------------------------------
    driverless_msgs::msg::ConeDetectionStamped _initial_track;
    driverless_msgs::msg::ConeDetectionStamped _initial_slam;

    // -------------------------------------------------------------------------
    // ROS Publishers
    // Publishers used to send cone detection data on various topics.
    // - _ground_truth_pub: Publishes ground truth map data.
    // - _lidar_detection_pub: Publishes LiDAR-based cone detection data.
    // - _vision_detection_pub: Publishes camera-based cone detection data.
    // - _slam_global_pub and _slam_local_pub: Publish SLAM-based cone detection data.
    // -------------------------------------------------------------------------
    rclcpp::Publisher<driverless_msgs::msg::ConeDetectionStamped>::SharedPtr _ground_truth_pub;
    rclcpp::Publisher<driverless_msgs::msg::ConeDetectionStamped>::SharedPtr _lidar_detection_pub;
    rclcpp::Publisher<driverless_msgs::msg::ConeDetectionStamped>::SharedPtr _vision_detection_pub;
    rclcpp::Publisher<driverless_msgs::msg::ConeDetectionStamped>::SharedPtr _slam_global_pub;
    rclcpp::Publisher<driverless_msgs::msg::ConeDetectionStamped>::SharedPtr _slam_local_pub;

    // -------------------------------------------------------------------------
    // ROS Services
    // Service to reset cone positions. Crazy right?
    // -------------------------------------------------------------------------
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_cone_pos_srv;

    // -------------------------------------------------------------------------
    // Sensor and SLAM Configuration Structures
    // These types (SensorConfig_t and SLAMConfig_t) are likely defined in the helpers files
    // and hold configuration parameters for the LiDAR sensor and SLAM simulation.
    // -------------------------------------------------------------------------
    SensorConfig_t _lidar_config;
    SensorConfig_t _camera_config;
    SLAMConfig_t _slam_config;
};

}  // namespace eufs_plugins
}  // namespace gazebo_plugins
