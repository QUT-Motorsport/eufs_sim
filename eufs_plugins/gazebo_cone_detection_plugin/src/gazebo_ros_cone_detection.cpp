#include "gazebo_cone_detection_plugin/gazebo_ros_cone_detection.hpp"

namespace gazebo_plugins {
namespace eufs_plugins {

GZ_REGISTER_MODEL_PLUGIN(ConeDetectionPlugin)

ConeDetectionPlugin::ConeDetectionPlugin() {}

void ConeDetectionPlugin::Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) {
    _ros_node = gazebo_ros::Node::Get(sdf);

    RCLCPP_DEBUG(_ros_node->get_logger(), "Loading SBGPlugin");

    _model = model;
    _world = _model->GetWorld();

    // Initialize parameters
    initParams();

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

    // Cone position reset service
    reset_cone_pos_srv = _ros_node->create_service<std_srvs::srv::Trigger>(
        "/system/reset_cones",
        std::bind(&ConeDetectionPlugin::resetConePosition, this, std::placeholders::_1, std::placeholders::_2));

    _last_gt_update = _world->SimTime();
    _last_lidar_update = _world->SimTime();
    _last_camera_update = _world->SimTime();
    _last_slam_update = _world->SimTime();

    _update_connection = gazebo::event::Events::ConnectWorldUpdateBegin(std::bind(&ConeDetectionPlugin::update, this));

    //  Store initial track
    _initial_track = get_ground_truth_track(_track_model, _last_gt_update, _map_frame, _ros_node->get_logger());

    RCLCPP_INFO(_ros_node->get_logger(), "ConeDetectionPlugin Loaded");
}

void ConeDetectionPlugin::initParams() {
    _map_frame = _ros_node->declare_parameter("map_frame", "map");
    _base_frame = _ros_node->declare_parameter("base_frame", "base_link");

    _track_model = get_model(_world, "track", _ros_node->get_logger());
    _car_link = get_link(_model, _base_frame, _ros_node->get_logger());
    _car_inital_pose = _car_link->WorldPose();

    _lidar_update_rate = _ros_node->declare_parameter("lidar_update_rate", 1.0);
    _camera_update_rate = _ros_node->declare_parameter("camera_update_rate", 1.0);
    _slam_update_rate = _ros_node->declare_parameter("slam_update_rate", 1.0);
    _gt_update_rate = _ros_node->declare_parameter("gt_update_rate", 1.0);

    _pub_gt = _ros_node->declare_parameter("publish_ground_truth", false);
    _simulate_perception = _ros_node->declare_parameter("simulate_perception", false);
    _simulate_slam = _ros_node->declare_parameter("simulate_slam", false);

    _lidar_config = populate_sensor_config("lidar", _ros_node);
    _camera_config = populate_sensor_config("camera", _ros_node);
    _slam_config = populate_slam_config(_ros_node);
}

void ConeDetectionPlugin::update() {
    publishGTTrack();
    publishLiDARDetection();
    publishCameraDetection();
    publishSLAM();
}

void ConeDetectionPlugin::publishGTTrack() {
    auto curr_time = _world->SimTime();
    if (calc_dt(_last_gt_update, curr_time) < (1.0 / _gt_update_rate)) {
        return;
    }
    _last_gt_update = curr_time;

    auto ground_truth_track = get_ground_truth_track(_track_model, curr_time, _map_frame, _ros_node->get_logger());

    if (has_subscribers(_ground_truth_pub)) {
        auto centered_ground_truth = get_track_centered_on_car_inital_pose(_car_inital_pose, ground_truth_track);
        _ground_truth_pub->publish(centered_ground_truth);
    }
}

void ConeDetectionPlugin::publishLiDARDetection() {
    auto curr_time = _world->SimTime();
    if (calc_dt(_last_lidar_update, curr_time) < (1.0 / _lidar_update_rate)) {
        return;
    }
    _last_lidar_update = curr_time;

    auto ground_truth_track = get_ground_truth_track(_track_model, curr_time, _map_frame, _ros_node->get_logger());

    if (has_subscribers(_lidar_detection_pub)) {
        auto lidar_detection = get_sensor_detection(_lidar_config, _car_link->WorldPose(), ground_truth_track);
        _lidar_detection_pub->publish(lidar_detection);
    }
}

void ConeDetectionPlugin::publishCameraDetection() {
    auto curr_time = _world->SimTime();
    if (calc_dt(_last_camera_update, curr_time) < (1.0 / _camera_update_rate)) {
        return;
    }
    _last_camera_update = curr_time;

    auto ground_truth_track = get_ground_truth_track(_track_model, curr_time, _map_frame, _ros_node->get_logger());

    if (has_subscribers(_vision_detection_pub)) {
        auto vision_detection = get_sensor_detection(_camera_config, _car_link->WorldPose(), ground_truth_track);
        _vision_detection_pub->publish(vision_detection);
    }
}

void ConeDetectionPlugin::publishSLAM() {
    auto curr_time = _world->SimTime();
    if (calc_dt(_last_slam_update, curr_time) < (1.0 / _slam_update_rate)) {
        return;
    }
    _last_slam_update = curr_time;

    auto ground_truth_track = get_ground_truth_track(_track_model, curr_time, _map_frame, _ros_node->get_logger());

    if (has_subscribers(_slam_global_pub) || has_subscribers(_slam_local_pub)) {
        if (_initial_slam.cones_with_cov.empty()) {
            _initial_slam = get_noisy_global_map(_slam_config, ground_truth_track);
        }

        if (has_subscribers(_slam_global_pub)) {
            auto slam_global_map = get_track_centered_on_car_inital_pose(_car_inital_pose, _initial_slam);
            _slam_global_pub->publish(slam_global_map);
        }

        if (has_subscribers(_slam_local_pub)) {
            auto noisy_local_map = get_noisy_local_map(_slam_config, _car_link->WorldPose(), _initial_slam);
            _slam_local_pub->publish(noisy_local_map);
        }
    }
}

// Resets the position of cones to initial track model
bool ConeDetectionPlugin::resetConePosition(std::shared_ptr<std_srvs::srv::Trigger::Request>,
                                            std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    gazebo::physics::Link_V links = _track_model->GetLinks();

    // Loop through all cones
    for (unsigned int i = 0; i < links.size(); i++) {
        driverless_msgs::msg::ConeWithCovariance cone = _initial_track.cones_with_cov[i];

        // Initial position and velocity variables
        const ignition::math::Pose3d pos(cone.cone.location.x, cone.cone.location.y, cone.cone.location.z, 0.0, 0.0,
                                         0.0);
        const ignition::math::Vector3d vel(0.0, 0.0, 0.0);
        const ignition::math::Vector3d angular(0.0, 0.0, 0.0);

        RCLCPP_INFO(_ros_node->get_logger(), "Resetting cone %d to position (%f, %f, %f)", i, pos.Pos().X(),
                    pos.Pos().Y(), pos.Pos().Z());

        // Set cone position to initial position (and velocity)
        links[i]->SetWorldPose(pos);
        links[i]->SetAngularVel(vel);
        links[i]->SetLinearVel(angular);
    }

    return response->success;
}
}  // namespace eufs_plugins
}  // namespace gazebo_plugins
