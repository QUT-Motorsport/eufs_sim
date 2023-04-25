#include "gazebo_cone_detection_plugin/gazebo_ros_cone_detection.hpp"

namespace gazebo_plugins {
namespace eufs_plugins {

GZ_REGISTER_MODEL_PLUGIN(ConeDetectionPlugin)

ConeDetectionPlugin::ConeDetectionPlugin() {}

void ConeDetectionPlugin::Load(gazebo::physics::ModelPtr parent, sdf::ElementPtr sdf) {
    _ros_node = gazebo_ros::Node::Get(sdf);

    world = parent->GetWorld();
    track_model = get_model(world, "track", _ros_node->get_logger());

    car_link = get_link(parent, "base_footprint", _ros_node->get_logger());
    car_inital_pose = car_link->WorldPose();

    update_rate = get_double_parameter(sdf, "updateRate", 0, "0.0 (as fast as possible)", _ros_node->get_logger());

    bool publish_ground_truth = get_bool_parameter(sdf, "publishGroundTruth", true, "true");
    bool simulate_perception = get_bool_parameter(sdf, "simulatePerception", true, "true");
    bool simulate_SLAM = get_bool_parameter(sdf, "simulateSLAM", true, "true");

    lidar_config = populate_sensor_config("lidar", sdf, _ros_node->get_logger());
    std::string lidar_topic = get_string_parameter(sdf, "lidarDetectionTopic", "lidar", "lidar", _ros_node->get_logger());

    camera_config = populate_sensor_config("camera", sdf, _ros_node->get_logger());
    std::string camera_topic = get_string_parameter(sdf, "cameraDetectionTopic", "camera", "camera", _ros_node->get_logger());

    slam_config = populate_slam_config(sdf, _ros_node->get_logger());

    if (publish_ground_truth) {
        ground_truth_pub =
            _ros_node->create_publisher<driverless_msgs::msg::ConeDetectionStamped>(("ground_truth/global_map"), 1);
    }

    if (simulate_perception) {
        lidar_detection_pub = _ros_node->create_publisher<driverless_msgs::msg::ConeDetectionStamped>((lidar_topic), 1);
        vision_detection_pub =
            _ros_node->create_publisher<driverless_msgs::msg::ConeDetectionStamped>((camera_topic), 1);
    }

    if (simulate_SLAM) {
        slam_global_pub =
            _ros_node->create_publisher<driverless_msgs::msg::ConeDetectionStamped>(("slam/global_map"), 1);
        slam_local_pub = _ros_node->create_publisher<driverless_msgs::msg::ConeDetectionStamped>(("slam/local_map"), 1);
    }

    // Cone position reset service
    reset_cone_pos_srv = _ros_node->create_service<std_srvs::srv::Trigger>(
        "/ros_can/reset_cone_pos",
        std::bind(&ConeDetectionPlugin::resetConePosition, this, std::placeholders::_1, std::placeholders::_2));

    last_update = world->SimTime();
    update_connection =
        gazebo::event::Events::ConnectWorldUpdateBegin(std::bind(&ConeDetectionPlugin::UpdateChild, this));

    //  Store initial track
    initial_track = get_ground_truth_track(track_model, last_update, _ros_node->get_logger());

    RCLCPP_INFO(_ros_node->get_logger(), "ConeDetectionPlugin Loaded");
}

void ConeDetectionPlugin::UpdateChild() {
    auto curr_time = world->SimTime();
    if (calc_dt(last_update, curr_time) < (1.0 / update_rate)) {
        return;
    }
    last_update = curr_time;

    ignition::math::Pose3d car_pose = car_link->WorldPose();
    driverless_msgs::msg::ConeDetectionStamped ground_truth_track =
        get_ground_truth_track(track_model, curr_time, _ros_node->get_logger());

    if (has_subscribers(ground_truth_pub)) {
        auto centered_ground_truth = get_track_centered_on_car_inital_pose(car_inital_pose, ground_truth_track);
        ground_truth_pub->publish(centered_ground_truth);
    }

    if (has_subscribers(lidar_detection_pub)) {
        auto lidar_detection = get_sensor_detection(lidar_config, car_pose, ground_truth_track);
        lidar_detection_pub->publish(lidar_detection);
    }

    if (has_subscribers(vision_detection_pub)) {
        auto vision_detection = get_sensor_detection(camera_config, car_pose, ground_truth_track);
        vision_detection_pub->publish(vision_detection);
    }

    if (has_subscribers(slam_global_pub) || has_subscribers(slam_local_pub)) {
        auto noisy_slam_ground_truth_track = get_noisy_slam_ground_truth_track(slam_config, ground_truth_track);

        if (has_subscribers(slam_global_pub)) {
            auto slam_global_map =
                get_track_centered_on_car_inital_pose(car_inital_pose, noisy_slam_ground_truth_track);
            slam_global_pub->publish(slam_global_map);
        }

        if (has_subscribers(slam_local_pub)) {
            auto slam_local_map = get_slam_local_map(slam_config, car_pose, noisy_slam_ground_truth_track);
            slam_local_pub->publish(slam_local_map);
        }
    }
}

// Resets the position of cones to initial track model
bool ConeDetectionPlugin::resetConePosition(std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                            std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    (void)request;   // suppress unused parameter warning
    (void)response;  // suppress unused parameter warning

    gazebo::physics::Link_V links = track_model->GetLinks();

    // Loop through all cones
    for (unsigned int i = 0; i < links.size(); i++) {
        driverless_msgs::msg::ConeWithCovariance cone = initial_track.cones_with_cov[i];

        // Initial position and velocity variables
        const ignition::math::Pose3d pos(cone.cone.location.x, cone.cone.location.y, cone.cone.location.z, 0.0, 0.0, 0.0);
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
