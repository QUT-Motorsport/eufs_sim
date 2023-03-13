#include "gazebo_cone_detection_plugin/gazebo_ros_cone_detection.hpp"

#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Dense"

#include "gazebo_helpers.hpp"
#include "track_helpers.hpp"

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

    ground_truth_pub = ros_node->create_publisher<driverless_msgs::msg::ConeDetectionStamped>(("ground_truth/global_map"), 1);

    last_update = world->SimTime();
    update_connection = gazebo::event::Events::ConnectWorldUpdateBegin(std::bind(&ConeDetectionPlugin::UpdateChild, this));
}

void ConeDetectionPlugin::UpdateChild() {
    auto curr_time = world->SimTime();
    if (calc_dt(last_update, curr_time) < (1.0 / update_rate)) {
        return;
    }
    last_update = curr_time;
    
    driverless_msgs::msg::ConeDetectionStamped ground_truth_track = get_ground_truth_track(track_model, curr_time, ros_node->get_logger());
    ground_truth_pub->publish(ground_truth_track);        
}

}  // namespace eufs_plugins
}  // namespace gazebo_plugins
