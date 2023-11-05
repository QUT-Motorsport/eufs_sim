#include "gazebo_embedded_plugins/gazebo_sw.hpp"

namespace gazebo_plugins {
namespace eufs_plugins {

SteeringWheelPlugin::SteeringWheelPlugin() {}

SteeringWheelPlugin::~SteeringWheelPlugin() { 
    this->socketCAN_->~SocketCAN();
    update_connection_.reset(); 
}

void SteeringWheelPlugin::Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) {
    rosnode_ = gazebo_ros::Node::Get(sdf);

    RCLCPP_DEBUG(rosnode_->get_logger(), "Loading Steering Wheel Plugin");

    model_ = model;
    world_ = model_->GetWorld();

    // socketcan setup
    this->socketCAN_ = std::make_shared<SocketCAN>();
    if (!this->socketCAN_->setup(can_interface_)) {
        RCLCPP_ERROR(rosnode_->get_logger(), "Failed to create connection on %s. Retrying...", can_interface_.c_str());
    }

    // subscribers
    lv_sub_ = this->rosnode_->create_subscription<std_msgs::msg::Bool>(
        "/sim_control/lv_key_state", 10, std::bind(&SteeringWheelPlugin::lv_key_callback, this, std::placeholders::_1));
    mission_select_sub_ = this->rosnode_->create_subscription<std_msgs::msg::UInt8>(
        "/sim_control/selected_mission", 10, std::bind(&SteeringWheelPlugin::mission_select_callback, this, std::placeholders::_1));
    mission_confirm_sub_ = this->rosnode_->create_subscription<std_msgs::msg::Bool>(
        "/sim_control/mission_btn_state", 10, std::bind(&SteeringWheelPlugin::mission_confirm_callback, this, std::placeholders::_1));

    // Connect to Gazebo
    update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(std::bind(&SteeringWheelPlugin::update, this));
    last_sim_time_ = world_->SimTime();

    RCLCPP_INFO(rosnode_->get_logger(), "Steering Wheel Plugin Loaded");
}

void SteeringWheelPlugin::lv_key_callback(const std_msgs::msg::Bool::SharedPtr msg) {
    LV_key_on = msg->data;
}

void SteeringWheelPlugin::mission_select_callback(const std_msgs::msg::UInt8::SharedPtr msg) {
    selected_mission = msg->data;
}

void SteeringWheelPlugin::mission_confirm_callback(const std_msgs::msg::Bool::SharedPtr msg) {
    mission_pressed = msg->data;
}

void SteeringWheelPlugin::update() {
    // Check against update rate
    gazebo::common::Time curTime = world_->SimTime();
    double dt = calc_dt(last_sim_time_, curTime);
    if (dt < (1 / update_rate_)) {
        return;
    }

    if (!LV_key_on) {
        return;
    }

    // get CAN messages
    auto res = this->socketCAN_->rx();
    for (auto &msg : *res) {
        if (msg.ID == DVL_Heartbeat_ID) {
            DVL_HeartbeatState_t dvl_heartbeat;
            Parse_DVL_Heartbeat(msg.data, &dvl_heartbeat);
        }
    }

    // state machine
    // switch ()

    // update last values
    last_sim_time_ = curTime;
}

GZ_REGISTER_MODEL_PLUGIN(SteeringWheelPlugin)

}  // namespace eufs_plugins
}  // namespace gazebo_plugins
