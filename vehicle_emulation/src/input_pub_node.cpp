#include "vehicle_emulation/input_pub_node.hpp"

#include <memory>
#include <sstream>
#include <string>

#include "rclcpp/rclcpp.hpp"

using namespace std::placeholders;
using namespace std::chrono_literals;

namespace vehicle_emulation {
InputProcessingNode::InputProcessingNode() : Node("input_processing_node") {
    // Force flush of the stdout buffer
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);

    // lap counter pub
    lap_counter_pub_ = this->create_publisher<std_msgs::msg::UInt8>("/system/laps_completed", 10);

    // reset trigger clients
    reset_car_pos_srv_ = this->create_client<std_srvs::srv::Trigger>("/system/reset_car_pos");
    while (!this->reset_car_pos_srv_->wait_for_service(1s)) {
        RCLCPP_INFO(this->get_logger(), "Waiting for reset_car_pos service to be available...");
    }

    reset_cones_srv_ = this->create_client<std_srvs::srv::Trigger>("/system/reset_cones");
    while (!this->reset_cones_srv_->wait_for_service(1s)) {
        RCLCPP_INFO(this->get_logger(), "Waiting for reset_cones service to be available...");
    }

    // switches and key state pubs
    lv_key_pub_ = this->create_publisher<std_msgs::msg::Bool>("/sim_control/lv_key_state", 10);
    ts_key_pub_ = this->create_publisher<std_msgs::msg::Bool>("/sim_control/ts_key_state", 10);
    as_key_pub_ = this->create_publisher<std_msgs::msg::Bool>("/sim_control/as_key_state", 10);
    sdc_pub_ = this->create_publisher<std_msgs::msg::Bool>("/sim_control/shdn_btn_state", 10);
    prechrg_pub_ = this->create_publisher<std_msgs::msg::Bool>("/sim_control/prechrg_btn_state", 10);
    mission_pub_ = this->create_publisher<std_msgs::msg::UInt8>("/sim_control/selected_mission", 10);
    mission_pressed_pub_ = this->create_publisher<std_msgs::msg::Bool>("/sim_control/mission_btn_state", 10);
    r2d_pub_ = this->create_publisher<std_msgs::msg::Bool>("/sim_control/r2d_btn_state", 10);
    estop_pub_ = this->create_publisher<std_msgs::msg::Bool>("/sim_control/estop_btn_state", 10);
    res_switch_pub_ = this->create_publisher<std_msgs::msg::Bool>("/sim_control/switch_state", 10);

    RCLCPP_INFO(this->get_logger(), "---RQT node initialised---");
}

InputProcessingNode::~InputProcessingNode() { RCLCPP_INFO(this->get_logger(), "Terminated RQT node"); }

void InputProcessingNode::lv_key_callback(bool value) {
    std::cout << "LV key state: " << value << std::endl;
    LV_key_on = value;

    std_msgs::msg::Bool lv_key_msg;
    lv_key_msg.data = LV_key_on;
    lv_key_pub_->publish(lv_key_msg);
}

void InputProcessingNode::ts_key_callback(bool value) {
    std::cout << "TS key state: " << value << std::endl;
    TS_key_on = value;

    std_msgs::msg::Bool ts_key_msg;
    ts_key_msg.data = TS_key_on;
    ts_key_pub_->publish(ts_key_msg);
}

void InputProcessingNode::as_key_callback(bool value) {
    std::cout << "AS key state: " << value << std::endl;
    AS_key_on = value;

    std_msgs::msg::Bool as_key_msg;
    as_key_msg.data = AS_key_on;
    as_key_pub_->publish(as_key_msg);
}

void InputProcessingNode::sdc_callback(bool value) {
    std::cout << "SDC btn state: " << value << std::endl;
    SDC_pressed = value;

    std_msgs::msg::Bool sdc_msg;
    sdc_msg.data = SDC_pressed;
    sdc_pub_->publish(sdc_msg);

    SDC_pressed = false;
}

void InputProcessingNode::ts_callback(bool value) {
    std::cout << "TS btn state: " << value << std::endl;
    TS_pressed = value;

    std_msgs::msg::Bool ts_msg;
    ts_msg.data = TS_pressed;
    prechrg_pub_->publish(ts_msg);

    TS_pressed = false;
}

void InputProcessingNode::mission_callback(int value) {
    std::cout << "Mission select: " << value << std::endl;
    selected_mission = value;

    std_msgs::msg::UInt8 mission_msg;
    mission_msg.data = selected_mission;
    mission_pub_->publish(mission_msg);
}

void InputProcessingNode::mission_pressed_callback(bool value) {
    std::cout << "Mission btn state: " << value << std::endl;
    mission_pressed = value;

    std_msgs::msg::Bool mission_pressed_msg;
    mission_pressed_msg.data = mission_pressed;
    mission_pressed_pub_->publish(mission_pressed_msg);
}

void InputProcessingNode::r2d_callback(bool value) {
    std::cout << "R2D btn state: " << value << std::endl;
    r2d_pressed = value;

    std_msgs::msg::Bool r2d_msg;
    r2d_msg.data = r2d_pressed;
    r2d_pub_->publish(r2d_msg);

    r2d_pressed = false;
}

void InputProcessingNode::estop_callback(bool value) {
    std::cout << "Estop btn state: " << value << std::endl;
    estop_pressed = value;

    std_msgs::msg::Bool estop_msg;
    estop_msg.data = estop_pressed;
    estop_pub_->publish(estop_msg);
}

void InputProcessingNode::switch_up_callback(bool value) {
    std::cout << "RES switch state: " << value << std::endl;
    switch_up = value;

    std_msgs::msg::Bool switch_up_msg;
    switch_up_msg.data = switch_up;
    res_switch_pub_->publish(switch_up_msg);
}

void InputProcessingNode::reset_states() {
    // car boot sequence
    LV_key_on = false;
    TS_key_on = false;
    AS_key_on = false;

    // SW
    selected_mission = 0;
    mission_pressed = false;

    // RES
    switch_up = false;
    estop_pressed = true;
    res_booted = false;

    // car signals
    pub_lap_count(0);

    // sim world
    reset_pressed = false;
}

void InputProcessingNode::pub_lap_count(int lap_count) {
    std_msgs::msg::UInt8 lap_count_msg;
    lap_count_msg.data = lap_count;
    lap_counter_pub_->publish(lap_count_msg);
}

}  // namespace vehicle_emulation
