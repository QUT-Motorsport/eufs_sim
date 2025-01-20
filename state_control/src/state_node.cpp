#include "state_control/state_node.hpp"

#include <memory>
#include <sstream>
#include <string>

#include "rclcpp/rclcpp.hpp"

using namespace std::placeholders;
using namespace std::chrono_literals;

// void Parse_RES_Heartbeat(uint8_t *data, RES_Status_t *status) {
//     status->estop = !(data[0] & (1 << 0));                        // ESTOP = PDO 2000 Bit 0
//     status->sw_k2 = data[0] & (1 << 1);                           // K2 = PDO 2000 Bit 1
//     status->bt_k3 = data[0] & (1 << 2);                           // K3 = PDO 2000 Bit 2
//     status->radio_quality = data[6];                              // Radio Quality = PDO 2006
//     status->loss_of_signal_shutdown_notice = data[7] & (1 << 6);  // LoSSN = PDO 2007 Bi
// }
RES_Heartbeat_t Compose_RES_Heartbeat(RES_Status_t *state) {
    RES_Heartbeat_t msg;

    msg.id = RES_Heartbeat_ID;

    // shift the bits to the correct position
    uint8_t estop = !(state->estop) << 0;
    uint8_t sw_k2 = state->sw_k2 << 1;
    uint8_t bt_k3 = state->bt_k3 << 2;
    uint8_t radio_quality = state->radio_quality << 0;
    uint8_t loss_of_signal_shutdown_notice = state->loss_of_signal_shutdown_notice << 6;

    // combine the bits
    msg.data[0] = estop | sw_k2 | bt_k3;
    msg.data[6] = radio_quality;
    msg.data[7] = loss_of_signal_shutdown_notice;

    return msg;
}

VCU_Heartbeat_t Compose_VCU_Heartbeat(uint8_t id, VCU_HeartbeatState_t *state) {
    VCU_Heartbeat_t msg;
    msg.id = VCU_Heartbeat_ID | id;

    msg.data[0] = state->stateID;
    msg.data[1] = state->coreFlags.rawMem;
    msg.data[2] = (state->otherFlags.rawMem) & 0xFF;
    msg.data[3] = (state->otherFlags.rawMem >> 8) & 0xFF;
    msg.data[4] = (state->VCU) & 0xFF;
    msg.data[5] = (state->VCU >> 8) & 0xFF;

    return msg;
}

SW_Heartbeat_t Compose_SW_Heartbeat(SW_HeartbeatState_t *state) {
    SW_Heartbeat_t msg;

    msg.id = SW_Heartbeat_ID;

    msg.data[0] = state->stateID;
    msg.data[1] = state->missionID;
    msg.data[2] = (state->flags.rawMem) & 0xFF;
    msg.data[3] = (state->flags.rawMem >> 8) & 0xFF;

    return msg;
}

EBS_CTRL_Heartbeat_t Compose_EBS_CTRL_Heartbeat(EBS_CTRL_HeartbeatState_t *state) {
    EBS_CTRL_Heartbeat_t msg;
    msg.id = EBS_CTRL_Heartbeat_ID;

    msg.data[0] = state->stateID;
    msg.data[1] = (state->flags.rawMem & 0xFF);
    msg.data[2] = ((state->flags.rawMem >> 8) & 0xFF);

    return msg;
}

driverless_msgs::msg::Can _d_2_f(uint32_t id, bool is_extended, uint8_t *data, uint8_t dlc) {
    driverless_msgs::msg::Can frame;
    frame.id = id;
    frame.id_type = is_extended;
    std::vector<uint8_t> v;
    v.assign(data, data + dlc);
    frame.data = v;
    frame.dlc = dlc;
    return frame;
}

namespace state_control {
StateNode::StateNode() : Node("state_control") {
    // Force flush of the stdout buffer
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);

    // AS status subscriber
    this->state_sub_ = this->create_subscription<driverless_msgs::msg::State>(
        "/system/as_status", 1, std::bind(&StateNode::as_state_callback, this, _1));

    // CAN pub
    can_pub_ = this->create_publisher<driverless_msgs::msg::Can>("/can/canbus_rosbound", 10);
    canopen_pub_ = this->create_publisher<driverless_msgs::msg::Can>("/can/canopen_rosbound", 10);

    // lap counter pub
    lap_counter_pub_ = this->create_publisher<std_msgs::msg::UInt8>("/system/laps_completed", 10);
    steering_ready_pub_ = this->create_publisher<std_msgs::msg::Bool>("/system/steering_ready", 10);

    // reset trigger clients
    reset_car_pos_srv_ = this->create_client<std_srvs::srv::Trigger>("/system/reset_car_pos");
    while (!this->reset_car_pos_srv_->wait_for_service(1s)) {
        RCLCPP_INFO(this->get_logger(), "Waiting for reset_car_pos service to be available...");
    }

    reset_cones_srv_ = this->create_client<std_srvs::srv::Trigger>("/system/reset_cones");
    while (!this->reset_cones_srv_->wait_for_service(1s)) {
        RCLCPP_INFO(this->get_logger(), "Waiting for reset_cones service to be available...");
    }

    // VCU EBS heartbeat
    ebs_ctrl_timer_ = this->create_wall_timer(50ms, std::bind(&StateNode::ebs_timer_callback, this));
    // SW heartbeat
    sw_timer_ = this->create_wall_timer(100ms, std::bind(&StateNode::sw_timer_callback, this));
    // RES
    res_heartbeat_timer_ = this->create_wall_timer(30ms, std::bind(&StateNode::res_heartbeat_timer_callback, this));
    // State machine
    state_machine_timer_ = this->create_wall_timer(20ms, std::bind(&StateNode::state_machine_timer_callback, this));

    // set initial car structs
    reset_car();

    RCLCPP_INFO(this->get_logger(), "---RQT node initialised---");
}

StateNode::~StateNode() { RCLCPP_INFO(this->get_logger(), "Terminated RQT node"); }

Car_State_t StateNode::get_car_state() { return this->car_state; }

driverless_msgs::msg::State StateNode::get_ros_state() { return this->state_msg; }

void StateNode::as_state_callback(const driverless_msgs::msg::State::SharedPtr msg) { state_msg = *msg; }

void StateNode::ebs_timer_callback() {
    if (!LV_key_on) {
        return;
    }
    // compose CAN frame then convert to ROS 2 message
    EBS_CTRL_Heartbeat_t CAN_msg = Compose_EBS_CTRL_Heartbeat(&this->EBS_CTRL_heartbeat);
    driverless_msgs::msg::Can ROS_CAN_msg = _d_2_f(CAN_msg.id, false, CAN_msg.data, sizeof(CAN_msg.data));
    this->can_pub_->publish(ROS_CAN_msg);
}

void StateNode::sw_timer_callback() {
    if (!LV_key_on) {
        return;
    }
    // compose CAN frame then convert to ROS 2 message
    SW_Heartbeat_t CAN_msg = Compose_SW_Heartbeat(&this->SW_heartbeat);
    driverless_msgs::msg::Can ROS_CAN_msg = _d_2_f(CAN_msg.id, false, CAN_msg.data, sizeof(CAN_msg.data));
    this->can_pub_->publish(ROS_CAN_msg);
}

void StateNode::res_heartbeat_timer_callback() {
    if (!res_booted) {
        return;
    }
    // compose CAN frame then convert to ROS 2 message
    this->RES_status.loss_of_signal_shutdown_notice = 0;
    this->RES_status.radio_quality = 255;
    RES_Heartbeat_t CAN_msg = Compose_RES_Heartbeat(&this->RES_status);
    driverless_msgs::msg::Can ROS_CAN_msg = _d_2_f(CAN_msg.id, false, CAN_msg.data, sizeof(CAN_msg.data));
    this->canopen_pub_->publish(ROS_CAN_msg);

    this->RES_status.bt_k3 = false;
}

void StateNode::res_boot_call() {
    // compose CAN frame then convert to ROS 2 message
    uint8_t p[8] = {0x01, RES_NODE_ID, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    driverless_msgs::msg::Can ROS_CAN_msg = _d_2_f(0x700 + RES_NODE_ID, false, p, sizeof(p));
    this->canopen_pub_->publish(ROS_CAN_msg);
    res_booted = true;
}

void StateNode::state_machine_timer_callback() {
    if (reset_pressed) {
        reset_states();
    }

    // three keys must be turned on to start the car
    if (LV_key_on == false) {
        this->car_state.AS_state = AS_STATES::CAR_OFF;
        this->car_state.TS_state = TS_STATES::TS_OFF;
        res_booted = false;

        reset_car();
    }
    if (TS_key_on == false) {
        this->car_state.TS_state = TS_STATES::TS_OFF;
        // car may be running processes
        if (this->car_state.AS_state > AS_STATES::MISSION_SELECTED) {
            this->car_state.AS_state = AS_STATES::MISSION_SELECTED;
        }
    }
    if (AS_key_on == false) {
        if (this->car_state.AS_state > AS_STATES::LV_ON) {
            this->car_state.AS_state = AS_STATES::LV_ON;
        }
        if (this->car_state.TS_state > TS_STATES::SDC_CLOSED) {
            this->car_state.TS_state = TS_STATES::SDC_CLOSED;
        }
        this->EBS_CTRL_heartbeat.stateID = EBS_CTRL_STATE::EBS_CTRL_STATE_IDLE;
    }
    // big red button
    if (estop_pressed) {
        // TS key may still be on
        if (this->car_state.TS_state > TS_STATES::TS_ON) {
            this->car_state.TS_state = TS_STATES::TS_ON;
        }
        // if the car is off already
        if (this->car_state.AS_state > AS_STATES::ESTOP) {
            this->car_state.AS_state = AS_STATES::ESTOP;
        }

        this->RES_status.estop = true;
        this->EBS_CTRL_heartbeat.stateID = EBS_CTRL_STATE::EBS_CTRL_STATE_SHUTDOWN;
    } else {
        this->RES_status.estop = false;
    }

    if (switch_up) {
        this->RES_status.sw_k2 = true;
    } else {
        this->RES_status.sw_k2 = false;
    }

    // state transitions
    if (this->car_state.AS_state == AS_STATES::CAR_OFF) {
        if (LV_key_on) {
            this->car_state.AS_state = AS_STATES::LV_ON;
            // RES is powered on
            res_boot_call();
        }
    }
    // LV key allows TS and AS keys to be turned on
    if (this->car_state.AS_state >= AS_STATES::LV_ON) {
        if (TS_key_on && this->car_state.TS_state == TS_STATES::TS_OFF) {
            this->car_state.TS_state = TS_STATES::TS_ON;
        }
        if (AS_key_on && this->car_state.AS_state == AS_STATES::LV_ON) {
            this->car_state.AS_state = AS_STATES::AS_ON;
        }
    }
    if (this->car_state.TS_state == TS_ON) {
        if (SDC_pressed) {
            this->car_state.TS_state = TS_STATES::SDC_CLOSED;
        }
    }
    if (this->car_state.AS_state == AS_STATES::AS_ON) {
        if (selected_mission != DRIVERLESS_MISSIONS::MISSION_NONE) {
            this->car_state.AS_state = AS_STATES::MISSION_SELECTED;
            // mission is selected
            this->SW_heartbeat.missionID = selected_mission;
            this->SW_heartbeat.stateID = sw_state::SW_STATE_SELECT_MISSION;
        }
    }
    if (this->car_state.AS_state == AS_STATES::MISSION_SELECTED) {
        if (mission_pressed) {
            this->car_state.AS_state = AS_STATES::MISSION_CONFIRMED;
            // mission is confirmed
            this->SW_heartbeat.stateID = sw_state::SW_STATE_MISSION_ACK;
        }
    }
    // mission confirm button needs to be pressed before TS can be activated
    if (this->car_state.TS_state == TS_STATES::SDC_CLOSED && this->car_state.AS_state == AS_STATES::MISSION_CONFIRMED) {
        if (TS_pressed) {
            this->car_state.TS_state = TS_STATES::TS_ACTIVE;
        }
    }
    if (this->car_state.TS_state == TS_STATES::TS_ACTIVE) {
        if (switch_up && this->car_state.AS_state == AS_STATES::MISSION_CONFIRMED) {
            this->car_state.AS_state = AS_STATES::EBS_CHECKS;
        }
    }
    if (this->car_state.AS_state == AS_STATES::EBS_CHECKS) {
        // ebs is ready for checks
        this->EBS_CTRL_heartbeat.stateID = EBS_CTRL_STATE::EBS_CTRL_STATE_READY;
        // TODO: add checks
        // if (checks_pass)
        this->car_state.AS_state = AS_STATES::WAIT_R2D;
    }
    if (this->car_state.AS_state == AS_STATES::WAIT_R2D) {
        if (r2d_pressed) {
            this->car_state.AS_state = AS_STATES::R2D;
            // res button
            this->RES_status.bt_k3 = true;
            // EBS is armed
            this->EBS_CTRL_heartbeat.stateID = EBS_CTRL_STATE::EBS_CTRL_STATE_DRIVE;

            std_msgs::msg::Bool steering_ready_msg;
            steering_ready_msg.data = true;
            steering_ready_pub_->publish(steering_ready_msg);
        }
    }
    if (this->car_state.AS_state == AS_STATES::R2D) {
        this->EBS_CTRL_heartbeat.stateID = EBS_CTRL_STATE::EBS_CTRL_STATE_DRIVE;
        this->car_state.AS_state = AS_STATES::DRIVING;
    }
    // if (this->car_state.AS_state == AS_STATES::DRIVING) {
    // }

    // print if changes
    if (this->prev_car_state.AS_state != this->car_state.AS_state ||
        this->prev_car_state.TS_state != this->car_state.TS_state) {
        RCLCPP_INFO(this->get_logger(), "AS_state: %d, TS_state: %d", this->car_state.AS_state,
                    this->car_state.TS_state);
    }
    this->prev_car_state = this->car_state;
    // reset the one-time button presses
    SDC_pressed = false;
    r2d_pressed = false;
    TS_pressed = false;
    mission_pressed = false;
}

void StateNode::reset_states() {
    // car boot sequence
    LV_key_on = false;
    TS_key_on = false;
    AS_key_on = false;

    // SW
    selected_mission = DRIVERLESS_MISSIONS::MISSION_NONE;
    mission_pressed = false;

    // RES
    switch_up = false;
    estop_pressed = true;
    res_booted = false;

    this->steering_ready = false;
    std_msgs::msg::Bool steering_ready_msg;
    steering_ready_msg.data = false;
    steering_ready_pub_->publish(steering_ready_msg);

    // AS
    this->car_state.AS_state = AS_STATES::CAR_OFF;
    this->car_state.TS_state = TS_STATES::TS_OFF;

    // car signals
    reset_car();
    pub_lap_count(0);

    // sim world
    reset_pressed = false;

    // reset_car_pos();
    // reset_cones();
}

void StateNode::reset_car() {
    this->EBS_CTRL_heartbeat.stateID = EBS_CTRL_STATE::EBS_CTRL_STATE_START;
    this->SW_heartbeat.stateID = sw_state::SW_STATE_START;
    this->SW_heartbeat.missionID = DRIVERLESS_MISSIONS::MISSION_NONE;
    this->RES_status.estop = true;
    this->RES_status.sw_k2 = false;
    this->RES_status.bt_k3 = false;
}

void StateNode::pub_lap_count(int lap_count) {
    std_msgs::msg::UInt8 lap_count_msg;
    lap_count_msg.data = lap_count;
    lap_counter_pub_->publish(lap_count_msg);
}

// void StateNode::reset_car_pos() {
//     // Reset car position
//     auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
//     auto result = this->reset_car_pos_srv_->async_send_request(request);
//     RCLCPP_INFO(this->get_logger(), "Vehicle position reset successful: %d", result.get()->success);
// }

// void StateNode::reset_cones() {
//     // Reset cones
//     auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
//     auto result = this->reset_cones_srv_->async_send_request(request);
//     RCLCPP_INFO(this->get_logger(), "Cone position reset successful: %d", result.get()->success);
// }

}  // namespace state_control
