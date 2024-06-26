#ifndef STATE_CONTROL__STATE_NODE_HPP_
#define STATE_CONTROL__STATE_NODE_HPP_

#include <QStringListModel>
#include <string>

#include "CAN_DVL.h"
#include "CAN_EBS_CTRL.h"
#include "CAN_RES.h"
#include "CAN_SW.h"
#include "CAN_VCU.h"
#include "QUTMS_can.h"
#include "driverless_msgs/msg/can.hpp"
#include "driverless_msgs/msg/state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "std_srvs/srv/trigger.hpp"

enum AS_STATES {
    CAR_OFF = 0x00,
    ESTOP = 0x01,
    LV_ON = 0x02,
    AS_ON = 0x03,
    MISSION_SELECTED = 0x04,
    MISSION_CONFIRMED = 0x05,
    EBS_CHECKS = 0x06,
    WAIT_R2D = 0x07,
    R2D = 0x08,
    DRIVING = 0x09,
};

enum TS_STATES {
    TS_OFF = 0x00,
    TS_ON = 0x01,
    SDC_CLOSED = 0x02,
    TS_ACTIVE = 0x03,
};

typedef struct {
    uint8_t AS_state;
    uint8_t TS_state;
} Car_State_t;

namespace state_control {
class StateNode : public rclcpp::Node {
   public:
    StateNode();
    virtual ~StateNode();

    Car_State_t get_car_state();
    driverless_msgs::msg::State get_ros_state();

    // Car boot sequence
    bool LV_key_on = false;
    bool TS_key_on = false;
    bool AS_key_on = false;
    bool SDC_pressed = false;
    bool TS_pressed = false;

    // SW
    uint8_t selected_mission = DRIVERLESS_MISSIONS::MISSION_NONE;
    bool mission_pressed = false;

    // RES
    bool res_booted = false;
    bool switch_up = false;
    bool r2d_pressed = false;
    bool estop_pressed = true;
    bool reset_pressed = false;

    // reset trigger clients
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr reset_car_pos_srv_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr reset_cones_srv_;

    // publish simulated laps
    void pub_lap_count(int lap_count);

   private:
    // states
    Car_State_t car_state;
    Car_State_t prev_car_state;
    driverless_msgs::msg::State state_msg;
    int count = 0;

    // state sub
    rclcpp::Subscription<driverless_msgs::msg::State>::SharedPtr state_sub_;
    void as_state_callback(const driverless_msgs::msg::State::SharedPtr msg);

    // CAN pub
    rclcpp::Publisher<driverless_msgs::msg::Can>::SharedPtr can_pub_;
    rclcpp::Publisher<driverless_msgs::msg::Can>::SharedPtr canopen_pub_;

    // lap count pub
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr lap_counter_pub_;

    // steering ready
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr steering_ready_pub_;
    bool steering_ready = false;

    // state machine timer
    rclcpp::TimerBase::SharedPtr state_machine_timer_;
    void state_machine_timer_callback();

    // VCU EBS heartbeat publisher
    rclcpp::TimerBase::SharedPtr ebs_ctrl_timer_;
    void ebs_timer_callback();
    EBS_CTRL_HeartbeatState_t EBS_CTRL_heartbeat;

    // SW heartbeat publisher
    rclcpp::TimerBase::SharedPtr sw_timer_;
    void sw_timer_callback();
    SW_HeartbeatState_t SW_heartbeat;

    // RES boot publisher
    rclcpp::TimerBase::SharedPtr res_boot_timer_;
    void res_boot_call();

    // RES heartbeat publisher
    rclcpp::TimerBase::SharedPtr res_heartbeat_timer_;
    void res_heartbeat_timer_callback();
    RES_Status_t RES_status;

    // resets
    void reset_states();
    void reset_car();
    // void reset_car_pos();
    // void reset_cones();
};
}  // namespace state_control
#endif  // STATE_CONTROL__STATE_NODE_HPP_
