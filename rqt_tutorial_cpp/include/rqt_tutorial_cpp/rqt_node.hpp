#ifndef RQT_TUTORIAL_CPP__RQT_NODE_HPP_
#define RQT_TUTORIAL_CPP__RQT_NODE_HPP_

#include <QStringListModel>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "driverless_msgs/msg/can.hpp"
#include "CAN_DVL.h"
#include "CAN_RES.h"
#include "CAN_SW.h"
#include "CAN_VCU.h"
#include "QUTMS_can.h"
// #include "can_interface.hpp"

enum AS_STATES {
    CAR_OFF = 0x00,
    ESTOP = 0x01,
    LV_ON = 0x02,
    AS_ON = 0x03,
    MISSION_SELECTED = 0x04,
    MISSION_CONFIRMED = 0x05,
    SWITCH_UP = 0x06,
    R2D_CHECKS = 0x07,
    R2D = 0x08,
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

namespace rqt_tutorial_cpp {
class RQTNode : public rclcpp::Node {
   public:
    RQTNode();
    virtual ~RQTNode();

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
    bool switch_up = false;
    bool r2d_pressed = false;
    bool estopped = true;

   private:
    Car_State_t car_state;
    int count = 0;

    rclcpp::Publisher<driverless_msgs::msg::Can>::SharedPtr can_pub_;

    // state machine timer
    rclcpp::TimerBase::SharedPtr state_machine_timer_;
    void state_machine_timer_callback();

    // VCU EBS heartbeat publisher
    rclcpp::TimerBase::SharedPtr vcu_ebs_timer_;
    void vcu_ebs_timer_callback();
    VCU_HeartbeatState_t EBS_VCU_heartbeat;

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

    void reset_states();
};
}  // namespace rqt_tutorial_cpp
#endif  // RQT_TUTORIAL_CPP__RQT_NODE_HPP_
