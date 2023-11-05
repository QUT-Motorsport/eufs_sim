#ifndef VEHICLE_EMULATION__STATE_NODE_HPP_
#define VEHICLE_EMULATION__STATE_NODE_HPP_

#include <QStringListModel>
#include <string>
#include <memory>
#include <sstream>
#include <sys/ipc.h>
#include <sys/shm.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace vehicle_emulation {
class InputProcessingNode : public rclcpp::Node {
   public:
    InputProcessingNode();
    virtual ~InputProcessingNode();

    // Car boot sequence
    bool LV_key_on = false;
    bool TS_key_on = false;
    bool AS_key_on = false;
    bool SDC_pressed = false;
    bool TS_pressed = false;

    // SW
    uint8_t selected_mission = 0;
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
    void lv_key_callback(bool value);
    void ts_key_callback(bool value);
    void as_key_callback(bool value);
    void sdc_callback(bool value);
    void ts_callback(bool value);
    void mission_callback(u_int8_t value);
    void mission_pressed_callback(bool value);
    void r2d_callback(bool value);
    void estop_callback(bool value);
    void switch_up_callback(bool value);

    void pub_lap_count(int lap_count);
    void reset_states();

   private:
    // states
    int count = 0;

    // switches and key state pubs
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr lv_key_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr ts_key_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr as_key_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr sdc_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr prechrg_pub_;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr mission_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr mission_pressed_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr r2d_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr estop_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr res_switch_pub_;

    // lap count pub
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr lap_counter_pub_;

    // // shm keys for each switch and key state
    // key_t lv_key_shm_key_ = 1000;
    // key_t ts_key_shm_key_ = 1001;
    // key_t as_key_shm_key_ = 1002;
    // key_t sdc_shm_key_ = 1003;
    // key_t prechrg_shm_key_ = 1004;
    // key_t mission_shm_key_ = 1005;
    // key_t mission_pressed_shm_key_ = 1006;
    // key_t r2d_shm_key_ = 1007;
    // key_t estop_shm_key_ = 1008;
    // key_t res_switch_shm_key_ = 1009;

    // int shmFlags = IPC_CREAT | (S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP|S_IROTH|S_IWOTH);
    // size_t shm_size_ = sizeof(uint8_t);
};
}  // namespace vehicle_emulation
#endif  // VEHICLE_EMULATION__STATE_NODE_HPP_
