#ifndef EUFS_PLUGINS_GAZEBO_EMBEDDED_PLUGINS_INCLUDE_GAZEBO_EMBEDDED_PLUGINS_GAZEBO_SW_HPP_
#define EUFS_PLUGINS_GAZEBO_EMBEDDED_PLUGINS_INCLUDE_GAZEBO_EMBEDDED_PLUGINS_GAZEBO_SW_HPP_

#include <memory>
#include <queue>
#include <string>
#include <vector>

#include <algorithm>
#include <fstream>
#include <mutex>   // NOLINT(build/c++11)
#include <thread>  // NOLINT(build/c++11)

#include "rclcpp/rclcpp.hpp"

// Gazebo Includes
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo_ros/node.hpp>

// ROS Includes
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/u_int8.hpp"

// Local includes
#include "helpers_gazebo.hpp"
#include "helpers_ros.hpp"
#include "CAN_SW.h"
#include "CAN_DVL.h"
#include "QUTMS_can.h"

#include "SocketCAN.hpp"


namespace gazebo_plugins {
namespace eufs_plugins {

class SteeringWheelPlugin : public gazebo::ModelPlugin {
   public:
    SteeringWheelPlugin();

    ~SteeringWheelPlugin() override;

    void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override;
    
    std::string can_interface_ = "vcan0";
    std::shared_ptr<SocketCAN> socketCAN_;

   private:
    void update();
    void lv_key_callback(const std_msgs::msg::Bool::SharedPtr msg);
    void mission_select_callback(const std_msgs::msg::UInt8::SharedPtr msg);
    void mission_confirm_callback(const std_msgs::msg::Bool::SharedPtr msg);

    std::shared_ptr<rclcpp::Node> rosnode_;

    // Gazebo
    gazebo::physics::WorldPtr world_;
    gazebo::physics::ModelPtr model_;
    gazebo::event::ConnectionPtr update_connection_;
    gazebo::common::Time last_sim_time_;
    int update_rate_ = 100;

    // gui subscribers
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr lv_sub_;
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr mission_select_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr mission_confirm_sub_;

    // states
    bool LV_key_on = false;
    int selected_mission = 0;
    bool mission_pressed = false;
};

}  // namespace eufs_plugins
}  // namespace gazebo_plugins

#endif  // EUFS_PLUGINS_GAZEBO_EMBEDDED_PLUGINS_INCLUDE_GAZEBO_EMBEDDED_PLUGINS_GAZEBO_ROS_SW_HPP_
