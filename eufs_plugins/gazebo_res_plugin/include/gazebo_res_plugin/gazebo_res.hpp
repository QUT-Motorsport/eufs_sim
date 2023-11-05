#ifndef EUFS_PLUGINS_GAZEBO_RES_PLUGIN_INCLUDE_GAZEBO_RES_PLUGIN_GAZEBO_RES_HPP_
#define EUFS_PLUGINS_GAZEBO_RES_PLUGIN_INCLUDE_GAZEBO_RES_PLUGIN_GAZEBO_RES_HPP_

#include <memory>
#include <queue>
#include <string>
#include <vector>
// ROS Includes
#include "rclcpp/rclcpp.hpp"

// Gazebo Includes
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo_ros/node.hpp>

// Local includes
#include "helpers_gazebo.hpp"
#include "helpers_ros.hpp"

namespace gazebo_plugins {
namespace eufs_plugins {

class RESPlugin : public gazebo::ModelPlugin {
   public:
    RESPlugin();

    ~RESPlugin() override;

    void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override;

   private:
    void update();

    std::shared_ptr<rclcpp::Node> _rosnode;

    // Gazebo
    gazebo::physics::WorldPtr _world;
    gazebo::physics::ModelPtr _model;
    gazebo::event::ConnectionPtr _update_connection;
    gazebo::common::Time _last_sim_time;

    // Rate to publish ros messages
    double _update_rate = 100;

    int counter = 0;
};

}  // namespace eufs_plugins
}  // namespace gazebo_plugins

#endif  // EUFS_PLUGINS_GAZEBO_RES_PLUGIN_INCLUDE_GAZEBO_RES_PLUGIN_GAZEBO_ROS_RES_HPP_
