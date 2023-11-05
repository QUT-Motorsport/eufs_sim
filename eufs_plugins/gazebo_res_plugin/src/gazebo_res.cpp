#include "gazebo_res_plugin/gazebo_res.hpp"

#include <algorithm>
#include <fstream>
#include <mutex>   // NOLINT(build/c++11)
#include <thread>  // NOLINT(build/c++11)

namespace gazebo_plugins {
namespace eufs_plugins {

RESPlugin::RESPlugin() {}

RESPlugin::~RESPlugin() { _update_connection.reset(); }

void RESPlugin::Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) {
    _rosnode = gazebo_ros::Node::Get(sdf);

    RCLCPP_DEBUG(_rosnode->get_logger(), "Loading RES Plugin");

    _model = model;
    _world = _model->GetWorld();

    // Connect to Gazebo
    _update_connection = gazebo::event::Events::ConnectWorldUpdateBegin(std::bind(&RESPlugin::update, this));
    _last_sim_time = _world->SimTime();

    RCLCPP_INFO(_rosnode->get_logger(), "RES Plugin Loaded");
}

void RESPlugin::update() {
    // Check against update rate
    gazebo::common::Time curTime = _world->SimTime();
    double dt = calc_dt(_last_sim_time, curTime);
    if (dt < (1 / _update_rate)) {
        return;
    }


    // update last values
    _last_sim_time = curTime;
}

GZ_REGISTER_MODEL_PLUGIN(RESPlugin)

}  // namespace eufs_plugins
}  // namespace gazebo_plugins
