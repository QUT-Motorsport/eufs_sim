// Main Include
#include "gazebo_sbg_plugin/gazebo_ros_sbg.hpp"

// STD Include
#include <algorithm>
#include <fstream>
#include <mutex>   // NOLINT(build/c++11)
#include <thread>  // NOLINT(build/c++11)

namespace gazebo_plugins {
namespace eufs_plugins {

GZ_REGISTER_MODEL_PLUGIN(SBGPlugin)

SBGPlugin::SBGPlugin() {}

// Gazebo plugin functions

void SBGPlugin::Load(gazebo::sensors::SensorPtr _sensor, sdf::ElementPtr _sdf)
{
    _ros_node = gazebo_ros::Node::Get(_sdf);
    _world = _model->GetWorld();
}

}
}
