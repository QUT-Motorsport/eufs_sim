// Copyright (c) 2019 Edinburgh University Formula Student (EUFS)
#include <gazebo/common/Plugin.hh>
#include <ros/ros.h>

namespace gazebo
{

class WorldPluginTutorial: public WorldPlugin
{
  public:
    WorldPluginTutorial(): WorldPlugin() {}

    void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
    {
      // Make sure ROS node for Gazebo is init.

      if (!ros::isInitialized())
      {
        ROS_FATAL_STREAM("Ros ain't initialized, so it can't load simple_world_plugin.  " +
                         "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package.");
        return;
      }

      ROS_INFO("Hello World!  This is deprecated, I think.");
    }
};
GZ_REGISTER_WORLD_PLUGIN(WorldPluginTutorial);
}  // namespace gazebo
