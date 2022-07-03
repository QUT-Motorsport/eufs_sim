# eufs_plugins
This file provides information for developers looking to contribute to eufs_plugins and contains a general template for a Gazebo plugin.

You can read more on Gazebo plugins [here](https://gazebosim.org/tutorials?cat=write_plugin).

# Structure of eufs_plugins
The following is a minimal file structure for a Gazebo plugin. <gazebo_test_plugin> being your own plugin.
```
├── CMakeLists.txt
├── gazebo_test_plugin
│   ├── CMakeLists.txt
│   ├── include
│   │   └── gazebo_test_plugin
│   │       └── gazebo_test_plugin.hpp
│   └── src
│       └── gazebo_test_plugin.cpp
├── package.xml
├── README.md
└── urdf
    └── eufs_plugins.gazebo.xacro
```

# Steps to produce


### Creating your plugin

Depending on your goal, your plugin should inherit from either:

- Model Plugin
- System Plugin
- World Plugin

In order to understand more on the capabilities of each plugin, you should refer to the [official website of gazebo](https://gazebosim.org/tutorials?cat=write_plugin).

For the purpose of this eufs_plugins setup, we will create a minimal model plugin that outputs "I love EUFS" to the terminal.

Inside the `src` folder of your `<eufs_plugin_package>` is where you add your C++ script:

```c++
#include "gazebo_test_plugin/gazebo_test_plugin.hpp"

namespace gazebo_plugins {
namespace eufs_plugins {

GZ_REGISTER_MODEL_PLUGIN(GazeboTestPlugin)

GazeboTestPlugin::GazeboTestPlugin() {

}

// Gazebo plugin functions
void GazeboTestPlugin::Load(gazebo::physics::ModelPtr _parent, sdf::ElementPtr _sdf) {
  
  this->model = _parent;

  this->update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
    std::bind(&GazeboTestPlugin::OnUpdate, this));
}  // GazeboConeGroundTruth

void GazeboTestPlugin::OnUpdate() {
    std::cout << "TestPlugin Loaded..." << std::endl;
    std::cout << "I Love EUFS!" <<  std::endl;
}

}  // namespace eufs_plugins
}  // namespace gazebo_plugins
```

Inside the `include` folder of your `<eufs_plugin_package>`, you will add its header file:

```c++
#ifndef EUFS_PLUGINS_GAZEBO_TEST_PLUGIN_INCLUDE_GAZEBO_TEST_PLUGIN_GAZEBO_TEST_PLUGIN_HPP_
#define EUFS_PLUGINS_GAZEBO_TEST_PLUGIN_INCLUDE_GAZEBO_TEST_PLUGIN_GAZEBO_TEST_PLUGIN_HPP_

#include "rclcpp/rclcpp.hpp"

// Gazebo header
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
#include <gazebo_ros/node.hpp>

namespace gazebo_plugins {
namespace eufs_plugins {

class GazeboTestPlugin : public gazebo::ModelPlugin {
 public:
  GazeboTestPlugin();

  // Gazebo plugin functions
  void Load(gazebo::physics::ModelPtr _parent, sdf::ElementPtr _sdf);

  void OnUpdate();

 private:
  // Required ROS gazebo plugin variables
  gazebo::physics::WorldPtr _world;
  gazebo::event::ConnectionPtr update_connection_;
  gazebo_ros::Node::SharedPtr rosnode_;
  gazebo::physics::ModelPtr model;

};

}  // namespace eufs_plugins
}  // namespace gazebo_plugins

#endif  // EUFS_PLUGINS_GAZEBO_TEST_PLUGIN_INCLUDE_GAZEBO_TEST_PLUGIN_GAZEBO_TEST_PLUGIN_HPP_
```

That is it for the script! 

### Editing your plugin's CMakeLists.txt

The minimum content that you should add inside your `CMakeLists.txt` will look something like this

```cmake
cmake_minimum_required(VERSION 3.5)

add_library(gazebo_test_plugin SHARED
  src/gazebo_test_plugin.cpp
)

find_package(gazebo_dev REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(rclcpp REQUIRED)

ament_target_dependencies(gazebo_test_plugin
  "gazebo_dev"
  "gazebo_ros"
  "rclcpp"
)

target_include_directories(gazebo_test_plugin PUBLIC include)

ament_export_libraries(gazebo_test_plugin)

install(TARGETS gazebo_test_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib)
```
You should add the dependencies of your plugins here. For more info on this, you should refer to other plugins. 

### Editing eufs_plugins/CMakeLists.txt

Another important step is to include your subdirectory into eufs_plugin's `CMakelists.txt` 

```cmake
.
.
.
# Add your plugin subdirectory here
add_subdirectory(gazebo_cone_ground_truth)
add_subdirectory(gazebo_race_car_model)
add_subdirectory(gazebo_test_plugin)
.
.
.
```

### Editing urdf/eufs_plugins.gazebo.xacro

`eufs_plugins.gazebo.xacro` gives Gazebo the name of your plugin. It is also where you provide the values of the parameters passed to your plugin as an sdf file. It is effectively a config file for your plugin. Without adding your plugin to this file, Gazebo will not be able to find your plugin!

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:arg name="publish_tf" default="false"/>
  <xacro:arg name="simulate_perception" default="false"/>
  <xacro:arg name="command_mode" default="acceleration"/>
  .
  .
  .
  .
  .
  <!--Add your plugin here-->
  <gazebo>
    <plugin name="test_plugin" filename="libgazebo_test_plugin.so"/>
      <example_param_1>4.0</example_param_1>
      <example_param_2>Hello</example_param_2>
      .
      .
      .
      <example_param_n>$(arg cool_variable)</example_param_n>
    </plugin>
  </gazebo>

</robot>
```

That is all you need to get your plugin up and running!

##### Remember to always build your package when you make your changes and source it afterwards.

### Run Simulation!

Once, you run the simulation, you should get the following output:

```shell
[eufs_launcher-3] [INFO] [spawn_entity.py-4]: process has finished cleanly [pid 9374]
[eufs_launcher-3] [gzserver-1] [INFO] [1650393499.597843012] [race_car]: RaceCarModelPlugin Loaded
[eufs_launcher-3] [gzserver-1] [INFO] [1650393499.611663091] [cone_ground_truth]: ConeGroundTruthPlugin Loaded
[eufs_launcher-3] [gzserver-1] TestPlugin Loaded...
[eufs_launcher-3] [gzserver-1] I Love EUFS!
[eufs_launcher-3] [gzserver-1] TestPlugin Loaded...
[eufs_launcher-3] [gzserver-1] I Love EUFS!
[eufs_launcher-3] [gzserver-1] TestPlugin Loaded...
[eufs_launcher-3] [gzserver-1] I Love EUFS!
[eufs_launcher-3] [gzserver-1] TestPlugin Loaded...
```

That is all you need! Good luck with your plugin!