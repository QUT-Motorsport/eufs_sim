# eufs_racecar
This package contains robot [meshes](./meshes), [materials](./materials) and [urdf](./urdf) files.

There is a tutorial on the wiki describing [How To Change The Car Model](https://gitlab.com/eufs/eufs_sim/-/wikis/How-To-Change-The-Car-Model).

This also contains the launch file which loads the car into eufs_sim (also launches [RQT GUI's](../eufs_rqt/README.md) and [RViz](http://wiki.ros.org/rviz)).

## ROS 2 Nodes
This package doesn't contain any source code for ROS 2 nodes. However, it is responsible for launching a series of third-party nodes in [load_car.launch.py](./launch/load_car.launch.py).

## Launch Parameters

| Name | Default | Description |
| ---- | ------- | ----------- |
| launch_group       | default        | If `perception`, enables the camera and lidar plugins. Disabled if `no_perception`. In addition, if `perception`, `true` is passed to the `simulate_perception` parameter for [gazebo_ros_race_car_model](../eufs_plugins/gazebo_race_car_model/src/gazebo_ros_race_car_model.cpp). `false` is passed otherwise. |
| rviz               | false          | Determines whether or not to launch [RViz](http://wiki.ros.org/rviz). |
| show_rqt_gui       | true           | Determines whether or not to launch [MissionControlGUI and RobotSteeringGUI](../eufs_rqt/README.md). |
| namespace          | eufs           | The name of the robot displayed in Gazebo. |
| robot_name         | eufs           | The robot name (used to find the correct directory of robot.urdf.xacro in [robots](./robots)). |
| vehicleModel       | DynamicBicycle | The [vehicle model sub-class](../eufs_models/src) to use in the [gazebo_ros_race_car_model](../eufs_plugins/gazebo_race_car_model/src/gazebo_ros_race_car_model.cpp). |
| commandMode        | acceleration   | Determines whether the [RobotSteeringGUI](../eufs_rqt/src/eufs_rqt/EUFSRobotSteeringGUI.py) controls vehicle acceleration or velocity. This also gets passed to the [gazebo_ros_race_car_model](../eufs_plugins/gazebo_race_car_model/src/gazebo_ros_race_car_model.cpp). |
| vehicleModelConfig | configDry.yaml | Determines the yaml config file (found in [robots](./robots)/`robot_name`) to pass into the [vehicle model](../eufs_models/README.md). |
| publish_gt_tf      | false          | Whether the [gazebo_ros_race_car_model](../eufs_plugins/gazebo_race_car_model/src/gazebo_ros_race_car_model.cpp) should publish the ground truth tf. |
| x                  | 0              | Initial x position of vehicle. |
| y                  | 0              | Initial y position of vehicle. |
| z                  | 0              | Initial z position of vehicle. |
| roll               | 0              | Initial roll of vehicle. |
| pitch              | 0              | Initial pitch of vehicle. |
| yaw                | 0              | Initial yaw of vehicle. |
