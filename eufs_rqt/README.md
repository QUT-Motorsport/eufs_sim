# eufs_rqt

Contains the GUI's for controlling the vehicle.
- [MissionControlGUI](./src/eufs_rqt/MissionControlGUI.py) - controls the [state_machine](../eufs_plugins/gazebo_race_car_model/src/state_machine.cpp). 
- [RobotSteeringGUI](./src/eufs_rqt/EUFSRobotSteeringGUI.py) - controls the car using [ackermann_msgs/AckermannDriveStamped](http://docs.ros.org/en/jade/api/ackermann_msgs/html/msg/AckermannDriveStamped.html) messages (a fork of [rqt_robot_steering](https://github.com/ros-visualization/rqt_robot_steering)).

For a basic usage guide see [How To Launch eufs_sim](https://gitlab.com/eufs/eufs_sim/-/wikis/Simulation/How-To-Launch-eufs_sim).

## Robot Steering GUI

### GUI Components

| Label | Type | Default | Purpose |
| ----- | ---- | ------- | ------- |
| -    | [QLineEdit](https://doc.qt.io/qt-5/qlineedit.html)              | /cmd | Determines the topic to which the Robot Steering GUI commands are published (as [ackermann_msgs/AckermannDriveStamped](http://docs.ros.org/en/jade/api/ackermann_msgs/html/msg/AckermannDriveStamped.html) messages). |
| Stop | [QPushButton](https://doc.qt.io/qt-5/qpushbutton.html)          | -            | Resets sliders to zero. |
| -    | [QSlider](https://doc.qt.io/qt-5/qslider.html) (vertical)       | 0            | Linear velocity/acceleration (m/s or m/s^2) sent on commands topic. |
| -    | [QSlider](https://doc.qt.io/qt-5/qslider.html) (horizontal)     | 0            | Steering angle (radians) sent on commands topic. |

The 4 [QDoubleSpinBox](https://doc.qt.io/qt-5/qdoublespinbox.html) objects set the minimum and maximum values of the sliders.
The default values of these are dynamic with respect to the command mode. For the acceleration command mode they all default to +/- 1.00.
For the velocity command mode the steering angle max/min values remain at +/-1.00 radians, but the linear velocity max/min values become +/-5.00 m/s.

The label showing the vertical slider value also changes to the appropriate units with command mode.

The '0' [QPushButtons](https://doc.qt.io/qt-5/qpushbutton.html) reset each individual slider to zero.

The +/- and >/< [QPushButtons](https://doc.qt.io/qt-5/qpushbutton.html) increment their sliders by a fixed amount.

### ROS 2 Publishers

The [Robot Steering GUI](./src/eufs_rqt/EUFSRobotSteeringGUI.py) doesn't use a publisher with a static topic name, instead the topic name can be set dynamically via a textbox input on the GUI.
By default, the topic used is `/cmd`. 

The GUI publishes vehicle command onto this topic using [ackermann_msgs/AckermannDriveStamped](http://docs.ros.org/en/jade/api/ackermann_msgs/html/msg/AckermannDriveStamped.html) messages.

### ROS 2 Clients

| Name | Type | Purpose |
| ---- | ---- | ------- |
| `/race_car_model/command_mode` | [std_srvs/Trigger](http://docs.ros.org/en/melodic/api/std_srvs/html/srv/Trigger.html) | Sends a request to the [race car model plugin](../eufs_plugins/gazebo_race_car_model/src/gazebo_ros_race_car_model.cpp), which should return the vehicle command mode. GUI will not display until a response is received. |

## Mission Control GUI

### GUI Components

| Label | Type | Default | Purpose |
| ----- | ---- | ------- | ------- |
| Mission          | [QComboBox](https://doc.qt.io/qt-5/qcombobox.html)     | NOT_SELECTED | Selects a mission. |
| Set Mission      | [QPushButton](https://doc.qt.io/qt-5/qpushbutton.html) | -            | Requests that the [state_machine](../eufs_plugins/gazebo_race_car_model/src/state_machine.cpp) changes it's AMI state to the mission corresponding to the adjacent QComboBox's current text. |
| Mission (lower)  | [QLabel](https://doc.qt.io/qt-5/qlabel.html)           | NOT_SELECTED | Displays the current running Mission. |
| Manual Drive     | [QPushButton](https://doc.qt.io/qt-5/qpushbutton.html) | -            | Requests that the [state_machine](../eufs_plugins/gazebo_race_car_model/src/state_machine.cpp) changes it's AMI state to AMI_MANUAL. |
| State            | [QLabel](https://doc.qt.io/qt-5/qlabel.html)           | OFF          | Displays the current [state_machine](../eufs_plugins/gazebo_race_car_model/src/state_machine.cpp) AS state. |
| Reset State      | [QPushButton](https://doc.qt.io/qt-5/qpushbutton.html) | -            | Requests AS and AMI state reset. |
| Reset Simulation | [QPushButton](https://doc.qt.io/qt-5/qpushbutton.html) | -            | Resets AS and AMI states, vehicle position and cone positions. |
| Request EBS      | [QPushButton](https://doc.qt.io/qt-5/qpushbutton.html) | -            | Requests that the [state_machine](../eufs_plugins/gazebo_race_car_model/src/state_machine.cpp) changes it's AS state to AS_EMERGENCY_BRAKE and stop the car.|

### ROS 2 Publishers

| Name | Type | Purpose |
| ---- | ---- | ------- |
| `/ros_can/set_mission` | [eufs_msgs/CanState](https://gitlab.com/eufs/eufs_msgs/-/blob/ros2/msg/CanState.msg) | Sends a mission request to the [state_machine](../eufs_plugins/gazebo_race_car_model/src/state_machine.cpp). Note that in the CanState message only the AMI state is used. |

### ROS 2 Subscribers

| Name | Type | Purpose |
| ---- | ---- | ------- |
| `/ros_can/state` | [eufs_msgs/CanState](https://gitlab.com/eufs/eufs_msgs/-/blob/ros2/msg/CanState.msg) | Obtains the simulated vehicle's current AS and AMI state and displays it on the GUI. |

### ROS 2 Clients

| Name | Type | Purpose |
| ---- | ---- | ------- |
| `/ros_can/ebs`               | [std_srvs/Trigger](http://docs.ros.org/en/melodic/api/std_srvs/html/srv/Trigger.html) | Requests that the current AS state in the [state_machine](../eufs_plugins/gazebo_race_car_model/src/state_machine.cpp) be changed to `AS_EMERGENCY_BRAKE`. |
| `/ros_can/reset`             | [std_srvs/Trigger](http://docs.ros.org/en/melodic/api/std_srvs/html/srv/Trigger.html) | Requests that the [state_machine](../eufs_plugins/gazebo_race_car_model/src/state_machine.cpp) resets its state. |
| `/ros_can/reset_vehicle_pos` | [std_srvs/Trigger](http://docs.ros.org/en/melodic/api/std_srvs/html/srv/Trigger.html) | Sends a request to the [race car model plugin](../eufs_plugins/gazebo_race_car_model/src/gazebo_ros_race_car_model.cpp) to reset the vehicle position. |
| `/ros_can/reset_cone_pos`    | [std_srvs/Trigger](http://docs.ros.org/en/melodic/api/std_srvs/html/srv/Trigger.html) | Sends a request to the [cone ground truth plugin](../eufs_plugins/gazebo_cone_ground_truth/src/gazebo_cone_ground_truth.cpp) to reset the cone position. |

## Changing UI Interfaces

The layout of the Robot Steering GUI is defined in [EUFSRobotSteeringGUI.ui](./resource/EUFSRobotSteeringGUI.ui).
The layout of the Mission Control GUI is defined in [MissionControlGUI.ui](./resource/MissionControlGUI.ui).
You can edit, add and remove features on the GUI's from here.

After this you must [generate a perspective file](https://www.clearpathrobotics.com/assets/guides/kinetic/ros/Creating%20RQT%20Dashboard.html).

Save this in the [config directory](./config) of this package and make sure to add it to the package [setup](./setup.py) file.

## Launching the GUI

The GUI can be launched as part of a `rqt_gui` ROS 2 node. The `package` parameter must be `rqt_gui` and the perspective file must be passed as an argument.

An example of this can be found in [load_car.launch.py](../eufs_racecar/launch/load_car.launch.py).
