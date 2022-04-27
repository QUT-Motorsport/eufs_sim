# eufs_launcher

## LauncherModule

Package that configures then launches eufs_sim. For a basic usage guide see [How To Launch eufs_sim](https://gitlab.com/eufs/eufs_sim/-/wikis/Simulation/How-To-Launch-eufs_sim).

### Launch Parameters:

| Name | Type | Default | Purpose |
| ----- | ---- |  ------ | ------- |
| config     | string | config/eufs_launcher | Name of yaml file (relative to `config_loc`) containing launcher default configuration. |
| config_loc | string | eufs_launcher        | Name of package containing the `config` file. |
| gui        | bool   | true                 | Enable/disable GUI. |

### ROS 2 Nodes
eufs_launcher can be launched as a ROS 2 node. See [eufs_launcher.launch.py](./launch/eufs_launcher.launch.py) for an example of how to launch.

This node has no services, publishers or subscribers.

### GUI Components

| Label | Type | Default | Purpose |
| ----- | ---- | ------- | ------- |
| Track                    | [QComboBox](https://doc.qt.io/qt-5/qcombobox.html)     | small_track    | Selects the world launch file in [eufs_tracks](../eufs_tracks/launch) to be launched. |
| Refresh                  | [QPushButton](https://doc.qt.io/qt-5/qpushbutton.html) | -              | Refreshes the track dropdown menu- checking if the track list has changed. |
| Launch!                  | [QPushButton](https://doc.qt.io/qt-5/qpushbutton.html) | -              | Launches eufs_sim with current launcher configuration. |
| Vehicle Model            | [QComboBox](https://doc.qt.io/qt-5/qcombobox.html)     | DynamicBicycle | The [vehicle model sub-class](../eufs_models/src) to use. |
| Command Mode             | [QComboBox](https://doc.qt.io/qt-5/qcombobox.html)     | acceleration   | Determines whether the vehicle is controlled using `acceleration` or `velocity`. Also determines the outputs of the [Robot Steering GUI](../eufs_rqt/src/eufs_rqt/EUFSRobotSteeringGUI.py). |
| Vehicle Moodel Presets   | [QComboBox](https://doc.qt.io/qt-5/qcombobox.html)     | DryTrack       | Which vehicle model config file to use from [eufs_racecar](../eufs_racecar/robots). |
| RViz                     | [QCheckBox](https://doc.qt.io/qt-5/qcheckbox.html)     | True           | Whether to launch RViz. |
| Gazebo GUI               | [QCheckBox](https://doc.qt.io/qt-5/qcheckbox.html)     | False          | Whether to launch the Gazebo GUI (i.e [gzclient](http://gazebosim.org/tutorials?tut=components&cat=get_started), gzserver will still be launched). |
| Use Simulated Perception | [QCheckBox](https://doc.qt.io/qt-5/qcheckbox.html)     | True           | Whether [gazebo_cone_ground_truth](../eufs_plugins/gazebo_cone_ground_truth/src/gazebo_cone_ground_truth.cpp) should publish cones with noise to 'simulate' the output of a perception system. |
| Ground Truth TF          | [QCheckBox](https://doc.qt.io/qt-5/qcheckbox.html)     | False          | Whether [gazebo_ros_race_car_model](../eufs_plugins/gazebo_race_car_model/src/gazebo_ros_race_car_model.cpp) should publish ground truth transforms. |
| Publish Ground Truth     | [QCheckBox](https://doc.qt.io/qt-5/qcheckbox.html)     | True           | Whether to publish ground truth topics. |

### Editing the GUI's UI

The file for editing the Launcher GUI's layout can be found in [launcher.ui](./resource/launcher.ui).
For adding additional options on launch, see [LauncherModule.py](./src/eufs_launcher/LauncherModule.py).
