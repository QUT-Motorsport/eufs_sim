# eufs_sim

ROS/Gazebo simulation packages for driverless FSAE vehicles.

![simulation](https://gitlab.com/eufs/eufs_sim/wikis/uploads/ec226003a607ce00ec37a51913bf65e3/image.png)

## Contents

1. [Install Prerequisites](#requirements)
2. [Compiling](#compiling)
3. [Running](#running)
4. [Launch options](#options)
5. [Sensors](#sensors)
6. [Known issues](#issues)

## 1. Install Prerequisites <a name="requirements"></a>

- Install Ubuntu 18.04 LTS
- Install [ros-melodic-desktop-full](http://wiki.ros.org/kinetic/Installation)
- Install [catkin tools](https://catkin-tools.readthedocs.io/en/latest/installing.html)
- Put EUFS Sim inside a [catkin workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace)
- Download [eufs_msgs](https://gitlab.com/eufs/eufs_msgs) and put them in the same folder as `eufs_sim`
- Install ROS dependencies by navigating to the catkin workspace and doing
`rosdep install -i --from-path src/`

_Note: eufs_sim will probably work with other versions of ROS but they are not maintained by the authors_

## 2. Compiling <a name="compiling"></a>

Navigate to your workspace and build the simulation:

```bash
cd [your-catkin-workspace]
catkin build
```

To enable ROS to find the EUFS packages you also need to run

```bash
source ./devel/setup.bash
```

_Note:_ source needs to be run on each new terminal you open.
You can also include it in your `.bashrc` file.

## 3. Running <a name="running"></a>

Now you can finally run our kickass simulation!!

```bash
roslaunch eufs_launcher eufs_launcher.launch
```

You shold have something like this:
![Full Gui](https://gitlab.com/eufs/eufs_sim/wikis/uploads/5bc6e6e51c1b489c372fd4d790744640/image.png)

The basic usage of the simulation is in the left hand side of
the launcher window.
Here you can select the options with which you want to launch
the simulation. These options are desribed in more detail [below](#options).

Using the default options this will launch a RQT with a steering GUI
and our ros_can_sim GUI, alongside a default RViz window.

In the ros_can_sim GUI you can select the a mission from the
drop down menu, and hit 'Set Mission'. This will publish
a `CanState` message to the `/ros_can/set_mission` topic.

Before the car will drive you need to tell it that it can.
This is done by publishing a `Bool` message with the data
set to `true` to the `/ros_can/set_mission` topic.
This can be done using a publisher in a ROS node or,
it can be done using the following command:
`rostopic pub -1 /ros_can/mission_flag std_msgs/Bool "data: true"`

After 5 seconds you should see the State changing from
`OFF` to `DRIVING`.
The car will now be able to drive around the track!

Commands can be sent to the car publihsing an`AckermannDriveStamped`
message to the `/cmd_vel_out` topic.

_Note:_
If you have selected the `Manual` mission and you want to use the
EUFS Robot Steering GUI without any additional nodes to transform the command,
you need to change the topic in the text field at the top of the GUI to
`/cmd_vel_out`


## 4. Launch options <a name="options"></a>

Using the Track dropdown menu you can select which track to start the
simulation in:

- small_track - small fixed trackdrive track used for rapid prototyping
and consistency
- skidpad - pre-configured track as per the Formula Student rules
- acceleration - pre-configured track as per the Formula Student rules
- rand - randomly generated autocross/trackdrive track. One is provided
by default with the sim but you can generate more with the GUI. For
more on that read the wiki

Additional options below:

- Vehicle model - determines the dynamics of the car in simulation.
This is done using the gazebo_race_car_model which allows you
to make a custom vehicle model.
- RViz - launches a default RViz window
- Use Simulated Perception - if enabled, a plugin simulates our perception
stack including noise from the system fitted to real-world data. If this
is disabled, then camera and lidar are fully simulated in simulation.
- Gazebo GUI - starts the Gazebo GUI. Warning: this might kill a weak
computer.
- Ground Truth TF - publishes ground truth state transform. Should be used
if no localisation system is running. However, note that ground truth
state is always publishes.

A full manual of how to use the GUI is
[available here](https://gitlab.com/eufs/eufs_sim/wikis/Simulation/Launcher-Overview).

## 5. Additional sensors <a name="sensors"></a>

**Sensor suit of the car by default:**

- Velodyne VLP16 lidar
- ZED Stereo camera
- IMU
- GPS
- wheel odometry

Additional sensors for testing are avilable via the
`ros-melodic-robotnik-sensor` package. Some of them are already
defined in `eufs_description/robots/eufs.urdf.xarco`. You can simply
commment them in and attach them appropriately to the car.

## 6. Known issues <a name="issues"></a>

- Sometimes you might end up with 2 numpy installations at the same time. To fix this simply uninstall numpy with `pip uninstall numpy` as many times as you can. Then reinstall it with `pip install numpy` and everything should work as usual!
- **If you need to rapidly re-launch** the gui it is important that
you wait a few seconds (5 should be more than enough).
If you try and do it rapidly then you may cause the gui to launch
gazebo while gazebo is still shutting itself down. This will
crash the program and won't give a satisfactory error message
(it will say that it put errors in the log files, but the log files
do not exist). More information can be found in the README
in eufs_launcher
- When using RViz and the car is driving fast, you might notice
that the wheels are dragged behind the chassis of the car. This is a
limitation of the robot_state_publisher node of ROS and shouldn't
cause any issues apart from bad visuals.
- The random track generation is sometimes prone to making bad
tracks that nearly overlap. Before committing to generated track,
closely inspect the image for any issues.
