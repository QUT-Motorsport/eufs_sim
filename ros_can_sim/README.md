# ros_can_sim
This is the package that simulates the ros_can interface of the ADS-DV in Gazebo. Also controls all of the
joints(motors) of the car and has the same internal state machine and outputs of the ADS-DV.

Modelled after the [original specs of the car](https://www.imeche.org/docs/default-source/1-oscar/formula-student/2019/fs-ai/ads-dv-software-interface-specification-v0-2.pdf?sfvrsn=2).

# Nodes
## `ros_can_sim_node`
Same as above

### Subscribers
- `/eufs/joint_states` (sensor_msgs/JointState) - states of the joints in Gazebo
- `/cmd_vel_out` (ackermann_msgs/AckermannDriveStamped) - control command for the car
- `/ros_can/flag` (std_msgs/Bool) - mission flag of the car. Used to control the state machine transition logic
- `/ros_can/set_mission` (eufs_msgs/CanState) - sets the mission of the internal state machine

### Services
- `/ros_can/reset` (std_srvs/Trigger) - resets the internal state machine. SIMULATION ONLY
- `/ros_can/ebs` (std_stvs/Trigger) - makes the car transition to the EMERGENCY_BRAKE state and stop

### Publishers
- `/ros_can/state` (eufs_msgs/CanState) - gives the state of the internal state machine
- `/ros_can/state_str` (std_msgs/String) - same as above but in string format for easy debugging
- `/ros_can/wheel_speeds` (eufs_msgs/WheelSpeeds) - RPM of the wheels and angle of the steering rack

### Parameters
- `wheelbase` - distance between the front and rear axis in m
- `wheel_radius` - radius in m
- `max_speed` - limit in m/s
- `max_steering` - maximum steering values in rad
- `steering_link_length` - the length of the steering links in an ackermann-type vehicle
- `desired_freq` - rate at which this node should operate
- `joint_state_time_window` - used in the start() function when the node waits for data from the simulation during boot

# Launches
- `ros_can_sim.launch` - launches the `ros_can_sim_node` fully parametarised for the ADS-DV car

# Custom rqt Plugins
- `ros_can_sim_gui` - controls the state machine of ros_can_sim
- `eufs_robot_steering_gui` - controls the car using AckermannDriveStamped messages (slightly modified version of [rqt_robot_steering](https://github.com/ros-visualization/rqt_robot_steering))

## Notes
- DO NOT RUN THIS IN THE REAL WORLD
