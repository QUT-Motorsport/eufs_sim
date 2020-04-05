# eufs_gazebo
This is the main simulation package which contains the simulation model of the car an several environments.

## Nodes

### sbg_raw_simulator.py
Node that simulates the outputs of our sbg sensor

#### Subscribers:
`/imu` (of type `sensor_msgs/Imu`)
`/gps`    (of type `sensor_msgs/NavSatFix`)

#### Publishers:
`/imu/nav_sat_fix` (of type `sensor_msgs/NavSatFix`)
`/imu/data`    (of type `sensor_msgs/Imu`)

### ekf_evaluator.py
Node that listens to ground truth and the ekf outputs and compares their accuracy

#### Subscribers:
`/odometry/filtered` (of type `nav_msgs/Odometry`)
`/accel/filtered` (of type `geometry_msgs/AccelWithCovarianceStamped`)
`/imu` (of type `sensor_msgs/Imu`)
`/gps_velocity` (of type `geometry_msgs/Vector3Stamped`)

#### Publishers:
`/ekf/evaluation` (of type `eufs_msgs/EKFErr`)

### fastslam_evaluator.py
Node that listens to ground truth and fastslam outputs and compares their accuracy

#### Subscribers:
`fast_slam/pose` (of type `geometry_msgs/Pose`)
`/fast_slam/map` (of type `eufs_msgs/ConeArray`)
`/ground_truth/cones` (of type `eufs_msgs/ConeArray`)
`/ground_truth/state ` (of type `eufs_msgs/CarState`)


#### Publishers:
`/slam/evaluation` (of type `eufs_msgs/SLAMErr`)

### perception_sensors_simulator.py
Node that applies the noise profile of perception sensors onto the ground truth
cone output received from `cone_ground_truth.py`

#### Subscribers:
`ground_truth/all_cones` (of type `eufs_msgs/ConeArray`)

#### Publishers:
`/perception_sim/cones` (of type `eufs_msgs/ConeArrayWithCovariance`)

## Launches

- `acceleration.launch` - Launches a simulation of the acceleration event at competition. 50m straight track.
- `big_track.launch` - Launches a simulation of a relatively big artificially created track (100x100m).
- `empty.launch` - Launches a simulation which only has the car without anything else.
- `eufs_control.launch` - Launches the joint controller for the robot. This is also included in all other simulation launches. Without this the car can't move.
- `skidpad.launch` - Launches a simulation of the skidpad event at competition. It's quite literally a figuire of 8.
- `small_track.launch` - Launches a simulation of an artificially created small track. Better run this if you don't have a good performing computing.
- `sprint17.launch` - Launches a simulation of the 2017 FSUK sprint event. This is the biggest simulation by far and will probably wreck your computer. Also nobody has bothered so far to colour code the cones on it.

## Notes
- If you are running a low-power computer or a VM, it's best to run only `small_track.launch` as the other sims will most likely kill your computer.
