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

### perception_sensors_simulator.py
Node that applies the noise profile of perception sensors onto the ground truth
cone output received from `cone_ground_truth.py`

#### Subscribers:
`ground_truth/all_cones` (of type `eufs_msgs/ConeArray`)

#### Publishers:
`/perception_sim/cones` (of type `eufs_msgs/ConeArrayWithCovariance`)

## Launches

- `acceleration.launch` - Launches a simulation of the acceleration event at competition. 50m straight track.
- `empty.launch` - Launches a simulation which only has the car without anything else.
- `skidpad.launch` - Launches a simulation of the skidpad event at competition. It's quite literally a figuire of 8.
- `small_track.launch` - Launches a simulation of an artificially created small track. Better run this if you don't have a good performing computing.

## Notes
- If you are running a low-power computer or a VM, it's best to run only `small_track.launch` as the other sims will most likely kill your computer.
