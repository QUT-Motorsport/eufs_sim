# eufs_gazebo
This is the main simulation package which contains the simulation model of the car an several environments.

## Nodes
### cone_ground_truth.py
Node that simulates cone locations as they would have been received
by the team's cone detection stack. This reads a CSV file generated
from the track files and then publishes the appropriate cone detections
based on the location of the car.

#### Parameters

- `~view_distance` (float, default: 15). Only the cones that are within this distance are published.
- `~fov` (float, default: 1.91986). Field of view in front of the car
- `~track_path` (string). Path to the track csv file.

#### Subscribers

- `/ground_truth/state_raw` (nav_msgs/Odometry).
        Simulated ground truth odometry for the car

#### Publishers

- `/ground_truth/cones` (eufs_msgs/ConeArray).
        Cone locations in the frame of the car
- `/ground_truth/cones/viz` (visualization_msgs/MarkerArray).
        Cone locations to be displayed in Rviz
- `/ground_truth/midpoints` (eufs_msgs/pointsArray).
        Track midpoints as an array of Points
- `/ground_truth/midpoints/viz` (visualization_msgs/Marker).
        Track midpoints for visualization in Rviz


### ground_truth_republisher.py
This takes the ground truth from the simulation as input and rotates it in correct direction of the car.
Not used but kept here in case someone needs to publish transforms
from ground truth odometry.

## Launches

- `acceleration.launch` - Launches a simulation of the acceleration event at competition. 50m straight track.
- `big_track.launch` - Launches a simulation of a relatively big arteficially created track (100x100m).
- `empty.launch` - Launches a simulation which only has the car without anything else.
- `eufs_control.launch` - Launches the joint controller for the robot. This is also included in all other simulation launches. Without this the car can't move.
- `skidpad.launch` - Launches a simulation of the skidpad event at competition. It's quite literally a figuire of 8.
- `small_track.launch` - Launches a simulation of an arteficially created small track. Better run this if you don't have a good performing computing.
- `sprint17.launch` - Launches a simulation of the 2017 FSUK sprint event. This is the biggest simulation by far and will probably wreck your computer. Also nobody has bothered so far to colour code the cones on it.

## Notes
- If you are running a low-power computer or a VM, it's best to run only `small_track.launch` as the other sims will most likely kill your computer.
