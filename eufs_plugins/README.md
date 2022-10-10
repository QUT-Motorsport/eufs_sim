# eufs_plugins

This package implements a number of [Gazebo plugins](http://gazebosim.org/tutorials?tut=plugins_hello_world) used in eufs_sim.


## gazebo_ros_race_car_model

Responsible for controlling the racecar in simulation and publishing its kinematic and dynamic state. This can also publish ground truth transforms.

A subset of the I/O of this plugin simulates the I/O of the EUFS [ros_can](https://gitlab.com/eufs/ros_can) interface node used for communicating with the FSUK-AI DDT vehicle.

### Parameters 

NB: if a parameter has no default value, a value must be provided.

| Name | Type | Default | Purpose |
| ----- | ---- |  ------ | ------- |
| `update_rate`                     | string    | `1000`             | Update rate of the vehicle model (updates per second). |
| `publish_rate`                    | string    | `200`              | Rate to publish messages of the vehicle model (messages per second). |
| `vehicle_model`                   | string    | `DynamicBicyle`    | [Vehicle model sub-class](../eufs_models/src) to use. |
| `front_right_wheel_steering`      | string    | -                  | Name of the front right steering wheel joint. |
| `front_left_wheel_steering`       | string    | -                  | Name of the front left steering wheel joint. |
| `front_right_wheel`               | string    | -                  | Name of the front right wheel joint. |
| `front_left_wheel`                | string    | -                  | Name of the front left wheel joint. |
| `rear_right_wheel`                | string    | -                  | Name of the rear right wheel joint. |
| `rear_left_wheel`                 | string    | -                  | Name of the rear left wheel joint. |
| `yaml_config`                     | string    | -                  | Path to a config file describing the physical properties of the racecar. |
| `referenceFrame`                  | string    | `"map"`            | Tf frame in which to publish data. |
| `robotFrame`                      | string    | `"base_footprint"` | Tf frame of the robot. |
| `publishTransform`                | bool      | `false`            | Whether to publish the car tf. |
| `groundTruthCarStateTopic`        | string    | -                  | Topic in which to publish the ground truth [eufs_msgs/CarState](https://gitlab.com/eufs/eufs_msgs/-/blob/ros2/msg/CarState.msg) message. |
| `localisationCarStateTopic`       | string    | -                  | Topic in which to publish the noisy [eufs_msgs/CarState](https://gitlab.com/eufs/eufs_msgs/-/blob/ros2/msg/CarState.msg) message. |
| `groundTruthWheelSpeedsTopicName` | string    | -                  | Topic in which to publish the ground truth [eufs_msgs/WheelSpeedsStamped](https://gitlab.com/eufs/eufs_msgs/-/blob/ros2/msg/WheelSpeedsStamped.msg) message. |
| `wheelSpeedsTopicName`            | string    | -                  | Topic in which to publish the noisy [eufs_msgs/WheelSpeedsStamped](https://gitlab.com/eufs/eufs_msgs/-/blob/ros2/msg/WheelSpeedsStamped.msg) message. |
| `odometryTopicName`               | string    | -                  | Topic in which to publish the noisy [nav_msgs/Odometry](http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Odometry.html) message. |
| `noise_config`                    | string    | -                  | Config file specifying the standard deviation of Gaussian noise to be added to position, orientation, velocity, acceleration and wheel speed messages. Passed to the [Vehicle model sub-class](../eufs_models). |
| `commandMode`                     | string    | `acceleration`     | Whether to accept `acceleration` or `velocity` control commands. |
| `controlDelay`                    | double    | -                  | Time taken for the simulated vehicle to enact a command (seconds). |
| `steeringLockTime`                | double    | -                  | Time taken to sweep between min and max steering lock (seconds). |
| `pubGroundTruth`                  | bool      | -                  | Whether to publish ground truth topics. |


### Example usage

This has to be inserted inside a robot URDF.

```xml
<gazebo>
    <plugin name="race_car" filename="libgazebo_race_car_model.so">
      <update_rate>1000.0</update_rate>
      <publish_rate>200.0</publish_rate>
      <vehicle_model>DynamicBicycle</vehicle_model>
      <front_left_wheel_steering>left_steering_hinge_joint</front_left_wheel_steering>
      <front_right_wheel_steering>right_steering_hinge_joint</front_right_wheel_steering>
      <front_left_wheel>front_left_wheel_joint</front_left_wheel>
      <front_right_wheel>front_right_wheel_joint</front_right_wheel>
      <rear_left_wheel>rear_left_wheel_joint</rear_left_wheel>
      <rear_right_wheel>rear_right_wheel_joint</rear_right_wheel>
      <yaml_config>"PATH/TO/CONFIG/IN/SHARE/DIRECTORY"</yaml_config>
      <noise_config>"PATH/TO/NOISE/CONFIG/IN/SHARE/DIRECTORY"</noise_config>
      <referenceFrame>map</referenceFrame>
      <robotFrame>base_footprint</robotFrame>
      <publishTransform>false</publishTransform>
      <groundTruthCarStateTopic>/ground_truth/state</groundTruthCarStateTopic>
      <localisationCarStateTopic>/odometry_integration/car_state</localisationCarStateTopic>
      <wheelSpeedsTopicName>/ros_can/wheel_speeds</wheelSpeedsTopicName>
      <groundTruthWheelSpeedsTopicName>/ground_truth/wheel_speeds</groundTruthWheelSpeedsTopicName>
      <odometryTopicName>/ground_truth/odom</odometryTopicName>
      <commandMode>acceleration</commandMode>
      <controlDelay>0.2</controlDelay>
      <steeringLockTime>1</steeringLockTime>
    </plugin>
</gazebo>
```

### ROS 2 Publishers

| Topic Name | Type | Purpose |
| ---------- | ---- | ------- |
| `groundTruthCarStateTopic` parameter value        | [eufs_msgs/CarState](https://gitlab.com/eufs/eufs_msgs/-/blob/ros2/msg/CarState.msg)                     | Publishes the ground truth dynamic and kinematic state of the simulated car. |
| `localisationCarStateTopic` parameter value       | [eufs_msgs/CarState](https://gitlab.com/eufs/eufs_msgs/-/blob/ros2/msg/CarState.msg)                     | Publishes the dynamic and kinematic state of the simulated car with added noise. |
| `groundTruthWheelSpeedsTopicName` parameter value | [eufs_msgs/WheelSpeedsStamped](https://gitlab.com/eufs/eufs_msgs/-/blob/ros2/msg/WheelSpeedsStamped.msg) | Publishes ground truth wheel speeds. |
| `wheelSpeedsTopicName` parameter value            | [eufs_msgs/WheelSpeedsStamped](https://gitlab.com/eufs/eufs_msgs/-/blob/ros2/msg/WheelSpeedsStamped.msg) | Publishes wheel speeds with added noise. |
| `odometryTopicName` parameter value               | [nav_msgs/Odometry](http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Odometry.html)                   | Publishes vehicle odometry with noise. |

### ROS 2 Subscribers

| Topic Name | Type | Purpose |
| ---------- | ---- | ------- |
| `/cmd` | [ackermann_msgs/AckermannDriveStamped](http://docs.ros.org/en/jade/api/ackermann_msgs/html/msg/AckermannDriveStamped.html) | Recieves desired vehicle command input. |

### ROS 2 Services

| Service Name | Type | Purpose |
| ------------ | ---- | ------- |
| `/ros_can/reset_vehicle_pos`   | [std_srvs/Trigger](http://docs.ros.org/en/melodic/api/std_srvs/html/srv/Trigger.html) | Resets the position of the simulated car. |
| `/race_car_model/command_mode` | [std_srvs/Trigger](http://docs.ros.org/en/melodic/api/std_srvs/html/srv/Trigger.html) | Returns the vehicle command mode (velocity or acceleration). |

## state_machine

Simulates the state machine of the FSUK-AI DDT vehicle. A design outline can be found on the [wiki](https://gitlab.com/eufs/eufs_sim/-/wikis/State-Machine).

### ROS 2 Publishers

| Topic Name | Type | Purpose |
| ---------- | ---- | ------- |
| `/ros_can/state`     | [eufs_msgs/CanState](https://gitlab.com/eufs/eufs_msgs/-/blob/ros2/msg/CanState.msg) | Publishes the AS and AMI state of the simulated car. |
| `/ros_can/state_str` | [std_msgs/String](http://docs.ros.org/en/noetic/api/std_msgs/html/msg/String.html)   | Publishes the AS and AMI state of the simulated car as a string. |

### ROS 2 Subscribers

| Topic Name | Type | Purpose |
| ---------- | ---- | ------- |
| `/ros_can/set_mission`      | [eufs_msgs/CanState](https://gitlab.com/eufs/eufs_msgs/-/blob/ros2/msg/CanState.msg) | Sets the AMI state of the simulated car. |
|`/ros_can/mission_completed` | [std_msgs/Bool](http://docs.ros.org/en/noetic/api/std_msgs/html/msg/Bool.html)       | True when the current autonomous mission has been completed. |

### ROS 2 Services 

| Service Name | Type | Purpose |
| ------------ | ---- | ------- |
| `/ros_can/reset` | [std_srvs/Trigger](http://docs.ros.org/en/melodic/api/std_srvs/html/srv/Trigger.html) | Resets car state. |
| `/ros_can/ebs`   | [std_srvs/Trigger](http://docs.ros.org/en/melodic/api/std_srvs/html/srv/Trigger.html) | Changes AS state to AS_EMERGENCY_BREAK, causing the racecar to come to an immediate stop. |

## gazebo_cone_ground_truth

Publishes data about the cones in the simulated world. Can publish ground truth cones of the entire track, ground truth data of cones currently in front of the car, or noisy data of cones currently in front of the car. Color of the cones in the noisy data are only known if visible with the camera (which is set to have a smaller field of view than the lidar). Cones visible with the lidar but not the camera are set to have no colour.

### Parameters

NB: if a parameter has no default value, a value must be provided.

| Name | Type | Default | Purpose |
| ----- | ---- |  ------ | ------- |
| `alwaysOn`                             | bool      | true               | Should Gazebo always invoke this plugin?  |
| `updateRate`                           | double    | 0                  | Rate at which this plugin publishes data (updates per second). Default is as fast as possible. |
| `cameraViewDistance`                   | double    | 15                 | Maximum distance for cones to be in camera range (metres). |
| `cameraMinViewDistance`                | double    | 1                  | Minimum distance for cones to be in camera range (metres). |
| `cameraFOV`                            | double    | 1.918889           | Absolute angle (radians) split evenly between +x and -x directions defining a sector containing all cones in the camera FOV. |
| `lidarViewDistance`                    | double    | 100                | Maximum distance from the car for which a cone is in lidar range (metres). |
| `lidarMinViewDistance`                 | double    | 1                  | Minimum distance from the car for which a cone is in lidar range (metres). |
| `lidarXViewDistance`                   | double    | 20                 | Range of x values a cone must have to be in lidar range (metres). |
| `lidarYViewDistance`                   | double    | 10                 | Range of y values a cone must have to be in lidar range (metres). |
| `lidarFOV`                             | double    | 3.14159            | Absolute angle (radians) split evenly between +x and -x directions defining a sector containing all cones in the lidar FOV. |
| `lidarOn`                              | bool      | true               | Whether the lidar is switched on. |
| `trackFrame`                           | string    | map                | Tf frame in which to publish the track. |
| `groundTruthConesTopicName`            | string    | -                  | Topic in which to publish ground truth cones ([eufs_msgs/ConeArrayWithCovariance](https://gitlab.com/eufs/eufs_msgs/-/blob/ros2/msg/ConeArrayWithCovariance.msg)). |
| `groundTruthConeMarkersTopicName`      | string    | -                  | Topic in which to publish ground truth cone markers for visualization ([visualization_msgs/MarkerArray](http://docs.ros.org/en/noetic/api/visualization_msgs/html/msg/MarkerArray.html)). |
| `groundTruthTrackTopicName`            | string    | -                  | Topic in which to publish the entire ground truth track, visualization topic is this + "/viz" ([eufs_msgs/ConeArrayWithCovariance](https://gitlab.com/eufs/eufs_msgs/-/blob/ros2/msg/ConeArrayWithCovariance.msg), viz msg: [visualization_msgs/MarkerArray](http://docs.ros.org/en/noetic/api/visualization_msgs/html/msg/MarkerArray.html)) |
| `simulatePerception`                   | bool      | true               | Whether this package should publish to the perception cones topic |
| `perceptionConesTopicName`             | string    | -                  | Topic in which to publish noisy cones if simulating perception ([eufs_msgs/ConeArrayWithCovariance](https://gitlab.com/eufs/eufs_msgs/-/blob/ros2/msg/ConeArrayWithCovariance.msg)). |
| `perceptionConeMarkersTopicName`       | string    | -                  | Topic in which to publish noisy cone markers for visualization ([visualization_msgs/MarkerArray](http://docs.ros.org/en/noetic/api/visualization_msgs/html/msg/MarkerArray.html)). |
| `perceptionCameraDepthNoiseParameterA` | double    | 0.0184             | Noise weighting parameter for camera depth noise contribution. |
| `perceptionCameraDepthNoiseParameterB` | double    | 0.2106             | Noise weighting parameter for camera depth noise contribution. |
| `perceptionLidarNoise`                 | double[3] | `[0.03,0.03, 0.0]` | Noise weighting parameters for lidar noise contribution. |
| `pubGroundTruth`                       | bool      | -                  | Whether to publish ground truth topics. |

#### Lidar View Distance Parameters (more detail):

The `lidarXViewDistance` and `lidarYViewDistance` parameters define a rectangular area.
The `lidarViewDistance`, `lidarMinViewDistance` and `lidarFOV` parameters define a sector of an [Annulus](https://en.wikipedia.org/wiki/Annulus_(mathematics)).
The area in which cones can be detected by lidar is the area of intersection between these two shapes.

### Example usage

This has to be inserted inside a robot URDF.

```xml
  <gazebo>
    <plugin name="cone_ground_truth" filename="libgazebo_cone_ground_truth.so">
      <alwaysOn>true</alwaysOn>
      <updateRate>25.0</updateRate>
      <cameraViewDistance>15</cameraViewDistance>
      <lidarViewDistance>100</lidarViewDistance>
      <lidarXViewDistance>20</lidarXViewDistance>
      <lidarYViewDistance>10</lidarYViewDistance>
      <lidarMinViewDistance>1</lidarMinViewDistance>
      <cameraMinViewDistance>1</cameraMinViewDistance>
      <cameraFOV>1.918889</cameraFOV><!--110 degrees-->
      <lidarFOV>3.141593</lidarFOV><!-- 180 degrees-->
      <lidarOn>true</lidarOn><!--If false, we only simulate camera measurements-->
      <trackFrame>map</trackFrame>
      <groundTruthConesTopicName>/ground_truth/cones</groundTruthConesTopicName>
      <groundTruthConeMarkersTopicName>/ground_truth/cones/viz</groundTruthConeMarkersTopicName>
      <groundTruthTrackTopicName>/ground_truth/track</groundTruthTrackTopicName>
      <simulatePerception>true</simulatePerception>
      <perceptionConesTopicName>/fusion/cones</perceptionConesTopicName>
      <perceptionConeMarkersTopicName>/fusion/cones/viz</perceptionConeMarkersTopicName>
      <perceptionCameraDepthNoiseParameterA>0.0184</perceptionCameraDepthNoiseParameterA>
      <perceptionCameraDepthNoiseParameterB>0.2106</perceptionCameraDepthNoiseParameterB>
      <perceptionLidarNoise>0.03 0.03 0.0</perceptionLidarNoise>
    </plugin>
  </gazebo>
```

### ROS 2 Publishers

| Topic Name | Type | Purpose |
| ---------- | ---- | ------- |
| `groundTruthConesTopicName` parameter value          | [eufs_msgs/ConeArrayWithCovariance](https://gitlab.com/eufs/eufs_msgs/-/blob/ros2/msg/ConeArrayWithCovariance.msg)  | Publishes ground truth array of cone objects detectable by sensors. |
| `groundTruthConeMarkersTopicName` parameter value    | [visualization_msgs/MarkerArray](http://docs.ros.org/en/noetic/api/visualization_msgs/html/msg/MarkerArray.html)    | Publishes ground truth array of marker objects detectable by sensors for visualisation. |
| `groundTruthTrackTopicName` parameter value          | [eufs_msgs/ConeArrayWithCovariance](https://gitlab.com/eufs/eufs_msgs/-/blob/ros2/msg/ConeArrayWithCovariance.msg)  | Publishes the entire ground truth track in the map frame. |
| `groundTruthTrackTopicName` parameter value + `/viz` | [visualization_msgs/MarkerArray](http://docs.ros.org/en/noetic/api/visualization_msgs/html/msg/MarkerArray.html)    | Publishes entire ground truth track as markers for visualisation. |
| `perceptionConesTopicName` parameter value           | [eufs_msgs/ConeArrayWithCovariance](https://gitlab.com/eufs/eufs_msgs/-/blob/ros2/msg/ConeArrayWithCovariance.msg)  | Publishes array of cone objects detectable by sensors with added noise. |
| `perceptionConeMarkersTopicName` parameter value     | [visualization_msgs/MarkerArray](http://docs.ros.org/en/noetic/api/visualization_msgs/html/msg/MarkerArray.html)    | Publishes array of marker objects detectable by sensors with added noise for visualisation. |

### ROS 2 Services

| Service Name | Type | Purpose |
| ------------ | ---- | ------- |
| `/ros_can/reset_cone_pos` | [std_srvs/Trigger](http://docs.ros.org/en/melodic/api/std_srvs/html/srv/Trigger.html) | Resets cone positions to initial track layout. |

## gazebo_simulate_bounding_boxes

Publishes synthesized bounding boxes.

Note : Bounding boxes depend on the ground truth cones topic. Its publish rate is limited to that of the ground truth cones topic.

### Parameters


| Name | Type | Default | Purpose |
| ----- | ---- |  ------ | -------|
| `publish_rate`                  | double   | 50.0                             | Sets the publish rate of the bounding boxes topics.                           |
| `config_file`                   | string   | boundingBoxes.yaml               | Retrieves the config file which contains the camera callibration settings and noises for the bounding boxes.|
| `targetFrame`                   | string   | "zed_right_camera_optical_frame" | Target frame must be any camera_optical_frame link.                          |
| `sourceFrame`                   | string   | "base_footprint"                 | Source frame from which the position of the `cones` are translated to.       |
| `cameraWidth`                   | int      | 1280                             | Camera resolution (width).                                                   |
| `cameraHeight`                  | int      | 720                              | Camera resolution (height).                                                  |
| `simulateBoundingBoxesTopicName`| string   | "bounding_boxes"                 | Topic which publishes bounding boxes of type `eufs_msgs/msg/BoundingBoxes`.  |
| `custom_camera_info`            | string   |  "customCameraInfo"              | Topic name of custom camera info.                                            |


### Example usage

This has to be inserted inside a robot URDF.

```xml
  <gazebo>
    <plugin name="bounding_boxes" filename="libgazebo_simulate_bounding_boxes.so">
      <config_file>config/boundingBoxes.yaml</config_file>
      <publish_rate>50.0</publish_rate>
      <targetFrame>zed_right_camera_optical_frame</targetFrame>
      <sourceFrame>base_footprint</sourceFrame>
      <cameraWidth>1280</cameraWidth>
      <cameraHeight>720</cameraHeight>
      <gtBoundingBoxesTopic>ground_truth/bounding_boxes</gtBoundingBoxesTopic>
      <noisyBoundingBoxesTopic>noisy_bounding_boxes</noisyBoundingBoxesTopic>
      <customCameraInfo>custom_camera_info</customCameraInfo>
    </plugin>
  </gazebo>
```

### ROS 2 Publishers

| Topic Name | Type | Purpose |
| ---------- | ---- | ------- |
| `gtBoundingBoxesTopic` parameter value           | [eufs_msgs/msg/BoundingBoxes](https://gitlab.com/eufs/eufs_msgs/-/blob/master/msg/BoundingBoxes.msg) | Publishes the ground truth bounding boxes.
| `noisyBoundingBoxesTopic` parameter value        | [eufs_msgs/msg/BoundingBoxes](https://gitlab.com/eufs/eufs_msgs/-/blob/master/msg/BoundingBoxes.msg) | Publishes bounding boxes with gaussian noise.
| `customCameraInfo` parameter value               | [sensor_msgs/msg/CameraInfo](http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/CameraInfo.html) | Contains the cameraInfo which will then be used to perform image projection from 3D plane to 2D plane.
