
# eufs_gazebo_plugins

This is the main simulation package which contains the simulation model of the car an several environments.

## Plugins

### gazebo_ros_race_car_model

Provides the interface in order to control the race car in the simulation.
Also provides ground truth state in the form of `nav_msgs/Odometry` and  
`eufs_msgs/CarState`. Additionally can publish transform.

#### Parameters

| Name | Type | Default | Purpose |
| ----- | ---- |  ------ | ------- |
| `update_rate`                | string    | `1000`             | The update rate of the vehicle model. |
| `publish_rate`               | string    | `200`              | The rate to publish messages of the vehicle model. |
| `vehicle_model`              | string    | `DynamicBicyle`    | The vehicle model class to use for the race car. |
| `front_left_wheel_steering`  | string    |                    | Required parameter. The name of the front left  steering wheel joint. |
| `front_right_wheel`          | string    |                    | Required parameter. The name of the front right steering wheel joint. |
| `front_left_wheel`           | string    |                    | Required parameter. The name of the front left  wheel joint. |
| `front_right_wheel_steering` | string    |                    | Required parameter. The name of the front right wheel joint. |
| `rear_left_wheel`            | string    |                    | Required parameter. The name of the rear  left  wheel joint. |
| `rear_right_wheel`           | string    |                    | Required parameter. The name of the rear  right wheel joint. |
| `yaml_config`                | string    |                    | Required parameter. The path to the config file describing the car. |
| `referenceFrame`             | string    | `"map"`            | The tf frame in which to publish data. |
| `robotFrame`                 | string    | `"base_footprint"` | The tf frame of the robot. |
| `publishTransform`           | bool      | `false`            | Whether or not the tf of the car should be published |
| `stateTopicName`             | string    | -                  | Required parameter. The topic in which to publish the eufs_msgs/CarState message. |
| `wheelSpeedsTopicName`       | string    | -                  | Required parameter. The topic in which to publish the eufs_msgs/WheelSpeedsStamped message. |
| `odometryTopicName`          | string    | -                  | Required parameter. The topic in which to publish the nav_msgs/Odometry message. |
| `positionNoise`              | double[3] | `[0,0,0]`          | Position noise.            Inducted in the position fields and in the covariance. ([x, y, z]) |
| `orientationNoise`           | double[3] | `[0,0,0]`          | Orientation noise.         Inducted in the position fields and in the covariance. ([yaw, pitch, roll]) |
| `linearVelocityNoise`        | double[3] | `[0,0,0]`          | Linear velocity noise.     Inducted in the position fields and in the covariance. ([x, y, z]) |
| `angularVelocityNoise`       | double[3] | `[0,0,0]`          | Angular velocity noise.    Inducted in the position fields and in the covariance. ([x, y, z]) |
| `linearAccelerationNoise`    | double[3] | `[0,0,0]`          | Linear acceleration noise. Inducted in the position fields and in the covariance. ([x, y, z]) |

#### Example usage

This has to be inserted inside a robot URDF

```xml
  <gazebo>
    <plugin name="race_car" filename="libgazebo_race_car_model.so">
      <update_rate>200.0</update_rate>
      <vehicle_model>$(arg vehicle_model)</vehicle_model>
      <front_left_wheel_steering>left_steering_hinge_joint</front_left_wheel_steering>
      <front_right_wheel_steering>right_steering_hinge_joint</front_right_wheel_steering>
      <front_left_wheel>front_left_wheel_joint</front_left_wheel>
      <front_right_wheel>front_right_wheel_joint</front_right_wheel>
      <rear_left_wheel>rear_left_wheel_joint</rear_left_wheel>
      <rear_right_wheel>rear_right_wheel_joint</rear_right_wheel>
      <yaml_config>$(arg config_file)</yaml_config>
      <referenceFrame>map</referenceFrame>
      <robotFrame>base_footprint</robotFrame>
      <publishTransform>$(arg publish_tf)</publishTransform>
      <stateTopicName>/ground_truth/state</stateTopicName>
      <wheelSpeedsTopicName>/ros_can/wheel_speeds</wheelSpeedsTopicName>
      <odometryTopicName>/ground_truth/odom</odometryTopicName>
      <positionNoise>0.0 0.0 0.0</positionNoise>
      <orientationNoise>0.0 0.0 0.0</orientationNoise>
      <linearVelocityNoise>0.0 0.0 0.0</linearVelocityNoise>
      <angularVelocityNoise>0.0 0.0 0.0</angularVelocityNoise>
      <linearAccelerationNoise>0.0 0.0 0.0</linearAccelerationNoise>
    </plugin>
  </gazebo>
```

### gazebo_cone_ground_truth

Provides ground truth cones in simulation in the form of `eufs_msgs/ConeArray`.

Can also simulate the perception stack by publishing cones with noise.

#### Parameters

| Name | Type | Default | Purpose |
| ----- | ---- |  ------ | ------- |
| `alwaysOn`                        | bool      | true                     | Should Gazebo always invoke this plugin?  |
| `updateRate`                      | float     | 0.0                      | The rate at which this plugin publishes data. Default is as fast as possible. |
| `viewDistance`                    | float     | 15.0                     | Distance from the car within which cones will be published |
| `fov`                             | float     | 1.91986                  | Angle in front of the car within which the cones will be published |
| `coneFrame`                       | string    | base_footprint           | The tf frame in which to publish data. |
| `groundTruthConesTopicName`       | string    | -                        | Required parameter. The topic in which to publish the eufs_msgs/CarState message. |
| `groundTruthConeMarkersTopicName` | string    | -                        | Required parameter. The topic in which to publish the eufs_msgs/CarState message. |
| `simulatePerception`              | bool      | $arg simulate_perception | Should cones be published to the perception cones topic |
| `perceptionConesTopicName`        | string    | -                        | Required parameter. The topic in which to publish the eufs_msgs/CarState message. |
| `perceptionConeMarkersTopicName`  | string    | -                        | Required parameter. The topic in which to publish the eufs_msgs/CarState message. |
| `perceptionNoise`                 | double[3] | `[0.2,0.2,0]`            | Noise of the cones published to the perception cones topic |

#### Example usage

This has to be inserted inside a robot URDF

```xml
  <gazebo>
    <plugin name="cone_ground_truth" filename="libgazebo_cone_ground_truth.so">
      <alwaysOn>true</alwaysOn>
      <updateRate>25.0</updateRate>
      <viewDistance>15</viewDistance>
      <fov>1.91986</fov>
      <coneFrame>base_footprint</coneFrame>
      <groundTruthConesTopicName>/ground_truth/cones</groundTruthConesTopicName>
      <groundTruthConeMarkersTopicName>/ground_truth/cones/viz</groundTruthConeMarkersTopicName>
      <simulatePerception>$(arg simulate_perception)</simulatePerception>
      <perceptionConesTopicName>/fusion/cones</perceptionConesTopicName>
      <perceptionConeMarkersTopicName>/fusion/cones/viz</perceptionConeMarkersTopicName>
      <perceptionNoise>0.2 0.2 0.0</perceptionNoise>
    </plugin>
  </gazebo>
```
