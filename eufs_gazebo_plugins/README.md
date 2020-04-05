
# eufs_gazebo_plugins

This is the main simulation package which contains the simulation model of the car an several environments.

## Plugins

### gazebo_state_ground_truth

Provides ground truth state in simulation in the form of `nav_msgs/Odometry` and  
`eufs_msgs/CarState`. Additionally can publish transform.

#### Parameters

| Name | Type | Default | Purpose |
| ----- | ---- |  ------ | ------- |
| `alwaysOn`                | bool      | true      | Should Gazebo always invoke this plugin?  |
| `updateRate`              | float     | 0.0       | The rate at which this plugin publishes data. Default is as fast as possible. |
| `referenceFrame`          | string    | -         | Required parameter. The tf frame in which to publish data. |
| `robotFrame`              | string    | -         | Required parameter. The tf frame of the robot. |
| `odometryTopicName`       | string    | -         | Required parameter. The topic in which to publish the nav_msgs/Odometry message. |
| `stateTopicName`          | string    | -         | Required parameter. The topic in which to publish the eufs_msgs/CarState message. |
| `positionNoise`           | double[3] | `[0,0,0]` | Position noise. Inducted in the position fields and in the covariance. |
| `orientationNoise`        | double[3] | `[0,0,0]` |  |
| `linearVelocityNoise`     | double[3] | `[0,0,0]` |  |
| `angularVelocityNoise`    | double[3] | `[0,0,0]` |  |
| `linearAccelerationNoise` | double[3] | `[0,0,0]` |  |

#### Example usage

This has to be inserted inside a robot URDF

```xml
  <gazebo>
    <plugin name="state_ground_truth" filename="libgazebo_state_ground_truth.so">
      <alwaysOn>true</alwaysOn>
      <updateRate>200.0</updateRate>
      <referenceFrame>map</referenceFrame>
      <robotFrame>base_footprint</robotFrame>
      <publishTransform>$(arg publish_tf)</publishTransform>
      <odometryTopicName>/ground_truth/odom</odometryTopicName>
      <stateTopicName>/ground_truth/state</stateTopicName>
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