
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