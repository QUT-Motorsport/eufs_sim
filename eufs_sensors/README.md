# eufs_sensors

This package contains [meshes](./meshes) and [urdf](./urdf) files for the car sensor suite.

A tutorial on [How To Add A New Sensor](https://gitlab.com/eufs/eufs_sim/-/wikis/How-To-Add-A-New-Sensor) can be found on the wiki.

## Sensor Parameters

### IMU (both SBG and ZED camera)

| Name | Default | Purpose |
| ---- | ------- | ------- |
| prefix | -       | Specifies the prefix for the name of the sensor. Useful if multiple identical sensors are added. |
| parent | -       | Determines the parent link onto which the sensor will be attached. |
| origin | -       | Position of the IMU base link relative to the parent link. |
| noise  | 0.08    | Standard deviation of the Gaussian noise added to linear acceleration and angular velocity readings from IMU. |

The topic which the IMU publishes to is named by the convention `/{prefix}`

### GPS

| Name | Default | Purpose |
| ---- | ------- | ------- |
| prefix | -       | Specifies the prefix for the name of the sensor. Useful if multiple identical sensors are added. |
| parent | -       | Determines the parent link onto which the sensor will be mounted. |
| origin | -       | Position of the GPS base link relative to the parent link. |

The topic which the GPS publishes to is named by the convention `/{prefix}`

### VLP-16R

| Name | Default | Purpose |
| ---- | ------- | ------- |
| origin    | -                | Position of the Lidar base link relative to the parent link. |
| parent    | base_footprint   | Determines the parent link onto which the sensor will be attached. |
| name      | velodyne         | Name of sensor. Useful if multiple identical sensors are added. |
| active    | -                | Determines whether or not the sensor takes any measurements. |
| topic     | /velodyne_points | Topic to publish the sensor output. |
| hz        | 10               | Sets the sensor update rate. |
| lasers    | 40               | Sets the number of vertical spinning lasers. |
| samples   | 350              | Sets the number of horizontal rotating samples. |
| min_range | 0.2              | Minimum range in metres. |
| max_range | 100.0            | Maximum range in metres. |
| noise     | 0.008            | Gaussian noise value in metres. |
| min_angle | -2               | Minimum horizontal angle in radians. |
| max_angle | 2                | Maximum horizontal angle in radians. |

### ZED Camera

| Name | Default | Purpose |
| ---- | ------- | ------- |
| prefix | -       | Specifies the prefix for the name of the sensor. Useful if multiple identical sensors are added. |
| parent | -       | Determines the parent link onto which the sensor will be attached. |
| origin | -       | Position of the ZED Camera base link relative to the parent link. |
| active | -       | Determines whether or not the sensor takes any measurements. |

The topics which the ZED Camera publishes to are named as follows:

- `/zed/left/camera_info`
- `/zed/left/image_rect_color`
- `/zed/right/camera_info`
- `/zed/right/image_rect_color`
- `zed/depth/camera_info`
- `/zed/depth/image_raw`
- `/zed/camera_info`
- `/zed/image_raw`
- `/zed/points`

## all_sensors

[all_sensors.urdf.xacro](./urdf/all_sensors.urdf.xacro) should include all the sensors you wish to run on the car.

This xacro file can then be included in a `robot.urdf.xacro` file, where each sensor's parameters are defined.

For eufs_sim these are found in [eufs_racecar/robots](../eufs_racecar/robots).

## meshes

The meshes directory must be included in the `GAZEBO_RESOURCE_PATH` environment variable. This allows gazebo to render the sensors.

In eufs_sim we define this variable in our world launch files found in [eufs_tracks/launch](../eufs_tracks/launch).
