# eufs_models

This package contains a library (`eufs_models`). This library implements numerous vehicle models. The current implemented vehicle models can be found 
within the [src](./src) directory.

An outline of the library design can be found [here](https://gitlab.com/eufs/eufs_sim/-/wikis/Library-Design).
There are also wiki pages containing information on the physical models used in 
[Point Mass](https://gitlab.com/eufs/eufs_sim/-/wikis/Point-Mass) and 
[Dynamic Bicycle](https://gitlab.com/eufs/eufs_sim/-/wikis/Dynamic-Bicycle).

## models.txt

The [models.txt](./models.txt) file contains the name of every vehicle model implemented in this library.
Each name should be on a new line. This file is used by the [eufs_launcher](../eufs_launcher/README.md)
to determine which vehicle models are available for use.

## Namespace

Everything in the `eufs_models` package is contained within the `eufs::models` namespace.
For example, to initialize a `DynamicBicycle` object:

```C++
#include <string>
#include "eufs_models/dynamic_bicycle"

std::string yaml_file_path = "<path_to_yaml_config_file>";
eufs::models::DynamicBicycle dynamic_bicycle_model = eufs::models::DynamicBicycle(yaml_file_path);
```

## Vehicle Model

The `VehicleModel` class is a base class from which all implementations of vehicle models should inherit from.
For examples, please take a look at the currently implemented models in the [src](./src) directory
(e.g `DynamicBicycle` or `PointMass`).

Inherited models must implement the `updateState` virtual method. This method is reponsible for updating the provided
state (`state`) based on the provided input (`input`) and change in time (`dt`). If using with
[gazebo_ros_race_car_model](../eufs_plugins/gazebo_race_car_model/src/gazebo_ros_race_car_model.cpp),
this is the method that is used to update the state of the vehicle in the simulation.

The other important aspect of the base class is the `_param` private attribute.
This stores the various vehicle parameters in the `Param` struct defined in [vehicle_param.hpp](./include/eufs_models/vehicle_param.hpp).
The struct is initialized by the `Param` constructor via the `VehicleModel` constructor.
Initialization requires a path to a yaml file. When implementing vehicle models make sure that
your constructor calls the `VehicleModel` constructor with the path to the yaml file that
contains your vehicle model parameters.

A vehicle model object will allow public use of the following methods:

| Name | input(s) | output | Purpose |
| ---- | ----- | ------ | ------- |
| `getWheelSpeeds` | [State](./include/eufs_models/vehicle_state.hpp), [Input](./include/eufs_models/vehicle_input.hpp) | [eufs_msgs/WheelSpeeds](https://gitlab.com/eufs/eufs_msgs/-/blob/ros2/msg/WheelSpeeds.msg) | Calculates the wheel speeds (rpm) based on the vehicle velocity. |
| `getSlipAngle` | [State](./include/eufs_models/vehicle_state.hpp), [Input](./include/eufs_models/vehicle_input.hpp), bool (is front?) | double | Gets the [slip angle](https://en.wikipedia.org/wiki/Slip_angle) (radians). |
| `validateInput` | [Input](./include/eufs_models/vehicle_input.hpp) | void | Ensures that the vehicle steering angle, velocity and acceleration values are never above their maximum or below their minimum. If they are, the values are clipped. |
| `validateState` | [State](./include/eufs_models/vehicle_state.hpp) | void | Ensures that the linear velocity is always greater than zero. If not, it is set to 0. |
| `updateState` | [State](./include/eufs_models/vehicle_state.hpp), [Input](./include/eufs_models/vehicle_input.hpp), double (timestep) | void | Updates then validates the current vehicle [State](./include/eufs_models/vehicle_state.hpp) based on the model dynamics and the (validated) command [input](./include/eufs_models/vehicle_input.hpp). |


## Noise

The [noise.hpp](./include/eufs_models/noise.hpp) header file defines a `Noise` class. This class implements two main methods: `applyNoise` and
`applyNoiseToWheelSpeeds`. The first of these methods takes a [State](./include/eufs_models/vehicle_state.hpp) object and applies Gaussian noise to every
[State](./include/eufs_models/vehicle_state.hpp) attribute. The second takes a [eufs_msgs/WheelSpeeds](https://gitlab.com/eufs/eufs_msgs/-/blob/ros2/msg/WheelSpeeds.msg) message and applies Gaussian noise to each individual wheel
speed. The exact Gaussian distribution depends on the [configuration file](./config/noise.yaml) provided to the `Noise` object during
initialization.

## CMake Setup

To utilise the vehicle model library in your own packages, the `CMakeLists.txt` file will need to be properly
configured. A proper configuration involves finding the `eufs_models` package which contains the vehicle model library
and then linking the library to the executable:

```CMake
# find eufs_models package
find_package(eufs_models REQUIRED)

# link vehicle model library to target executable
target_link_libraries(${PROJECT_NAME}
  eufs_models::eufs_models
)
```

If the above doesn't work for you, the following alternative may:

```CMake
find_package(eufs_models REQUIRED)

# Directories to look for *.h and *.hpp file in
include_directories(
  include
    ${eufs_models_INCLUDE_DIRS}
)

# Target Libraries that do not support ament
target_link_libraries(${PROJECT_NAME}
  ${eufs_models_LIBRARIES}
)
```

See [here](../eufs_plugins/gazebo_race_car_model/CMakeLists.txt) for an example of setting up this library.
