# EUFS Vehicle Models Package

This package contains a library (`eufs_models`). This library implements numerous vehicle model used by the [Edinburgh University Formulat Student AI team](https://eufs.eusa.ed.ac.uk/ai). The current implemented vehicle models can be found within the `src` directory.

## models.txt

The `models.txt` file contains the name of every vehicle model implemented in this library. Each name should be on a new line. This file is used by the `eufs_launcher` to know what vehicle models are available for use.

## Namespace

Everything in the `eufs_models` package is contains within the `eufs::models` namespace. For example, to initialize a `DynamicBicycle` object:

```C++
#include <string>
#include "eufs_models/dynamic_bicycle"

std::string yaml_file_path = "<path_to_yaml_config_file>";
eufs::models::DynamicBicycle dynamic_bicycle_model = eufs::models::DynamicBicycle(yaml_file_path);
```

## Vehicle Model

The `VehicleModel` class is a base class from which all implementations of vehicles models should inherit from. For examples, please take a look at the currently implemented models in the `src` directory (e.g `DynamicBicycle` or `PointMass`).

Inherited models must implement the `updateState` virtual method. This method is reponsible for updating the provided state (`state`) based on the provided input (`input`) and change in time (`dt`). If using with `eufs_plugins/gazebo_race_car_model`, this is the method that is used to update the state of the vehicle in the simulation.

The other important aspect of the base class is the `_param` private attribute. This stores the various vehicle parameters in the `Param` struct defined in `vehicle_param.hpp`. The struct is initialized by the `Param` constructor via the `VehicleModel` constructor. Initialization requires a path to a yaml file. When implementing vehicle models make sure that your constructor calls the `VehicleModel` constructor with the path to the yaml file that contains your vehicle model parameters.

## Noise

The `noise.hpp` header file defines a `Noise` class. This class implements one main method `applyNoise`. This method takes a `State` object and applies Gaussian noise to every `State` attribute. The exact Gaussian distribution depends on the configuration file provided to the `Noise` object during initialization.

An example configuration file can be found in the `config` directory.

## CMake Setup

To utilise the vehicle model library in your own packages, the `CMakeLists.txt` file will need to be properly configured. A proper configuration involves finding the `eufs_models` package which contains the vehicle model library and then linking the library to the executable:

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
