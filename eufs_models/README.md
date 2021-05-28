# EUFS Vehicle Models Package

This package contains a library (`eufs_models`). This library implements numerous vehicle model used by the [Edinburgh University Formulat Student AI team](https://eufs.eusa.ed.ac.uk/ai). The current implemented vehicle models can be found within the `src` directory.

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
