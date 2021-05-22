# EUFS Vehicle Models Package

This package contains a library (`eufs_models`). This library implements numerous vehicle model used by the [Edinburgh University Formulat Student AI team](https://eufs.eusa.ed.ac.uk/ai). The current implemented vehicle models can be found within the `src` directory.

## Usage

To utilise the vehicle model library in your own packages, the `CMakeLists.txt` file will need to be properly configured. A proper configuration involves finding the `eufs_models` package which contains the vehicle model library and then linking the library to the executable:

```CMake
# find eufs_models package
find_package(eufs_models REQUIRED)

# link vehicle model library to target executable
target_link_libraries(${PROJECT_NAME}
  eufs_models::eufs_models
)
```

A minimal but complete `CMakeLists.txt` file is included below to illustrate what a properly configured `CMakeLists.txt` file might look like:

```CMake
cmake_minimum_required(VERSION 3.5)
project(eufs_test)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(eufs_models REQUIRED)

# create executable
add_executable(${PROJECT_NAME}
  src/test.cpp
)

ament_target_dependencies(${PROJECT_NAME} rclcpp)

target_link_libraries(${PROJECT_NAME}
  eufs_models::eufs_models
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# install executable
install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
```
