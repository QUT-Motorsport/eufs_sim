cmake_minimum_required(VERSION 3.5)
project(eufs_gazebo_plugins)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)

find_package(eufs_msgs REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)

find_package(tf2_ros REQUIRED)
find_package(tf REQUIRED)

find_package(nodelet REQUIRED)

find_package(Boost REQUIRED COMPONENTS thread)
find_package(gazebo REQUIRED)

ament_export_dependencies(
  gazebo_msgs
  rclcpp
  nodelet
  geometry_msgs
  nav_msgs
  eufs_msgs
  ackermann_msgs
  tf2_ros
  tf
)

ament_export_include_directories(
  gazebo_cone_ground_truth/include
)

ament_export_libraries(gazebo_state_ground_truth)

add_subdirectory(gazebo_cone_ground_truth)
add_subdirectory(gazebo_race_car_model)

ament_package()
