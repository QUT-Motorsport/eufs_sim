cmake_minimum_required(VERSION 3.5)

add_library(gazebo_sbg_plugin SHARED
  src/gazebo_ros_sbg.cpp)

find_package(gazebo_dev REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sbg_driver REQUIRED)

ament_target_dependencies(gazebo_sbg_plugin
  "gazebo_dev"
  "gazebo_ros"
  "rclcpp"
  "nav_msgs"
  "geometry_msgs"
  "tf2"
  "tf2_ros"
  "tf2_geometry_msgs"
  "std_msgs"
  "sbg_driver"
)

target_include_directories(gazebo_sbg_plugin PUBLIC include)
target_link_libraries(gazebo_sbg_plugin eufs_models::eufs_models)
ament_export_libraries(gazebo_sbg_plugin)

install(TARGETS gazebo_sbg_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib)
