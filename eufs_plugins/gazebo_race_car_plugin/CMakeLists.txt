cmake_minimum_required(VERSION 3.5)

add_library(gazebo_ros_race_car SHARED
  src/gazebo_ros_race_car.cpp
  src/state_machine.cpp)

find_package(gazebo_dev REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(eufs_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(eufs_models REQUIRED)

ament_target_dependencies(gazebo_ros_race_car
  "gazebo_dev"
  "gazebo_ros"
  "rclcpp"
  "eufs_msgs"
  "nav_msgs"
  "geometry_msgs"
  "tf2"
  "ackermann_msgs"
  "tf2_ros"
  "tf2_geometry_msgs"
  "std_srvs"
  "std_msgs"
  "eufs_models"
)

target_include_directories(gazebo_ros_race_car PUBLIC include)
target_link_libraries(gazebo_ros_race_car eufs_models::eufs_models)
ament_export_libraries(gazebo_ros_race_car)

install(TARGETS gazebo_ros_race_car
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib)
