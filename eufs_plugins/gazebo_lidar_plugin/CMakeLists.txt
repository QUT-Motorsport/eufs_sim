cmake_minimum_required(VERSION 3.5)

add_library(gazebo_ros_lidar SHARED
  src/gazebo_ros_lidar.cpp)

ament_target_dependencies(gazebo_ros_lidar
  "gazebo_dev"
  "gazebo_ros"
  "rclcpp"
  "sensor_msgs"
)

target_include_directories(gazebo_ros_lidar PUBLIC include)
ament_export_libraries(gazebo_ros_lidar)

install(TARGETS gazebo_ros_lidar
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
