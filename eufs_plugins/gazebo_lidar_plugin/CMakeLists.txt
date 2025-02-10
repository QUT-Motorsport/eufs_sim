cmake_minimum_required(VERSION 3.5)

add_library(gazebo_ros_lidar SHARED
  src/gazebo_ros_lidar.cpp)

# target_link_libraries(gazebo_ros_lidar
#   PRIVATE
#     gz-sensors8::gz-sensors8
#     gz-plugin2::core
# )
  
ament_target_dependencies(gazebo_ros_lidar
  # "gazebo_dev"
  "ros_gz"
  "rclcpp"
  "sensor_msgs"
  "gz-transport13"
  "gz-common5"
  "gz-plugin2"
  "gz-sensors8"
)

target_include_directories(gazebo_ros_lidar PUBLIC include)
ament_export_libraries(gazebo_ros_lidar)

install(TARGETS gazebo_ros_lidar
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
