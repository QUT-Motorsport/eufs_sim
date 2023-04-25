cmake_minimum_required(VERSION 3.5)

add_library(gazebo_ros_sbg_driver SHARED
  src/gazebo_ros_sbg_driver.cpp)

ament_target_dependencies(gazebo_ros_sbg_driver
  "gazebo_dev"
  "gazebo_ros"
  "rclcpp"
  "nav_msgs"
  "geometry_msgs"
  "sensor_msgs"
  "std_msgs"
  "sbg_driver"
)

target_include_directories(gazebo_ros_sbg_driver PUBLIC include)
target_link_libraries(gazebo_ros_sbg_driver eufs_models::eufs_models)
ament_export_libraries(gazebo_ros_sbg_driver)

install(TARGETS gazebo_ros_sbg_driver
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib)
