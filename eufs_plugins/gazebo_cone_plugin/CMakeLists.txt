install(DIRECTORY config DESTINATION share/${PROJECT_NAME} FILES_MATCHING PATTERN "*.yaml")

# ground truth cones plugin
add_library(gazebo_ros_cone_gen SHARED
  src/gazebo_ros_cone_gen.cpp)

ament_target_dependencies(gazebo_ros_cone_gen
  "gazebo_dev"
  "gazebo_ros"
  "rclcpp"
  "std_srvs"
  "eufs_msgs"
  "geometry_msgs"
  "ament_index_cpp"
  "driverless_msgs")

target_include_directories(gazebo_ros_cone_gen PUBLIC include)

ament_export_libraries(gazebo_ros_cone_gen)

install(TARGETS gazebo_ros_cone_gen
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib)
