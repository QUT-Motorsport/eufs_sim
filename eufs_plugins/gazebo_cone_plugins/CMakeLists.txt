install(DIRECTORY config DESTINATION share/${PROJECT_NAME} FILES_MATCHING PATTERN "*.yaml")

# ground truth cones plugin
add_library(gazebo_ground_truth_cones SHARED
  src/gazebo_ground_truth_cones.cpp)

ament_target_dependencies(gazebo_ground_truth_cones
  "gazebo_dev"
  "gazebo_ros"
  "rclcpp"
  "std_srvs"
  "eufs_msgs"
  "geometry_msgs"
  "ament_index_cpp")

target_include_directories(gazebo_ground_truth_cones PUBLIC include)

ament_export_libraries(gazebo_ground_truth_cones)

install(TARGETS gazebo_ground_truth_cones
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib)

# camera cones plugin
add_library(gazebo_camera_cones SHARED
        src/gazebo_camera_cones.cpp)

ament_target_dependencies(gazebo_camera_cones
        "gazebo_dev"
        "gazebo_ros"
        "rclcpp"
        "eufs_msgs"
        "ament_index_cpp")

target_include_directories(gazebo_camera_cones PUBLIC include)

ament_export_libraries(gazebo_camera_cones)

install(TARGETS gazebo_camera_cones
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION lib)
