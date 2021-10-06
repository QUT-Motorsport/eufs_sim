add_library(gazebo_cone_ground_truth SHARED
  src/gazebo_cone_ground_truth.cpp)

ament_target_dependencies(gazebo_cone_ground_truth
  "gazebo_dev"
  "gazebo_ros"
  "rclcpp"
  "std_srvs"
  "eufs_msgs"
  "visualization_msgs"
  "geometry_msgs"
  "ament_index_cpp")

target_include_directories(gazebo_cone_ground_truth PUBLIC include)

ament_export_libraries(gazebo_cone_ground_truth)

install(TARGETS gazebo_cone_ground_truth
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib)
