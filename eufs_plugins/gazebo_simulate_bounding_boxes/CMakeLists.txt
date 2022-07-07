add_library(gazebo_simulate_bounding_boxes SHARED
  src/gazebo_simulate_bounding_boxes.cpp)

ament_target_dependencies(gazebo_simulate_bounding_boxes
  "gazebo_dev"
  "gazebo_ros"
  "rclcpp"
  "sensor_msgs"
  "eufs_msgs"
  "image_geometry")

target_include_directories(gazebo_simulate_bounding_boxes PUBLIC include)

ament_export_libraries(gazebo_simulate_bounding_boxes)

install(DIRECTORY config DESTINATION share/${PROJECT_NAME} FILES_MATCHING PATTERN "*.yaml")

install(TARGETS gazebo_simulate_bounding_boxes
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib)
