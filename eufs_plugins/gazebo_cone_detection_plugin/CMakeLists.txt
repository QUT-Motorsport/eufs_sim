cmake_minimum_required(VERSION 3.5)

# ground truth cones plugin
add_library(gazebo_ros_cone_detection SHARED
  src/gazebo_ros_cone_detection.cpp)

ament_target_dependencies(gazebo_ros_cone_detection
  "gazebo_dev"
  "gazebo_ros"
  "rclcpp"
  "std_srvs"
  "geometry_msgs"
  "ament_index_cpp"
  "driverless_msgs")

target_include_directories(gazebo_ros_cone_detection PUBLIC include)
target_include_directories(gazebo_ros_cone_detection PUBLIC ${PROJECT_SOURCE_DIR}/include)
ament_export_libraries(gazebo_ros_cone_detection)

install(TARGETS gazebo_ros_cone_detection
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib
)
