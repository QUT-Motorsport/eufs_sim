find_package(gz-sim8 REQUIRED)
find_package(gz-plugin2 REQUIRED)
find_package(gz-transport13 REQUIRED)

add_library(gazebo_ros_cone_detection SHARED
  src/gazebo_ros_cone_detection.cpp
)

ament_target_dependencies(gazebo_ros_cone_detection
  ros_gz
  rclcpp
  std_srvs
  geometry_msgs
  ament_index_cpp
  driverless_msgs
)

target_include_directories(gazebo_ros_cone_detection PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${PROJECT_SOURCE_DIR}/include
)

target_link_libraries(gazebo_ros_cone_detection
  gz-sim8::gz-sim8
  gz-plugin2::gz-plugin2
  gz-transport13::gz-transport13
)

ament_export_libraries(gazebo_ros_cone_detection)

install(TARGETS gazebo_ros_cone_detection
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)