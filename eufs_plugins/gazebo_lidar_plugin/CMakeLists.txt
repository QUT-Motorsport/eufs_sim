find_package(gz-sim8 REQUIRED)
find_package(gz-plugin2 REQUIRED)
find_package(gz-rendering8 REQUIRED)
find_package(gz-transport13 REQUIRED)
find_package(gz-sensors8 REQUIRED COMPONENTS lidar)

add_library(gazebo_ros_lidar SHARED
  src/gazebo_ros_lidar.cpp
)

ament_target_dependencies(gazebo_ros_lidar
  ros_gz
  rclcpp
  sensor_msgs
)

target_include_directories(gazebo_ros_lidar PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${PROJECT_SOURCE_DIR}/include
)

target_link_libraries(gazebo_ros_lidar
  gz-sim8::gz-sim8
  gz-plugin2::gz-plugin2
  gz-rendering8::gz-rendering8
  gz-transport13::gz-transport13
  gz-sensors8::lidar
)

ament_export_libraries(gazebo_ros_lidar)

install(TARGETS gazebo_ros_lidar
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)