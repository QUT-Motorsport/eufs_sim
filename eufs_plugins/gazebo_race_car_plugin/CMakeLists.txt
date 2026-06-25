add_library(gazebo_ros_race_car SHARED
  src/gazebo_ros_race_car.cpp
)

find_package(gz-sim8 REQUIRED)
find_package(gz-plugin2 REQUIRED)
find_package(gz-transport13 REQUIRED)

ament_target_dependencies(gazebo_ros_race_car
  ros_gz
  rclcpp
  nav_msgs
  geometry_msgs
  tf2
  ackermann_msgs
  tf2_ros
  tf2_geometry_msgs
  std_srvs
  std_msgs
  sensor_msgs
  eufs_models
  driverless_msgs
)

target_include_directories(gazebo_ros_race_car PUBLIC
  include
  ${PROJECT_SOURCE_DIR}/include
)

target_link_libraries(gazebo_ros_race_car
  gz-plugin2::gz-plugin2
  gz-sim8::gz-sim8
  gz-transport13::gz-transport13
  eufs_models::eufs_models
)

ament_export_libraries(gazebo_ros_race_car)

install(TARGETS gazebo_ros_race_car
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib
)
