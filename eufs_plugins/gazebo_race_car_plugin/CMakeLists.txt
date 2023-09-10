cmake_minimum_required(VERSION 3.5)

add_library(gazebo_ros_race_car SHARED
  src/gazebo_ros_race_car.cpp)

ament_target_dependencies(gazebo_ros_race_car
  "gazebo_dev"
  "gazebo_ros"
  "rclcpp"
  "nav_msgs"
  "geometry_msgs"
  "tf2"
  "ackermann_msgs"
  "tf2_ros"
  "tf2_geometry_msgs"
  "std_srvs"
  "std_msgs"
  "sensor_msgs"
  "eufs_models"
  "driverless_msgs"
)

target_include_directories(gazebo_ros_race_car PUBLIC include)
target_include_directories(gazebo_ros_race_car PUBLIC ${PROJECT_SOURCE_DIR}/include)
target_link_libraries(gazebo_ros_race_car eufs_models::eufs_models)
ament_export_libraries(gazebo_ros_race_car)

install(TARGETS gazebo_ros_race_car
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib
)
