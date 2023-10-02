cmake_minimum_required(VERSION 3.5)

add_library(gazebo_ros_steering_motor SHARED
  src/gazebo_ros_steering_motor.cpp)

ament_target_dependencies(gazebo_ros_steering_motor
  "gazebo_dev"
  "gazebo_ros"
  "rclcpp"
  "std_msgs"
  "driverless_msgs"
)

target_include_directories(gazebo_ros_steering_motor PUBLIC include)
target_include_directories(gazebo_ros_steering_motor PUBLIC ${PROJECT_SOURCE_DIR}/include)
ament_export_libraries(gazebo_ros_steering_motor)

install(TARGETS gazebo_ros_steering_motor
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib
)
