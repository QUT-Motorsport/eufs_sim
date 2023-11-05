cmake_minimum_required(VERSION 3.5)

add_library(gazebo_res SHARED
  src/gazebo_res.cpp)

ament_target_dependencies(gazebo_res
  "gazebo_dev"
  "gazebo_ros"
  "rclcpp"
  "std_msgs"
)

add_compile_definitions(QUTMS_CAN_RES)

target_include_directories(gazebo_res PUBLIC include)
target_include_directories(gazebo_res PUBLIC ${PROJECT_SOURCE_DIR}/include)
ament_export_libraries(gazebo_res)

install(TARGETS gazebo_res
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib
)
