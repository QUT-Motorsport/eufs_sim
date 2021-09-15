add_library(gazebo_race_car_model SHARED
  src/gazebo_ros_race_car_model.cpp
  src/state_machine.cpp)

ament_target_dependencies(gazebo_race_car_model
  "gazebo_dev"
  "gazebo_ros"
  "rclcpp"
  "eufs_msgs"
  "nav_msgs"
  "geometry_msgs"
  "tf2"
  "tf2_ros"
  "std_srvs"
  "std_msgs"
  "eufs_models")

target_include_directories(gazebo_race_car_model PUBLIC include)
target_link_libraries(gazebo_race_car_model eufs_models::eufs_models)
ament_export_libraries(gazebo_race_car_model)

install(TARGETS gazebo_race_car_model
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib)
