cmake_minimum_required(VERSION 3.5)
project(eufs_gazebo)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

find_package(gazebo_ros REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_py REQUIRED)
find_package(xacro REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

install(PROGRAMS
  nodes/sbg_raw_data_simulator.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY tracks worlds
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.launch" PATTERN "*.launch.py" PATTERN "*.txt"
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
