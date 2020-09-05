cmake_minimum_required(VERSION 3.5)
project(eufs_launcher)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)

find_package(roslint REQUIRED)

find_package(gazebo REQUIRED)
find_package(gazebo_ros REQUIRED)

find_package(Boost REQUIRED)

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(${Boost_INCLUDE_DIR} ${GAZEBO_INCLUDE_DIRS})

ament_export_dependencies(
  rclcpp
  gazebo_ros
)

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
endif()

install(PROGRAMS scripts/rqt_eufs_launcher.py
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS scripts/rqt_eufs_sketcher.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  resource
  DESTINATION share
)

install(FILES
  plugin.xml
  DESTINATION share
)

ament_package()
