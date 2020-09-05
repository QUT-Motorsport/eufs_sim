cmake_minimum_required(VERSION 3.5)
project(ros_can_sim)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)

find_package(eufs_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

find_package(std_srvs REQUIRED)

find_package(rqt_gui REQUIRED)
find_package(rqt_gui_py REQUIRED)

find_package(tf REQUIRED)

find_package(roslint REQUIRED)

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
endif()

#ament_package()

install(PROGRAMS nodes/ros_can_sim_gui
        DESTINATION lib/${PROJECT_NAME}
        )

install(PROGRAMS nodes/eufs_robot_steering_gui
        DESTINATION lib/${PROJECT_NAME}
        )

install(DIRECTORY
        resource
        DESTINATION share/${PROJECT_NAME}
        )

install(FILES
        plugin.xml
        DESTINATION share/${PROJECT_NAME}
        )