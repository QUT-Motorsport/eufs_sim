cmake_minimum_required(VERSION 2.8.3)
project(robot_control)

set(BUILD_FLAGS "-std=c++14 -Wuninitialized -Wall -Wextra -Wunused-parameter")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${BUILD_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  ackermann_msgs
  geometry_msgs
  nav_msgs
  roscpp
  sensor_msgs
  tf
        std_msgs
        eufs_msgs
        rqt_gui
        std_srvs
        rqt_gui_py
)

catkin_package(
 CATKIN_DEPENDS
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_python_setup()

add_executable(robot_control_node src/ros_can_sim.cpp)

target_link_libraries(robot_control_node
   ${catkin_LIBRARIES}
)

install(PROGRAMS nodes/ros_can_sim_gui
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

install(DIRECTORY
        resource
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        )

install(FILES
        plugin.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        )


