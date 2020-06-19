cmake_minimum_required(VERSION 2.8.3)
project(ros_can_sim)

set(BUILD_FLAGS "-std=c++14 -Wuninitialized -Wall -Wextra -Wunused-parameter")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${BUILD_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
        roscpp
        tf
        rqt_gui
        rqt_gui_py
        ackermann_msgs
        geometry_msgs
        sensor_msgs
        std_msgs
        eufs_msgs
        std_srvs
        roslint
        )

catkin_package(
        #  INCLUDE_DIRS include
        #  LIBRARIES snap_vislam
        #  CATKIN_DEPENDS roscpp rospy std_msgs
        #  DEPENDS system_lib
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
        ${catkin_INCLUDE_DIRS}
)

# needed because of setup.py
catkin_python_setup()

install(PROGRAMS nodes/ros_can_sim_gui
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

install(PROGRAMS nodes/eufs_robot_steering_gui
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

#############
## Roslint ##
#############

roslint_python()
#roslint_cpp()
