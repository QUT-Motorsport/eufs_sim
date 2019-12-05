cmake_minimum_required(VERSION 2.8.3)
project(eufs_gazebo_plugins)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
        gazebo_dev
        message_generation
        gazebo_msgs
        roscpp
        nodelet
        geometry_msgs
        nav_msgs
        eufs_msgs
        tf2_ros
        )

find_package(Boost REQUIRED COMPONENTS thread)
find_package(gazebo REQUIRED)

include_directories(include
        ${catkin_INCLUDE_DIRS}
        ${GAZEBO_INCLUDE_DIRS}
        ${SDFormat_INCLUDE_DIRS}
        ${OGRE_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIRS})

link_directories(
        ${catkin_LIBRARY_DIRS}
        ${OGRE_LIBRARY_DIRS}
        ${OGRE-Terrain_LIBRARY_DIRS}
        ${OGRE-Paging_LIBRARY_DIRS})

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES gazebo_state_ground_truth
        CATKIN_DEPENDS
        message_runtime
        gazebo_msgs
        roscpp
        nodelet
        geometry_msgs
        nav_msgs
        eufs_msgs
        tf2_ros
)

# Add single plugin
add_library(gazebo_state_ground_truth src/gazebo_state_ground_truth.cpp)
target_include_directories(gazebo_state_ground_truth PUBLIC include ${CMAKE_SOURCE_DIR} ${catkin_LIBRARY_DIRS})
target_link_libraries(gazebo_state_ground_truth ${GAZEBO_LIBRARIES} ${IGNITION-MSGS_LIBRARIES} ${catkin_LIBRARIES} ${OGRE_LIBRARY})

install(TARGETS gazebo_state_ground_truth
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

# Tests
# These need to be run with -j1 flag because gazebo can't be run
# in parallel.
#if (CATKIN_ENABLE_TESTING)
#    find_package(rostest REQUIRED)
#    # TODO
#endif ()