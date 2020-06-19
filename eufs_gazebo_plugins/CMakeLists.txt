cmake_minimum_required(VERSION 3.5)
project(eufs_gazebo_plugins)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
        gazebo_msgs
        roscpp
        nodelet
        geometry_msgs
        nav_msgs
        ackermann_msgs
        eufs_msgs
        tf2_ros
        tf
        )

find_package(Boost REQUIRED COMPONENTS thread)
find_package(gazebo REQUIRED)

catkin_package(
        INCLUDE_DIRS gazebo_cone_ground_truth/include
        LIBRARIES gazebo_state_ground_truth
        CATKIN_DEPENDS
        gazebo_msgs
        roscpp
        nodelet
        geometry_msgs
        nav_msgs
        eufs_msgs
        ackermann_msgs
        tf2_ros
        tf
)

add_subdirectory(gazebo_cone_ground_truth)
add_subdirectory(gazebo_race_car_model)

# Tests
# These need to be run with -j1 flag because gazebo can't be run
# in parallel.
#if (CATKIN_ENABLE_TESTING)
#    find_package(rostest REQUIRED)
#    # TODO
#endif ()
