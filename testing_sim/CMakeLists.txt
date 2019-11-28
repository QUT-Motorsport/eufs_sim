cmake_minimum_required(VERSION 2.8.3)
project(testing_sim)

find_package(catkin REQUIRED)

find_package(rostest REQUIRED)
add_rostest(launch/test_simulation.launch)
