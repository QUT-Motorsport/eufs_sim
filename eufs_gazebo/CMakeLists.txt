cmake_minimum_required(VERSION 2.8.3)
project(eufs_gazebo)

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  eufs_description
  ros_can_sim
  eufs_msgs
  roscpp
  rospy
  std_msgs
  std_srvs
  tf
  roslint
)

catkin_package()

install(DIRECTORY launch/
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
   FILES_MATCHING PATTERN "*.launch" PATTERN "*.machine" PATTERN "*.yaml" PATTERN "*.urdf"
)

install(DIRECTORY config/
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
   FILES_MATCHING PATTERN "*.yaml"
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

#############
## Roslint ##
#############

roslint_python()
# roslint_cpp() # removed as there currently is no cpp in this package

