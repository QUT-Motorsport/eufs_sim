cmake_minimum_required(VERSION 2.8.3)
project(eufs_msgs)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  message_generation
  genmsg
  std_msgs
  sensor_msgs
  geometry_msgs
  diagnostic_msgs
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)
find_package(Boost REQUIRED thread)


################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

add_message_files(
  DIRECTORY msg
  FILES
  wheelSpeeds.msg
  stateEstimatorStatus.msg
  FilterOutput.msg
  canState.msg
  systemState.msg
  driveCommand.msg
  driveCommand2.msg
  driveCommand4.msg
  chassisState.msg
  lapStats.msg
  pathIntegralParams.msg
  pathIntegralStats.msg
  fullState.msg
)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

generate_messages(DEPENDENCIES
  std_msgs
  sensor_msgs
  geometry_msgs
  diagnostic_msgs
)

catkin_package(CATKIN_DEPENDS
  message_runtime
  genmsg
  std_msgs
  sensor_msgs
  geometry_msgs
  diagnostic_msgs
)
