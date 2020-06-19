cmake_minimum_required(VERSION 3.5)
project(eufs_launcher)

add_compile_options(-std=c++11)


find_package(Boost REQUIRED)
# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
  roscpp
  gazebo_ros
  roslint
)

# Depend on system install of Gazebo
find_package(gazebo REQUIRED)

catkin_python_setup()

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})

catkin_package(
  DEPENDS
    roscpp
    gazebo_ros
)

install(PROGRAMS scripts/rqt_eufs_launcher.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(PROGRAMS scripts/rqt_eufs_sketcher.py
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

# Uncomment the line below when C++ files are added to this package
# roslint_cpp()
