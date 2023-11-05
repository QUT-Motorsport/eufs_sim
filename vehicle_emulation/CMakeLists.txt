cmake_minimum_required(VERSION 3.5)
project(vehicle_emulation)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find includes in corresponding build directories
set(CMAKE_INCLUDE_CURRENT_DIR ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(qt_gui_cpp REQUIRED)
find_package(rqt_gui_cpp REQUIRED)
find_package(Qt5Widgets REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

# add
include_directories(
  include
)

set(src_files
  src/input_pub_node.cpp
  src/input_control_gui.cpp
)

set(hd_files
  include/${PROJECT_NAME}/input_control_gui.hpp
)

set(ui_files
  ui/input_control_gui.ui
)

qt5_wrap_cpp(moc_files ${hd_files})
qt5_wrap_ui(ui_hd_files ${ui_files})

set(dependencies
  rclcpp
  qt_gui_cpp
  rqt_gui_cpp
  Qt5Widgets
  std_msgs
  std_srvs
)

add_library(${PROJECT_NAME} SHARED
  ${src_files}
  ${moc_files}
  ${ui_hd_files}
)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})

# install
install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib/${PROJECT_NAME}
  RUNTIME DESTINATION bin/${PROJECT_NAME}
  INCLUDES DESTINATION include
)

install(DIRECTORY include/
  DESTINATION include/
)

install(PROGRAMS scripts/input_pub_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY ui
  DESTINATION share/${PROJECT_NAME}
)

install(FILES plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

SET_SOURCE_FILES_PROPERTIES(${SOURCES} PROPERTIES LANGUAGE CXX )

# Export
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME} ${SOURCES})
pluginlib_export_plugin_description_file(rqt_gui "plugin.xml")
ament_export_dependencies(${dependencies})
ament_package()
