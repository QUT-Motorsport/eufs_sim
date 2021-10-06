cmake_minimum_required(VERSION 3.5)
project(eufs_models)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -fPIC)
endif()

# Global flag to cause add_library() to create shared libraries if on.
# If set to true, this will cause all libraries to be built shared
# unless the library was explicitly added as a static library.
option(BUILD_SHARED_LIBS ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(eufs_msgs REQUIRED)

# build library
add_library(${PROJECT_NAME}
  src/dynamic_bicycle.cpp
  src/point_mass.cpp
  src/vehicle_model.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>")
target_link_libraries(eufs_models
  ${YAML_CPP_LIBRARIES})
ament_target_dependencies(eufs_models
  "eufs_msgs"
  "yaml-cpp")

# export library
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

ament_export_dependencies(ament_cmake)
ament_export_dependencies(yaml-cpp)
ament_export_dependencies(eufs_msgs)

install(
  DIRECTORY include/
  DESTINATION include)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include)

ament_package()

install(FILES models.txt
  DESTINATION share/eufs_models/models)

install(FILES config/noise.yaml
  DESTINATION share/eufs_models/config)