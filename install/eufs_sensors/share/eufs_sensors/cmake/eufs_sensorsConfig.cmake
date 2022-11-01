# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_eufs_sensors_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED eufs_sensors_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(eufs_sensors_FOUND FALSE)
  elseif(NOT eufs_sensors_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(eufs_sensors_FOUND FALSE)
  endif()
  return()
endif()
set(_eufs_sensors_CONFIG_INCLUDED TRUE)

# output package information
if(NOT eufs_sensors_FIND_QUIETLY)
  message(STATUS "Found eufs_sensors: 2.0.0 (${eufs_sensors_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'eufs_sensors' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${eufs_sensors_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(eufs_sensors_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "ament_cmake_export_dependencies-extras.cmake")
foreach(_extra ${_extras})
  include("${eufs_sensors_DIR}/${_extra}")
endforeach()
