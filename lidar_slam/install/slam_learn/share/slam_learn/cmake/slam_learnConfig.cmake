# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_slam_learn_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED slam_learn_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(slam_learn_FOUND FALSE)
  elseif(NOT slam_learn_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(slam_learn_FOUND FALSE)
  endif()
  return()
endif()
set(_slam_learn_CONFIG_INCLUDED TRUE)

# output package information
if(NOT slam_learn_FIND_QUIETLY)
  message(STATUS "Found slam_learn: 0.0.0 (${slam_learn_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'slam_learn' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${slam_learn_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(slam_learn_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${slam_learn_DIR}/${_extra}")
endforeach()
