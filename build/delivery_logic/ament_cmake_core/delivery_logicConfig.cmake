# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_delivery_logic_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED delivery_logic_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(delivery_logic_FOUND FALSE)
  elseif(NOT delivery_logic_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(delivery_logic_FOUND FALSE)
  endif()
  return()
endif()
set(_delivery_logic_CONFIG_INCLUDED TRUE)

# output package information
if(NOT delivery_logic_FIND_QUIETLY)
  message(STATUS "Found delivery_logic: 0.0.0 (${delivery_logic_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'delivery_logic' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${delivery_logic_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(delivery_logic_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${delivery_logic_DIR}/${_extra}")
endforeach()
