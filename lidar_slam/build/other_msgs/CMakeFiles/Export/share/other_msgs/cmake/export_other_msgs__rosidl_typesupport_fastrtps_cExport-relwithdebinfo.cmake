#----------------------------------------------------------------
# Generated CMake target import file for configuration "RelWithDebInfo".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "other_msgs::other_msgs__rosidl_typesupport_fastrtps_c" for configuration "RelWithDebInfo"
set_property(TARGET other_msgs::other_msgs__rosidl_typesupport_fastrtps_c APPEND PROPERTY IMPORTED_CONFIGURATIONS RELWITHDEBINFO)
set_target_properties(other_msgs::other_msgs__rosidl_typesupport_fastrtps_c PROPERTIES
  IMPORTED_LOCATION_RELWITHDEBINFO "${_IMPORT_PREFIX}/lib/libother_msgs__rosidl_typesupport_fastrtps_c.so"
  IMPORTED_SONAME_RELWITHDEBINFO "libother_msgs__rosidl_typesupport_fastrtps_c.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS other_msgs::other_msgs__rosidl_typesupport_fastrtps_c )
list(APPEND _IMPORT_CHECK_FILES_FOR_other_msgs::other_msgs__rosidl_typesupport_fastrtps_c "${_IMPORT_PREFIX}/lib/libother_msgs__rosidl_typesupport_fastrtps_c.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
