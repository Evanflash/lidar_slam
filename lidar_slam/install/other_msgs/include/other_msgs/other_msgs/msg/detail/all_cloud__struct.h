// NOLINT: This file starts with a BOM since it contain non-ASCII characters
// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from other_msgs:msg/AllCloud.idl
// generated code does not contain a copyright notice

#ifndef OTHER_MSGS__MSG__DETAIL__ALL_CLOUD__STRUCT_H_
#define OTHER_MSGS__MSG__DETAIL__ALL_CLOUD__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'corner_sharp'
// Member 'corner_less_sharp'
// Member 'surf_flat'
// Member 'surf_less_flat'
// Member 'full_point_res'
#include "other_msgs/msg/detail/point__struct.h"

/// Struct defined in msg/AllCloud in the package other_msgs.
/**
  * header
 */
typedef struct other_msgs__msg__AllCloud
{
  std_msgs__msg__Header header;
  /// 曲率最大的点
  other_msgs__msg__Point__Sequence corner_sharp;
  /// 曲率次大的点
  other_msgs__msg__Point__Sequence corner_less_sharp;
  /// 曲率最小的点
  other_msgs__msg__Point__Sequence surf_flat;
  /// 曲率次小的点
  other_msgs__msg__Point__Sequence surf_less_flat;
  /// 所有点
  other_msgs__msg__Point__Sequence full_point_res;
} other_msgs__msg__AllCloud;

// Struct for a sequence of other_msgs__msg__AllCloud.
typedef struct other_msgs__msg__AllCloud__Sequence
{
  other_msgs__msg__AllCloud * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} other_msgs__msg__AllCloud__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // OTHER_MSGS__MSG__DETAIL__ALL_CLOUD__STRUCT_H_
