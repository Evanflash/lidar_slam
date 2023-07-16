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
// Member 'trans_form'
#include "rosidl_runtime_c/primitives_sequence.h"
// Member 'surf_flat'
// Member 'surf_less_flat'
// Member 'ground_flat'
// Member 'ground_less_flat'
#include "other_msgs/msg/detail/point__struct.h"

/// Struct defined in msg/AllCloud in the package other_msgs.
/**
  * header
 */
typedef struct other_msgs__msg__AllCloud
{
  std_msgs__msg__Header header;
  /// 6自由度参数，x y z qx qy qz
  rosidl_runtime_c__float__Sequence trans_form;
  /// 平面特征点
  other_msgs__msg__Point__Sequence surf_flat;
  /// 所有平面点
  other_msgs__msg__Point__Sequence surf_less_flat;
  /// 地面特征点
  other_msgs__msg__Point__Sequence ground_flat;
  /// 所有地面点
  other_msgs__msg__Point__Sequence ground_less_flat;
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
