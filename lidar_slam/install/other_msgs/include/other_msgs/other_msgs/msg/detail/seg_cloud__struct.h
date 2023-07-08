﻿// NOLINT: This file starts with a BOM since it contain non-ASCII characters
// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from other_msgs:msg/SegCloud.idl
// generated code does not contain a copyright notice

#ifndef OTHER_MSGS__MSG__DETAIL__SEG_CLOUD__STRUCT_H_
#define OTHER_MSGS__MSG__DETAIL__SEG_CLOUD__STRUCT_H_

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
// Member 'seg_cloud'
// Member 'ground_cloud'
#include "other_msgs/msg/detail/point__struct.h"
// Member 'seg_range'
// Member 'ground_range'
#include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in msg/SegCloud in the package other_msgs.
typedef struct other_msgs__msg__SegCloud
{
  std_msgs__msg__Header header;
  /// 聚类点
  other_msgs__msg__Point__Sequence seg_cloud;
  rosidl_runtime_c__float__Sequence seg_range;
  /// 地面点
  other_msgs__msg__Point__Sequence ground_cloud;
  rosidl_runtime_c__float__Sequence ground_range;
} other_msgs__msg__SegCloud;

// Struct for a sequence of other_msgs__msg__SegCloud.
typedef struct other_msgs__msg__SegCloud__Sequence
{
  other_msgs__msg__SegCloud * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} other_msgs__msg__SegCloud__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // OTHER_MSGS__MSG__DETAIL__SEG_CLOUD__STRUCT_H_