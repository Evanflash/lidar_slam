// NOLINT: This file starts with a BOM since it contain non-ASCII characters
// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from other_msgs:msg/Point.idl
// generated code does not contain a copyright notice

#ifndef OTHER_MSGS__MSG__DETAIL__POINT__STRUCT_H_
#define OTHER_MSGS__MSG__DETAIL__POINT__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/Point in the package other_msgs.
/**
  * 一个点
 */
typedef struct other_msgs__msg__Point
{
  float x;
  float y;
  float z;
  float i;
} other_msgs__msg__Point;

// Struct for a sequence of other_msgs__msg__Point.
typedef struct other_msgs__msg__Point__Sequence
{
  other_msgs__msg__Point * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} other_msgs__msg__Point__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // OTHER_MSGS__MSG__DETAIL__POINT__STRUCT_H_
