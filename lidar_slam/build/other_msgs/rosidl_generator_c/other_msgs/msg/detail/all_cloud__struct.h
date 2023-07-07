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
// Member 'cornersharp'
// Member 'cornerlesssharp'
// Member 'surfflat'
// Member 'surflessflat'
// Member 'fullpointres'
#include "other_msgs/msg/detail/point__struct.h"

/// Struct defined in msg/AllCloud in the package other_msgs.
/**
  * header
 */
typedef struct other_msgs__msg__AllCloud
{
  std_msgs__msg__Header header;
  other_msgs__msg__Point__Sequence cornersharp;
  other_msgs__msg__Point__Sequence cornerlesssharp;
  other_msgs__msg__Point__Sequence surfflat;
  other_msgs__msg__Point__Sequence surflessflat;
  other_msgs__msg__Point__Sequence fullpointres;
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
