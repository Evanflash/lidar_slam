// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from other_msgs:msg/SegCloud.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "other_msgs/msg/detail/seg_cloud__rosidl_typesupport_introspection_c.h"
#include "other_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "other_msgs/msg/detail/seg_cloud__functions.h"
#include "other_msgs/msg/detail/seg_cloud__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `seg_ring_str_ind`
// Member `seg_ring_end_ind`
// Member `seg_range`
// Member `grd_ring_str_ind`
// Member `grd_ring_end_ind`
// Member `ground_range`
#include "rosidl_runtime_c/primitives_sequence_functions.h"
// Member `seg_cloud`
// Member `ground_cloud`
#include "other_msgs/msg/point.h"
// Member `seg_cloud`
// Member `ground_cloud`
#include "other_msgs/msg/detail/point__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void other_msgs__msg__SegCloud__rosidl_typesupport_introspection_c__SegCloud_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  other_msgs__msg__SegCloud__init(message_memory);
}

void other_msgs__msg__SegCloud__rosidl_typesupport_introspection_c__SegCloud_fini_function(void * message_memory)
{
  other_msgs__msg__SegCloud__fini(message_memory);
}

size_t other_msgs__msg__SegCloud__rosidl_typesupport_introspection_c__size_function__SegCloud__seg_ring_str_ind(
  const void * untyped_member)
{
  const rosidl_runtime_c__int32__Sequence * member =
    (const rosidl_runtime_c__int32__Sequence *)(untyped_member);
  return member->size;
}

const void * other_msgs__msg__SegCloud__rosidl_typesupport_introspection_c__get_const_function__SegCloud__seg_ring_str_ind(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__int32__Sequence * member =
    (const rosidl_runtime_c__int32__Sequence *)(untyped_member);
  return &member->data[index];
}

void * other_msgs__msg__SegCloud__rosidl_typesupport_introspection_c__get_function__SegCloud__seg_ring_str_ind(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__int32__Sequence * member =
    (rosidl_runtime_c__int32__Sequence *)(untyped_member);
  return &member->data[index];
}

void other_msgs__msg__SegCloud__rosidl_typesupport_introspection_c__fetch_function__SegCloud__seg_ring_str_ind(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const int32_t * item =
    ((const int32_t *)
    other_msgs__msg__SegCloud__rosidl_typesupport_introspection_c__get_const_function__SegCloud__seg_ring_str_ind(untyped_member, index));
  int32_t * value =
    (int32_t *)(untyped_value);
  *value = *item;
}

void other_msgs__msg__SegCloud__rosidl_typesupport_introspection_c__assign_function__SegCloud__seg_ring_str_ind(
  void * untyped_member, size_t index, const void * untyped_value)
{
  int32_t * item =
    ((int32_t *)
    other_msgs__msg__SegCloud__rosidl_typesupport_introspection_c__get_function__SegCloud__seg_ring_str_ind(untyped_member, index));
  const int32_t * value =
    (const int32_t *)(untyped_value);
  *item = *value;
}

bool other_msgs__msg__SegCloud__rosidl_typesupport_introspection_c__resize_function__SegCloud__seg_ring_str_ind(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__int32__Sequence * member =
    (rosidl_runtime_c__int32__Sequence *)(untyped_member);
  rosidl_runtime_c__int32__Sequence__fini(member);
  return rosidl_runtime_c__int32__Sequence__init(member, size);
}

size_t other_msgs__msg__SegCloud__rosidl_typesupport_introspection_c__size_function__SegCloud__seg_ring_end_ind(
  const void * untyped_member)
{
  const rosidl_runtime_c__int32__Sequence * member =
    (const rosidl_runtime_c__int32__Sequence *)(untyped_member);
  return member->size;
}

const void * other_msgs__msg__SegCloud__rosidl_typesupport_introspection_c__get_const_function__SegCloud__seg_ring_end_ind(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__int32__Sequence * member =
    (const rosidl_runtime_c__int32__Sequence *)(untyped_member);
  return &member->data[index];
}

void * other_msgs__msg__SegCloud__rosidl_typesupport_introspection_c__get_function__SegCloud__seg_ring_end_ind(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__int32__Sequence * member =
    (rosidl_runtime_c__int32__Sequence *)(untyped_member);
  return &member->data[index];
}

void other_msgs__msg__SegCloud__rosidl_typesupport_introspection_c__fetch_function__SegCloud__seg_ring_end_ind(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const int32_t * item =
    ((const int32_t *)
    other_msgs__msg__SegCloud__rosidl_typesupport_introspection_c__get_const_function__SegCloud__seg_ring_end_ind(untyped_member, index));
  int32_t * value =
    (int32_t *)(untyped_value);
  *value = *item;
}

void other_msgs__msg__SegCloud__rosidl_typesupport_introspection_c__assign_function__SegCloud__seg_ring_end_ind(
  void * untyped_member, size_t index, const void * untyped_value)
{
  int32_t * item =
    ((int32_t *)
    other_msgs__msg__SegCloud__rosidl_typesupport_introspection_c__get_function__SegCloud__seg_ring_end_ind(untyped_member, index));
  const int32_t * value =
    (const int32_t *)(untyped_value);
  *item = *value;
}

bool other_msgs__msg__SegCloud__rosidl_typesupport_introspection_c__resize_function__SegCloud__seg_ring_end_ind(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__int32__Sequence * member =
    (rosidl_runtime_c__int32__Sequence *)(untyped_member);
  rosidl_runtime_c__int32__Sequence__fini(member);
  return rosidl_runtime_c__int32__Sequence__init(member, size);
}

size_t other_msgs__msg__SegCloud__rosidl_typesupport_introspection_c__size_function__SegCloud__seg_cloud(
  const void * untyped_member)
{
  const other_msgs__msg__Point__Sequence * member =
    (const other_msgs__msg__Point__Sequence *)(untyped_member);
  return member->size;
}

const void * other_msgs__msg__SegCloud__rosidl_typesupport_introspection_c__get_const_function__SegCloud__seg_cloud(
  const void * untyped_member, size_t index)
{
  const other_msgs__msg__Point__Sequence * member =
    (const other_msgs__msg__Point__Sequence *)(untyped_member);
  return &member->data[index];
}

void * other_msgs__msg__SegCloud__rosidl_typesupport_introspection_c__get_function__SegCloud__seg_cloud(
  void * untyped_member, size_t index)
{
  other_msgs__msg__Point__Sequence * member =
    (other_msgs__msg__Point__Sequence *)(untyped_member);
  return &member->data[index];
}

void other_msgs__msg__SegCloud__rosidl_typesupport_introspection_c__fetch_function__SegCloud__seg_cloud(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const other_msgs__msg__Point * item =
    ((const other_msgs__msg__Point *)
    other_msgs__msg__SegCloud__rosidl_typesupport_introspection_c__get_const_function__SegCloud__seg_cloud(untyped_member, index));
  other_msgs__msg__Point * value =
    (other_msgs__msg__Point *)(untyped_value);
  *value = *item;
}

void other_msgs__msg__SegCloud__rosidl_typesupport_introspection_c__assign_function__SegCloud__seg_cloud(
  void * untyped_member, size_t index, const void * untyped_value)
{
  other_msgs__msg__Point * item =
    ((other_msgs__msg__Point *)
    other_msgs__msg__SegCloud__rosidl_typesupport_introspection_c__get_function__SegCloud__seg_cloud(untyped_member, index));
  const other_msgs__msg__Point * value =
    (const other_msgs__msg__Point *)(untyped_value);
  *item = *value;
}

bool other_msgs__msg__SegCloud__rosidl_typesupport_introspection_c__resize_function__SegCloud__seg_cloud(
  void * untyped_member, size_t size)
{
  other_msgs__msg__Point__Sequence * member =
    (other_msgs__msg__Point__Sequence *)(untyped_member);
  other_msgs__msg__Point__Sequence__fini(member);
  return other_msgs__msg__Point__Sequence__init(member, size);
}

size_t other_msgs__msg__SegCloud__rosidl_typesupport_introspection_c__size_function__SegCloud__seg_range(
  const void * untyped_member)
{
  const rosidl_runtime_c__float__Sequence * member =
    (const rosidl_runtime_c__float__Sequence *)(untyped_member);
  return member->size;
}

const void * other_msgs__msg__SegCloud__rosidl_typesupport_introspection_c__get_const_function__SegCloud__seg_range(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__float__Sequence * member =
    (const rosidl_runtime_c__float__Sequence *)(untyped_member);
  return &member->data[index];
}

void * other_msgs__msg__SegCloud__rosidl_typesupport_introspection_c__get_function__SegCloud__seg_range(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__float__Sequence * member =
    (rosidl_runtime_c__float__Sequence *)(untyped_member);
  return &member->data[index];
}

void other_msgs__msg__SegCloud__rosidl_typesupport_introspection_c__fetch_function__SegCloud__seg_range(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const float * item =
    ((const float *)
    other_msgs__msg__SegCloud__rosidl_typesupport_introspection_c__get_const_function__SegCloud__seg_range(untyped_member, index));
  float * value =
    (float *)(untyped_value);
  *value = *item;
}

void other_msgs__msg__SegCloud__rosidl_typesupport_introspection_c__assign_function__SegCloud__seg_range(
  void * untyped_member, size_t index, const void * untyped_value)
{
  float * item =
    ((float *)
    other_msgs__msg__SegCloud__rosidl_typesupport_introspection_c__get_function__SegCloud__seg_range(untyped_member, index));
  const float * value =
    (const float *)(untyped_value);
  *item = *value;
}

bool other_msgs__msg__SegCloud__rosidl_typesupport_introspection_c__resize_function__SegCloud__seg_range(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__float__Sequence * member =
    (rosidl_runtime_c__float__Sequence *)(untyped_member);
  rosidl_runtime_c__float__Sequence__fini(member);
  return rosidl_runtime_c__float__Sequence__init(member, size);
}

size_t other_msgs__msg__SegCloud__rosidl_typesupport_introspection_c__size_function__SegCloud__grd_ring_str_ind(
  const void * untyped_member)
{
  const rosidl_runtime_c__int32__Sequence * member =
    (const rosidl_runtime_c__int32__Sequence *)(untyped_member);
  return member->size;
}

const void * other_msgs__msg__SegCloud__rosidl_typesupport_introspection_c__get_const_function__SegCloud__grd_ring_str_ind(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__int32__Sequence * member =
    (const rosidl_runtime_c__int32__Sequence *)(untyped_member);
  return &member->data[index];
}

void * other_msgs__msg__SegCloud__rosidl_typesupport_introspection_c__get_function__SegCloud__grd_ring_str_ind(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__int32__Sequence * member =
    (rosidl_runtime_c__int32__Sequence *)(untyped_member);
  return &member->data[index];
}

void other_msgs__msg__SegCloud__rosidl_typesupport_introspection_c__fetch_function__SegCloud__grd_ring_str_ind(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const int32_t * item =
    ((const int32_t *)
    other_msgs__msg__SegCloud__rosidl_typesupport_introspection_c__get_const_function__SegCloud__grd_ring_str_ind(untyped_member, index));
  int32_t * value =
    (int32_t *)(untyped_value);
  *value = *item;
}

void other_msgs__msg__SegCloud__rosidl_typesupport_introspection_c__assign_function__SegCloud__grd_ring_str_ind(
  void * untyped_member, size_t index, const void * untyped_value)
{
  int32_t * item =
    ((int32_t *)
    other_msgs__msg__SegCloud__rosidl_typesupport_introspection_c__get_function__SegCloud__grd_ring_str_ind(untyped_member, index));
  const int32_t * value =
    (const int32_t *)(untyped_value);
  *item = *value;
}

bool other_msgs__msg__SegCloud__rosidl_typesupport_introspection_c__resize_function__SegCloud__grd_ring_str_ind(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__int32__Sequence * member =
    (rosidl_runtime_c__int32__Sequence *)(untyped_member);
  rosidl_runtime_c__int32__Sequence__fini(member);
  return rosidl_runtime_c__int32__Sequence__init(member, size);
}

size_t other_msgs__msg__SegCloud__rosidl_typesupport_introspection_c__size_function__SegCloud__grd_ring_end_ind(
  const void * untyped_member)
{
  const rosidl_runtime_c__int32__Sequence * member =
    (const rosidl_runtime_c__int32__Sequence *)(untyped_member);
  return member->size;
}

const void * other_msgs__msg__SegCloud__rosidl_typesupport_introspection_c__get_const_function__SegCloud__grd_ring_end_ind(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__int32__Sequence * member =
    (const rosidl_runtime_c__int32__Sequence *)(untyped_member);
  return &member->data[index];
}

void * other_msgs__msg__SegCloud__rosidl_typesupport_introspection_c__get_function__SegCloud__grd_ring_end_ind(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__int32__Sequence * member =
    (rosidl_runtime_c__int32__Sequence *)(untyped_member);
  return &member->data[index];
}

void other_msgs__msg__SegCloud__rosidl_typesupport_introspection_c__fetch_function__SegCloud__grd_ring_end_ind(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const int32_t * item =
    ((const int32_t *)
    other_msgs__msg__SegCloud__rosidl_typesupport_introspection_c__get_const_function__SegCloud__grd_ring_end_ind(untyped_member, index));
  int32_t * value =
    (int32_t *)(untyped_value);
  *value = *item;
}

void other_msgs__msg__SegCloud__rosidl_typesupport_introspection_c__assign_function__SegCloud__grd_ring_end_ind(
  void * untyped_member, size_t index, const void * untyped_value)
{
  int32_t * item =
    ((int32_t *)
    other_msgs__msg__SegCloud__rosidl_typesupport_introspection_c__get_function__SegCloud__grd_ring_end_ind(untyped_member, index));
  const int32_t * value =
    (const int32_t *)(untyped_value);
  *item = *value;
}

bool other_msgs__msg__SegCloud__rosidl_typesupport_introspection_c__resize_function__SegCloud__grd_ring_end_ind(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__int32__Sequence * member =
    (rosidl_runtime_c__int32__Sequence *)(untyped_member);
  rosidl_runtime_c__int32__Sequence__fini(member);
  return rosidl_runtime_c__int32__Sequence__init(member, size);
}

size_t other_msgs__msg__SegCloud__rosidl_typesupport_introspection_c__size_function__SegCloud__ground_cloud(
  const void * untyped_member)
{
  const other_msgs__msg__Point__Sequence * member =
    (const other_msgs__msg__Point__Sequence *)(untyped_member);
  return member->size;
}

const void * other_msgs__msg__SegCloud__rosidl_typesupport_introspection_c__get_const_function__SegCloud__ground_cloud(
  const void * untyped_member, size_t index)
{
  const other_msgs__msg__Point__Sequence * member =
    (const other_msgs__msg__Point__Sequence *)(untyped_member);
  return &member->data[index];
}

void * other_msgs__msg__SegCloud__rosidl_typesupport_introspection_c__get_function__SegCloud__ground_cloud(
  void * untyped_member, size_t index)
{
  other_msgs__msg__Point__Sequence * member =
    (other_msgs__msg__Point__Sequence *)(untyped_member);
  return &member->data[index];
}

void other_msgs__msg__SegCloud__rosidl_typesupport_introspection_c__fetch_function__SegCloud__ground_cloud(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const other_msgs__msg__Point * item =
    ((const other_msgs__msg__Point *)
    other_msgs__msg__SegCloud__rosidl_typesupport_introspection_c__get_const_function__SegCloud__ground_cloud(untyped_member, index));
  other_msgs__msg__Point * value =
    (other_msgs__msg__Point *)(untyped_value);
  *value = *item;
}

void other_msgs__msg__SegCloud__rosidl_typesupport_introspection_c__assign_function__SegCloud__ground_cloud(
  void * untyped_member, size_t index, const void * untyped_value)
{
  other_msgs__msg__Point * item =
    ((other_msgs__msg__Point *)
    other_msgs__msg__SegCloud__rosidl_typesupport_introspection_c__get_function__SegCloud__ground_cloud(untyped_member, index));
  const other_msgs__msg__Point * value =
    (const other_msgs__msg__Point *)(untyped_value);
  *item = *value;
}

bool other_msgs__msg__SegCloud__rosidl_typesupport_introspection_c__resize_function__SegCloud__ground_cloud(
  void * untyped_member, size_t size)
{
  other_msgs__msg__Point__Sequence * member =
    (other_msgs__msg__Point__Sequence *)(untyped_member);
  other_msgs__msg__Point__Sequence__fini(member);
  return other_msgs__msg__Point__Sequence__init(member, size);
}

size_t other_msgs__msg__SegCloud__rosidl_typesupport_introspection_c__size_function__SegCloud__ground_range(
  const void * untyped_member)
{
  const rosidl_runtime_c__float__Sequence * member =
    (const rosidl_runtime_c__float__Sequence *)(untyped_member);
  return member->size;
}

const void * other_msgs__msg__SegCloud__rosidl_typesupport_introspection_c__get_const_function__SegCloud__ground_range(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__float__Sequence * member =
    (const rosidl_runtime_c__float__Sequence *)(untyped_member);
  return &member->data[index];
}

void * other_msgs__msg__SegCloud__rosidl_typesupport_introspection_c__get_function__SegCloud__ground_range(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__float__Sequence * member =
    (rosidl_runtime_c__float__Sequence *)(untyped_member);
  return &member->data[index];
}

void other_msgs__msg__SegCloud__rosidl_typesupport_introspection_c__fetch_function__SegCloud__ground_range(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const float * item =
    ((const float *)
    other_msgs__msg__SegCloud__rosidl_typesupport_introspection_c__get_const_function__SegCloud__ground_range(untyped_member, index));
  float * value =
    (float *)(untyped_value);
  *value = *item;
}

void other_msgs__msg__SegCloud__rosidl_typesupport_introspection_c__assign_function__SegCloud__ground_range(
  void * untyped_member, size_t index, const void * untyped_value)
{
  float * item =
    ((float *)
    other_msgs__msg__SegCloud__rosidl_typesupport_introspection_c__get_function__SegCloud__ground_range(untyped_member, index));
  const float * value =
    (const float *)(untyped_value);
  *item = *value;
}

bool other_msgs__msg__SegCloud__rosidl_typesupport_introspection_c__resize_function__SegCloud__ground_range(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__float__Sequence * member =
    (rosidl_runtime_c__float__Sequence *)(untyped_member);
  rosidl_runtime_c__float__Sequence__fini(member);
  return rosidl_runtime_c__float__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember other_msgs__msg__SegCloud__rosidl_typesupport_introspection_c__SegCloud_message_member_array[9] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(other_msgs__msg__SegCloud, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "seg_ring_str_ind",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(other_msgs__msg__SegCloud, seg_ring_str_ind),  // bytes offset in struct
    NULL,  // default value
    other_msgs__msg__SegCloud__rosidl_typesupport_introspection_c__size_function__SegCloud__seg_ring_str_ind,  // size() function pointer
    other_msgs__msg__SegCloud__rosidl_typesupport_introspection_c__get_const_function__SegCloud__seg_ring_str_ind,  // get_const(index) function pointer
    other_msgs__msg__SegCloud__rosidl_typesupport_introspection_c__get_function__SegCloud__seg_ring_str_ind,  // get(index) function pointer
    other_msgs__msg__SegCloud__rosidl_typesupport_introspection_c__fetch_function__SegCloud__seg_ring_str_ind,  // fetch(index, &value) function pointer
    other_msgs__msg__SegCloud__rosidl_typesupport_introspection_c__assign_function__SegCloud__seg_ring_str_ind,  // assign(index, value) function pointer
    other_msgs__msg__SegCloud__rosidl_typesupport_introspection_c__resize_function__SegCloud__seg_ring_str_ind  // resize(index) function pointer
  },
  {
    "seg_ring_end_ind",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(other_msgs__msg__SegCloud, seg_ring_end_ind),  // bytes offset in struct
    NULL,  // default value
    other_msgs__msg__SegCloud__rosidl_typesupport_introspection_c__size_function__SegCloud__seg_ring_end_ind,  // size() function pointer
    other_msgs__msg__SegCloud__rosidl_typesupport_introspection_c__get_const_function__SegCloud__seg_ring_end_ind,  // get_const(index) function pointer
    other_msgs__msg__SegCloud__rosidl_typesupport_introspection_c__get_function__SegCloud__seg_ring_end_ind,  // get(index) function pointer
    other_msgs__msg__SegCloud__rosidl_typesupport_introspection_c__fetch_function__SegCloud__seg_ring_end_ind,  // fetch(index, &value) function pointer
    other_msgs__msg__SegCloud__rosidl_typesupport_introspection_c__assign_function__SegCloud__seg_ring_end_ind,  // assign(index, value) function pointer
    other_msgs__msg__SegCloud__rosidl_typesupport_introspection_c__resize_function__SegCloud__seg_ring_end_ind  // resize(index) function pointer
  },
  {
    "seg_cloud",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(other_msgs__msg__SegCloud, seg_cloud),  // bytes offset in struct
    NULL,  // default value
    other_msgs__msg__SegCloud__rosidl_typesupport_introspection_c__size_function__SegCloud__seg_cloud,  // size() function pointer
    other_msgs__msg__SegCloud__rosidl_typesupport_introspection_c__get_const_function__SegCloud__seg_cloud,  // get_const(index) function pointer
    other_msgs__msg__SegCloud__rosidl_typesupport_introspection_c__get_function__SegCloud__seg_cloud,  // get(index) function pointer
    other_msgs__msg__SegCloud__rosidl_typesupport_introspection_c__fetch_function__SegCloud__seg_cloud,  // fetch(index, &value) function pointer
    other_msgs__msg__SegCloud__rosidl_typesupport_introspection_c__assign_function__SegCloud__seg_cloud,  // assign(index, value) function pointer
    other_msgs__msg__SegCloud__rosidl_typesupport_introspection_c__resize_function__SegCloud__seg_cloud  // resize(index) function pointer
  },
  {
    "seg_range",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(other_msgs__msg__SegCloud, seg_range),  // bytes offset in struct
    NULL,  // default value
    other_msgs__msg__SegCloud__rosidl_typesupport_introspection_c__size_function__SegCloud__seg_range,  // size() function pointer
    other_msgs__msg__SegCloud__rosidl_typesupport_introspection_c__get_const_function__SegCloud__seg_range,  // get_const(index) function pointer
    other_msgs__msg__SegCloud__rosidl_typesupport_introspection_c__get_function__SegCloud__seg_range,  // get(index) function pointer
    other_msgs__msg__SegCloud__rosidl_typesupport_introspection_c__fetch_function__SegCloud__seg_range,  // fetch(index, &value) function pointer
    other_msgs__msg__SegCloud__rosidl_typesupport_introspection_c__assign_function__SegCloud__seg_range,  // assign(index, value) function pointer
    other_msgs__msg__SegCloud__rosidl_typesupport_introspection_c__resize_function__SegCloud__seg_range  // resize(index) function pointer
  },
  {
    "grd_ring_str_ind",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(other_msgs__msg__SegCloud, grd_ring_str_ind),  // bytes offset in struct
    NULL,  // default value
    other_msgs__msg__SegCloud__rosidl_typesupport_introspection_c__size_function__SegCloud__grd_ring_str_ind,  // size() function pointer
    other_msgs__msg__SegCloud__rosidl_typesupport_introspection_c__get_const_function__SegCloud__grd_ring_str_ind,  // get_const(index) function pointer
    other_msgs__msg__SegCloud__rosidl_typesupport_introspection_c__get_function__SegCloud__grd_ring_str_ind,  // get(index) function pointer
    other_msgs__msg__SegCloud__rosidl_typesupport_introspection_c__fetch_function__SegCloud__grd_ring_str_ind,  // fetch(index, &value) function pointer
    other_msgs__msg__SegCloud__rosidl_typesupport_introspection_c__assign_function__SegCloud__grd_ring_str_ind,  // assign(index, value) function pointer
    other_msgs__msg__SegCloud__rosidl_typesupport_introspection_c__resize_function__SegCloud__grd_ring_str_ind  // resize(index) function pointer
  },
  {
    "grd_ring_end_ind",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(other_msgs__msg__SegCloud, grd_ring_end_ind),  // bytes offset in struct
    NULL,  // default value
    other_msgs__msg__SegCloud__rosidl_typesupport_introspection_c__size_function__SegCloud__grd_ring_end_ind,  // size() function pointer
    other_msgs__msg__SegCloud__rosidl_typesupport_introspection_c__get_const_function__SegCloud__grd_ring_end_ind,  // get_const(index) function pointer
    other_msgs__msg__SegCloud__rosidl_typesupport_introspection_c__get_function__SegCloud__grd_ring_end_ind,  // get(index) function pointer
    other_msgs__msg__SegCloud__rosidl_typesupport_introspection_c__fetch_function__SegCloud__grd_ring_end_ind,  // fetch(index, &value) function pointer
    other_msgs__msg__SegCloud__rosidl_typesupport_introspection_c__assign_function__SegCloud__grd_ring_end_ind,  // assign(index, value) function pointer
    other_msgs__msg__SegCloud__rosidl_typesupport_introspection_c__resize_function__SegCloud__grd_ring_end_ind  // resize(index) function pointer
  },
  {
    "ground_cloud",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(other_msgs__msg__SegCloud, ground_cloud),  // bytes offset in struct
    NULL,  // default value
    other_msgs__msg__SegCloud__rosidl_typesupport_introspection_c__size_function__SegCloud__ground_cloud,  // size() function pointer
    other_msgs__msg__SegCloud__rosidl_typesupport_introspection_c__get_const_function__SegCloud__ground_cloud,  // get_const(index) function pointer
    other_msgs__msg__SegCloud__rosidl_typesupport_introspection_c__get_function__SegCloud__ground_cloud,  // get(index) function pointer
    other_msgs__msg__SegCloud__rosidl_typesupport_introspection_c__fetch_function__SegCloud__ground_cloud,  // fetch(index, &value) function pointer
    other_msgs__msg__SegCloud__rosidl_typesupport_introspection_c__assign_function__SegCloud__ground_cloud,  // assign(index, value) function pointer
    other_msgs__msg__SegCloud__rosidl_typesupport_introspection_c__resize_function__SegCloud__ground_cloud  // resize(index) function pointer
  },
  {
    "ground_range",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(other_msgs__msg__SegCloud, ground_range),  // bytes offset in struct
    NULL,  // default value
    other_msgs__msg__SegCloud__rosidl_typesupport_introspection_c__size_function__SegCloud__ground_range,  // size() function pointer
    other_msgs__msg__SegCloud__rosidl_typesupport_introspection_c__get_const_function__SegCloud__ground_range,  // get_const(index) function pointer
    other_msgs__msg__SegCloud__rosidl_typesupport_introspection_c__get_function__SegCloud__ground_range,  // get(index) function pointer
    other_msgs__msg__SegCloud__rosidl_typesupport_introspection_c__fetch_function__SegCloud__ground_range,  // fetch(index, &value) function pointer
    other_msgs__msg__SegCloud__rosidl_typesupport_introspection_c__assign_function__SegCloud__ground_range,  // assign(index, value) function pointer
    other_msgs__msg__SegCloud__rosidl_typesupport_introspection_c__resize_function__SegCloud__ground_range  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers other_msgs__msg__SegCloud__rosidl_typesupport_introspection_c__SegCloud_message_members = {
  "other_msgs__msg",  // message namespace
  "SegCloud",  // message name
  9,  // number of fields
  sizeof(other_msgs__msg__SegCloud),
  other_msgs__msg__SegCloud__rosidl_typesupport_introspection_c__SegCloud_message_member_array,  // message members
  other_msgs__msg__SegCloud__rosidl_typesupport_introspection_c__SegCloud_init_function,  // function to initialize message memory (memory has to be allocated)
  other_msgs__msg__SegCloud__rosidl_typesupport_introspection_c__SegCloud_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t other_msgs__msg__SegCloud__rosidl_typesupport_introspection_c__SegCloud_message_type_support_handle = {
  0,
  &other_msgs__msg__SegCloud__rosidl_typesupport_introspection_c__SegCloud_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_other_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, other_msgs, msg, SegCloud)() {
  other_msgs__msg__SegCloud__rosidl_typesupport_introspection_c__SegCloud_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  other_msgs__msg__SegCloud__rosidl_typesupport_introspection_c__SegCloud_message_member_array[3].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, other_msgs, msg, Point)();
  other_msgs__msg__SegCloud__rosidl_typesupport_introspection_c__SegCloud_message_member_array[7].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, other_msgs, msg, Point)();
  if (!other_msgs__msg__SegCloud__rosidl_typesupport_introspection_c__SegCloud_message_type_support_handle.typesupport_identifier) {
    other_msgs__msg__SegCloud__rosidl_typesupport_introspection_c__SegCloud_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &other_msgs__msg__SegCloud__rosidl_typesupport_introspection_c__SegCloud_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
