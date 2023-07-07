// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from other_msgs:msg/AllCloud.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "other_msgs/msg/detail/all_cloud__rosidl_typesupport_introspection_c.h"
#include "other_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "other_msgs/msg/detail/all_cloud__functions.h"
#include "other_msgs/msg/detail/all_cloud__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `cornersharp`
// Member `cornerlesssharp`
// Member `surfflat`
// Member `surflessflat`
// Member `fullpointres`
#include "other_msgs/msg/point.h"
// Member `cornersharp`
// Member `cornerlesssharp`
// Member `surfflat`
// Member `surflessflat`
// Member `fullpointres`
#include "other_msgs/msg/detail/point__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void other_msgs__msg__AllCloud__rosidl_typesupport_introspection_c__AllCloud_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  other_msgs__msg__AllCloud__init(message_memory);
}

void other_msgs__msg__AllCloud__rosidl_typesupport_introspection_c__AllCloud_fini_function(void * message_memory)
{
  other_msgs__msg__AllCloud__fini(message_memory);
}

size_t other_msgs__msg__AllCloud__rosidl_typesupport_introspection_c__size_function__AllCloud__cornersharp(
  const void * untyped_member)
{
  const other_msgs__msg__Point__Sequence * member =
    (const other_msgs__msg__Point__Sequence *)(untyped_member);
  return member->size;
}

const void * other_msgs__msg__AllCloud__rosidl_typesupport_introspection_c__get_const_function__AllCloud__cornersharp(
  const void * untyped_member, size_t index)
{
  const other_msgs__msg__Point__Sequence * member =
    (const other_msgs__msg__Point__Sequence *)(untyped_member);
  return &member->data[index];
}

void * other_msgs__msg__AllCloud__rosidl_typesupport_introspection_c__get_function__AllCloud__cornersharp(
  void * untyped_member, size_t index)
{
  other_msgs__msg__Point__Sequence * member =
    (other_msgs__msg__Point__Sequence *)(untyped_member);
  return &member->data[index];
}

void other_msgs__msg__AllCloud__rosidl_typesupport_introspection_c__fetch_function__AllCloud__cornersharp(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const other_msgs__msg__Point * item =
    ((const other_msgs__msg__Point *)
    other_msgs__msg__AllCloud__rosidl_typesupport_introspection_c__get_const_function__AllCloud__cornersharp(untyped_member, index));
  other_msgs__msg__Point * value =
    (other_msgs__msg__Point *)(untyped_value);
  *value = *item;
}

void other_msgs__msg__AllCloud__rosidl_typesupport_introspection_c__assign_function__AllCloud__cornersharp(
  void * untyped_member, size_t index, const void * untyped_value)
{
  other_msgs__msg__Point * item =
    ((other_msgs__msg__Point *)
    other_msgs__msg__AllCloud__rosidl_typesupport_introspection_c__get_function__AllCloud__cornersharp(untyped_member, index));
  const other_msgs__msg__Point * value =
    (const other_msgs__msg__Point *)(untyped_value);
  *item = *value;
}

bool other_msgs__msg__AllCloud__rosidl_typesupport_introspection_c__resize_function__AllCloud__cornersharp(
  void * untyped_member, size_t size)
{
  other_msgs__msg__Point__Sequence * member =
    (other_msgs__msg__Point__Sequence *)(untyped_member);
  other_msgs__msg__Point__Sequence__fini(member);
  return other_msgs__msg__Point__Sequence__init(member, size);
}

size_t other_msgs__msg__AllCloud__rosidl_typesupport_introspection_c__size_function__AllCloud__cornerlesssharp(
  const void * untyped_member)
{
  const other_msgs__msg__Point__Sequence * member =
    (const other_msgs__msg__Point__Sequence *)(untyped_member);
  return member->size;
}

const void * other_msgs__msg__AllCloud__rosidl_typesupport_introspection_c__get_const_function__AllCloud__cornerlesssharp(
  const void * untyped_member, size_t index)
{
  const other_msgs__msg__Point__Sequence * member =
    (const other_msgs__msg__Point__Sequence *)(untyped_member);
  return &member->data[index];
}

void * other_msgs__msg__AllCloud__rosidl_typesupport_introspection_c__get_function__AllCloud__cornerlesssharp(
  void * untyped_member, size_t index)
{
  other_msgs__msg__Point__Sequence * member =
    (other_msgs__msg__Point__Sequence *)(untyped_member);
  return &member->data[index];
}

void other_msgs__msg__AllCloud__rosidl_typesupport_introspection_c__fetch_function__AllCloud__cornerlesssharp(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const other_msgs__msg__Point * item =
    ((const other_msgs__msg__Point *)
    other_msgs__msg__AllCloud__rosidl_typesupport_introspection_c__get_const_function__AllCloud__cornerlesssharp(untyped_member, index));
  other_msgs__msg__Point * value =
    (other_msgs__msg__Point *)(untyped_value);
  *value = *item;
}

void other_msgs__msg__AllCloud__rosidl_typesupport_introspection_c__assign_function__AllCloud__cornerlesssharp(
  void * untyped_member, size_t index, const void * untyped_value)
{
  other_msgs__msg__Point * item =
    ((other_msgs__msg__Point *)
    other_msgs__msg__AllCloud__rosidl_typesupport_introspection_c__get_function__AllCloud__cornerlesssharp(untyped_member, index));
  const other_msgs__msg__Point * value =
    (const other_msgs__msg__Point *)(untyped_value);
  *item = *value;
}

bool other_msgs__msg__AllCloud__rosidl_typesupport_introspection_c__resize_function__AllCloud__cornerlesssharp(
  void * untyped_member, size_t size)
{
  other_msgs__msg__Point__Sequence * member =
    (other_msgs__msg__Point__Sequence *)(untyped_member);
  other_msgs__msg__Point__Sequence__fini(member);
  return other_msgs__msg__Point__Sequence__init(member, size);
}

size_t other_msgs__msg__AllCloud__rosidl_typesupport_introspection_c__size_function__AllCloud__surfflat(
  const void * untyped_member)
{
  const other_msgs__msg__Point__Sequence * member =
    (const other_msgs__msg__Point__Sequence *)(untyped_member);
  return member->size;
}

const void * other_msgs__msg__AllCloud__rosidl_typesupport_introspection_c__get_const_function__AllCloud__surfflat(
  const void * untyped_member, size_t index)
{
  const other_msgs__msg__Point__Sequence * member =
    (const other_msgs__msg__Point__Sequence *)(untyped_member);
  return &member->data[index];
}

void * other_msgs__msg__AllCloud__rosidl_typesupport_introspection_c__get_function__AllCloud__surfflat(
  void * untyped_member, size_t index)
{
  other_msgs__msg__Point__Sequence * member =
    (other_msgs__msg__Point__Sequence *)(untyped_member);
  return &member->data[index];
}

void other_msgs__msg__AllCloud__rosidl_typesupport_introspection_c__fetch_function__AllCloud__surfflat(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const other_msgs__msg__Point * item =
    ((const other_msgs__msg__Point *)
    other_msgs__msg__AllCloud__rosidl_typesupport_introspection_c__get_const_function__AllCloud__surfflat(untyped_member, index));
  other_msgs__msg__Point * value =
    (other_msgs__msg__Point *)(untyped_value);
  *value = *item;
}

void other_msgs__msg__AllCloud__rosidl_typesupport_introspection_c__assign_function__AllCloud__surfflat(
  void * untyped_member, size_t index, const void * untyped_value)
{
  other_msgs__msg__Point * item =
    ((other_msgs__msg__Point *)
    other_msgs__msg__AllCloud__rosidl_typesupport_introspection_c__get_function__AllCloud__surfflat(untyped_member, index));
  const other_msgs__msg__Point * value =
    (const other_msgs__msg__Point *)(untyped_value);
  *item = *value;
}

bool other_msgs__msg__AllCloud__rosidl_typesupport_introspection_c__resize_function__AllCloud__surfflat(
  void * untyped_member, size_t size)
{
  other_msgs__msg__Point__Sequence * member =
    (other_msgs__msg__Point__Sequence *)(untyped_member);
  other_msgs__msg__Point__Sequence__fini(member);
  return other_msgs__msg__Point__Sequence__init(member, size);
}

size_t other_msgs__msg__AllCloud__rosidl_typesupport_introspection_c__size_function__AllCloud__surflessflat(
  const void * untyped_member)
{
  const other_msgs__msg__Point__Sequence * member =
    (const other_msgs__msg__Point__Sequence *)(untyped_member);
  return member->size;
}

const void * other_msgs__msg__AllCloud__rosidl_typesupport_introspection_c__get_const_function__AllCloud__surflessflat(
  const void * untyped_member, size_t index)
{
  const other_msgs__msg__Point__Sequence * member =
    (const other_msgs__msg__Point__Sequence *)(untyped_member);
  return &member->data[index];
}

void * other_msgs__msg__AllCloud__rosidl_typesupport_introspection_c__get_function__AllCloud__surflessflat(
  void * untyped_member, size_t index)
{
  other_msgs__msg__Point__Sequence * member =
    (other_msgs__msg__Point__Sequence *)(untyped_member);
  return &member->data[index];
}

void other_msgs__msg__AllCloud__rosidl_typesupport_introspection_c__fetch_function__AllCloud__surflessflat(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const other_msgs__msg__Point * item =
    ((const other_msgs__msg__Point *)
    other_msgs__msg__AllCloud__rosidl_typesupport_introspection_c__get_const_function__AllCloud__surflessflat(untyped_member, index));
  other_msgs__msg__Point * value =
    (other_msgs__msg__Point *)(untyped_value);
  *value = *item;
}

void other_msgs__msg__AllCloud__rosidl_typesupport_introspection_c__assign_function__AllCloud__surflessflat(
  void * untyped_member, size_t index, const void * untyped_value)
{
  other_msgs__msg__Point * item =
    ((other_msgs__msg__Point *)
    other_msgs__msg__AllCloud__rosidl_typesupport_introspection_c__get_function__AllCloud__surflessflat(untyped_member, index));
  const other_msgs__msg__Point * value =
    (const other_msgs__msg__Point *)(untyped_value);
  *item = *value;
}

bool other_msgs__msg__AllCloud__rosidl_typesupport_introspection_c__resize_function__AllCloud__surflessflat(
  void * untyped_member, size_t size)
{
  other_msgs__msg__Point__Sequence * member =
    (other_msgs__msg__Point__Sequence *)(untyped_member);
  other_msgs__msg__Point__Sequence__fini(member);
  return other_msgs__msg__Point__Sequence__init(member, size);
}

size_t other_msgs__msg__AllCloud__rosidl_typesupport_introspection_c__size_function__AllCloud__fullpointres(
  const void * untyped_member)
{
  const other_msgs__msg__Point__Sequence * member =
    (const other_msgs__msg__Point__Sequence *)(untyped_member);
  return member->size;
}

const void * other_msgs__msg__AllCloud__rosidl_typesupport_introspection_c__get_const_function__AllCloud__fullpointres(
  const void * untyped_member, size_t index)
{
  const other_msgs__msg__Point__Sequence * member =
    (const other_msgs__msg__Point__Sequence *)(untyped_member);
  return &member->data[index];
}

void * other_msgs__msg__AllCloud__rosidl_typesupport_introspection_c__get_function__AllCloud__fullpointres(
  void * untyped_member, size_t index)
{
  other_msgs__msg__Point__Sequence * member =
    (other_msgs__msg__Point__Sequence *)(untyped_member);
  return &member->data[index];
}

void other_msgs__msg__AllCloud__rosidl_typesupport_introspection_c__fetch_function__AllCloud__fullpointres(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const other_msgs__msg__Point * item =
    ((const other_msgs__msg__Point *)
    other_msgs__msg__AllCloud__rosidl_typesupport_introspection_c__get_const_function__AllCloud__fullpointres(untyped_member, index));
  other_msgs__msg__Point * value =
    (other_msgs__msg__Point *)(untyped_value);
  *value = *item;
}

void other_msgs__msg__AllCloud__rosidl_typesupport_introspection_c__assign_function__AllCloud__fullpointres(
  void * untyped_member, size_t index, const void * untyped_value)
{
  other_msgs__msg__Point * item =
    ((other_msgs__msg__Point *)
    other_msgs__msg__AllCloud__rosidl_typesupport_introspection_c__get_function__AllCloud__fullpointres(untyped_member, index));
  const other_msgs__msg__Point * value =
    (const other_msgs__msg__Point *)(untyped_value);
  *item = *value;
}

bool other_msgs__msg__AllCloud__rosidl_typesupport_introspection_c__resize_function__AllCloud__fullpointres(
  void * untyped_member, size_t size)
{
  other_msgs__msg__Point__Sequence * member =
    (other_msgs__msg__Point__Sequence *)(untyped_member);
  other_msgs__msg__Point__Sequence__fini(member);
  return other_msgs__msg__Point__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember other_msgs__msg__AllCloud__rosidl_typesupport_introspection_c__AllCloud_message_member_array[6] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(other_msgs__msg__AllCloud, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "cornersharp",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(other_msgs__msg__AllCloud, cornersharp),  // bytes offset in struct
    NULL,  // default value
    other_msgs__msg__AllCloud__rosidl_typesupport_introspection_c__size_function__AllCloud__cornersharp,  // size() function pointer
    other_msgs__msg__AllCloud__rosidl_typesupport_introspection_c__get_const_function__AllCloud__cornersharp,  // get_const(index) function pointer
    other_msgs__msg__AllCloud__rosidl_typesupport_introspection_c__get_function__AllCloud__cornersharp,  // get(index) function pointer
    other_msgs__msg__AllCloud__rosidl_typesupport_introspection_c__fetch_function__AllCloud__cornersharp,  // fetch(index, &value) function pointer
    other_msgs__msg__AllCloud__rosidl_typesupport_introspection_c__assign_function__AllCloud__cornersharp,  // assign(index, value) function pointer
    other_msgs__msg__AllCloud__rosidl_typesupport_introspection_c__resize_function__AllCloud__cornersharp  // resize(index) function pointer
  },
  {
    "cornerlesssharp",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(other_msgs__msg__AllCloud, cornerlesssharp),  // bytes offset in struct
    NULL,  // default value
    other_msgs__msg__AllCloud__rosidl_typesupport_introspection_c__size_function__AllCloud__cornerlesssharp,  // size() function pointer
    other_msgs__msg__AllCloud__rosidl_typesupport_introspection_c__get_const_function__AllCloud__cornerlesssharp,  // get_const(index) function pointer
    other_msgs__msg__AllCloud__rosidl_typesupport_introspection_c__get_function__AllCloud__cornerlesssharp,  // get(index) function pointer
    other_msgs__msg__AllCloud__rosidl_typesupport_introspection_c__fetch_function__AllCloud__cornerlesssharp,  // fetch(index, &value) function pointer
    other_msgs__msg__AllCloud__rosidl_typesupport_introspection_c__assign_function__AllCloud__cornerlesssharp,  // assign(index, value) function pointer
    other_msgs__msg__AllCloud__rosidl_typesupport_introspection_c__resize_function__AllCloud__cornerlesssharp  // resize(index) function pointer
  },
  {
    "surfflat",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(other_msgs__msg__AllCloud, surfflat),  // bytes offset in struct
    NULL,  // default value
    other_msgs__msg__AllCloud__rosidl_typesupport_introspection_c__size_function__AllCloud__surfflat,  // size() function pointer
    other_msgs__msg__AllCloud__rosidl_typesupport_introspection_c__get_const_function__AllCloud__surfflat,  // get_const(index) function pointer
    other_msgs__msg__AllCloud__rosidl_typesupport_introspection_c__get_function__AllCloud__surfflat,  // get(index) function pointer
    other_msgs__msg__AllCloud__rosidl_typesupport_introspection_c__fetch_function__AllCloud__surfflat,  // fetch(index, &value) function pointer
    other_msgs__msg__AllCloud__rosidl_typesupport_introspection_c__assign_function__AllCloud__surfflat,  // assign(index, value) function pointer
    other_msgs__msg__AllCloud__rosidl_typesupport_introspection_c__resize_function__AllCloud__surfflat  // resize(index) function pointer
  },
  {
    "surflessflat",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(other_msgs__msg__AllCloud, surflessflat),  // bytes offset in struct
    NULL,  // default value
    other_msgs__msg__AllCloud__rosidl_typesupport_introspection_c__size_function__AllCloud__surflessflat,  // size() function pointer
    other_msgs__msg__AllCloud__rosidl_typesupport_introspection_c__get_const_function__AllCloud__surflessflat,  // get_const(index) function pointer
    other_msgs__msg__AllCloud__rosidl_typesupport_introspection_c__get_function__AllCloud__surflessflat,  // get(index) function pointer
    other_msgs__msg__AllCloud__rosidl_typesupport_introspection_c__fetch_function__AllCloud__surflessflat,  // fetch(index, &value) function pointer
    other_msgs__msg__AllCloud__rosidl_typesupport_introspection_c__assign_function__AllCloud__surflessflat,  // assign(index, value) function pointer
    other_msgs__msg__AllCloud__rosidl_typesupport_introspection_c__resize_function__AllCloud__surflessflat  // resize(index) function pointer
  },
  {
    "fullpointres",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(other_msgs__msg__AllCloud, fullpointres),  // bytes offset in struct
    NULL,  // default value
    other_msgs__msg__AllCloud__rosidl_typesupport_introspection_c__size_function__AllCloud__fullpointres,  // size() function pointer
    other_msgs__msg__AllCloud__rosidl_typesupport_introspection_c__get_const_function__AllCloud__fullpointres,  // get_const(index) function pointer
    other_msgs__msg__AllCloud__rosidl_typesupport_introspection_c__get_function__AllCloud__fullpointres,  // get(index) function pointer
    other_msgs__msg__AllCloud__rosidl_typesupport_introspection_c__fetch_function__AllCloud__fullpointres,  // fetch(index, &value) function pointer
    other_msgs__msg__AllCloud__rosidl_typesupport_introspection_c__assign_function__AllCloud__fullpointres,  // assign(index, value) function pointer
    other_msgs__msg__AllCloud__rosidl_typesupport_introspection_c__resize_function__AllCloud__fullpointres  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers other_msgs__msg__AllCloud__rosidl_typesupport_introspection_c__AllCloud_message_members = {
  "other_msgs__msg",  // message namespace
  "AllCloud",  // message name
  6,  // number of fields
  sizeof(other_msgs__msg__AllCloud),
  other_msgs__msg__AllCloud__rosidl_typesupport_introspection_c__AllCloud_message_member_array,  // message members
  other_msgs__msg__AllCloud__rosidl_typesupport_introspection_c__AllCloud_init_function,  // function to initialize message memory (memory has to be allocated)
  other_msgs__msg__AllCloud__rosidl_typesupport_introspection_c__AllCloud_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t other_msgs__msg__AllCloud__rosidl_typesupport_introspection_c__AllCloud_message_type_support_handle = {
  0,
  &other_msgs__msg__AllCloud__rosidl_typesupport_introspection_c__AllCloud_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_other_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, other_msgs, msg, AllCloud)() {
  other_msgs__msg__AllCloud__rosidl_typesupport_introspection_c__AllCloud_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  other_msgs__msg__AllCloud__rosidl_typesupport_introspection_c__AllCloud_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, other_msgs, msg, Point)();
  other_msgs__msg__AllCloud__rosidl_typesupport_introspection_c__AllCloud_message_member_array[2].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, other_msgs, msg, Point)();
  other_msgs__msg__AllCloud__rosidl_typesupport_introspection_c__AllCloud_message_member_array[3].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, other_msgs, msg, Point)();
  other_msgs__msg__AllCloud__rosidl_typesupport_introspection_c__AllCloud_message_member_array[4].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, other_msgs, msg, Point)();
  other_msgs__msg__AllCloud__rosidl_typesupport_introspection_c__AllCloud_message_member_array[5].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, other_msgs, msg, Point)();
  if (!other_msgs__msg__AllCloud__rosidl_typesupport_introspection_c__AllCloud_message_type_support_handle.typesupport_identifier) {
    other_msgs__msg__AllCloud__rosidl_typesupport_introspection_c__AllCloud_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &other_msgs__msg__AllCloud__rosidl_typesupport_introspection_c__AllCloud_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
