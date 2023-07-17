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
// Member `trans_form`
#include "rosidl_runtime_c/primitives_sequence_functions.h"
// Member `corner_less_sharp`
// Member `surf_less_flat`
// Member `ground_less_flat`
#include "other_msgs/msg/point.h"
// Member `corner_less_sharp`
// Member `surf_less_flat`
// Member `ground_less_flat`
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

size_t other_msgs__msg__AllCloud__rosidl_typesupport_introspection_c__size_function__AllCloud__trans_form(
  const void * untyped_member)
{
  const rosidl_runtime_c__float__Sequence * member =
    (const rosidl_runtime_c__float__Sequence *)(untyped_member);
  return member->size;
}

const void * other_msgs__msg__AllCloud__rosidl_typesupport_introspection_c__get_const_function__AllCloud__trans_form(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__float__Sequence * member =
    (const rosidl_runtime_c__float__Sequence *)(untyped_member);
  return &member->data[index];
}

void * other_msgs__msg__AllCloud__rosidl_typesupport_introspection_c__get_function__AllCloud__trans_form(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__float__Sequence * member =
    (rosidl_runtime_c__float__Sequence *)(untyped_member);
  return &member->data[index];
}

void other_msgs__msg__AllCloud__rosidl_typesupport_introspection_c__fetch_function__AllCloud__trans_form(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const float * item =
    ((const float *)
    other_msgs__msg__AllCloud__rosidl_typesupport_introspection_c__get_const_function__AllCloud__trans_form(untyped_member, index));
  float * value =
    (float *)(untyped_value);
  *value = *item;
}

void other_msgs__msg__AllCloud__rosidl_typesupport_introspection_c__assign_function__AllCloud__trans_form(
  void * untyped_member, size_t index, const void * untyped_value)
{
  float * item =
    ((float *)
    other_msgs__msg__AllCloud__rosidl_typesupport_introspection_c__get_function__AllCloud__trans_form(untyped_member, index));
  const float * value =
    (const float *)(untyped_value);
  *item = *value;
}

bool other_msgs__msg__AllCloud__rosidl_typesupport_introspection_c__resize_function__AllCloud__trans_form(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__float__Sequence * member =
    (rosidl_runtime_c__float__Sequence *)(untyped_member);
  rosidl_runtime_c__float__Sequence__fini(member);
  return rosidl_runtime_c__float__Sequence__init(member, size);
}

size_t other_msgs__msg__AllCloud__rosidl_typesupport_introspection_c__size_function__AllCloud__corner_less_sharp(
  const void * untyped_member)
{
  const other_msgs__msg__Point__Sequence * member =
    (const other_msgs__msg__Point__Sequence *)(untyped_member);
  return member->size;
}

const void * other_msgs__msg__AllCloud__rosidl_typesupport_introspection_c__get_const_function__AllCloud__corner_less_sharp(
  const void * untyped_member, size_t index)
{
  const other_msgs__msg__Point__Sequence * member =
    (const other_msgs__msg__Point__Sequence *)(untyped_member);
  return &member->data[index];
}

void * other_msgs__msg__AllCloud__rosidl_typesupport_introspection_c__get_function__AllCloud__corner_less_sharp(
  void * untyped_member, size_t index)
{
  other_msgs__msg__Point__Sequence * member =
    (other_msgs__msg__Point__Sequence *)(untyped_member);
  return &member->data[index];
}

void other_msgs__msg__AllCloud__rosidl_typesupport_introspection_c__fetch_function__AllCloud__corner_less_sharp(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const other_msgs__msg__Point * item =
    ((const other_msgs__msg__Point *)
    other_msgs__msg__AllCloud__rosidl_typesupport_introspection_c__get_const_function__AllCloud__corner_less_sharp(untyped_member, index));
  other_msgs__msg__Point * value =
    (other_msgs__msg__Point *)(untyped_value);
  *value = *item;
}

void other_msgs__msg__AllCloud__rosidl_typesupport_introspection_c__assign_function__AllCloud__corner_less_sharp(
  void * untyped_member, size_t index, const void * untyped_value)
{
  other_msgs__msg__Point * item =
    ((other_msgs__msg__Point *)
    other_msgs__msg__AllCloud__rosidl_typesupport_introspection_c__get_function__AllCloud__corner_less_sharp(untyped_member, index));
  const other_msgs__msg__Point * value =
    (const other_msgs__msg__Point *)(untyped_value);
  *item = *value;
}

bool other_msgs__msg__AllCloud__rosidl_typesupport_introspection_c__resize_function__AllCloud__corner_less_sharp(
  void * untyped_member, size_t size)
{
  other_msgs__msg__Point__Sequence * member =
    (other_msgs__msg__Point__Sequence *)(untyped_member);
  other_msgs__msg__Point__Sequence__fini(member);
  return other_msgs__msg__Point__Sequence__init(member, size);
}

size_t other_msgs__msg__AllCloud__rosidl_typesupport_introspection_c__size_function__AllCloud__surf_less_flat(
  const void * untyped_member)
{
  const other_msgs__msg__Point__Sequence * member =
    (const other_msgs__msg__Point__Sequence *)(untyped_member);
  return member->size;
}

const void * other_msgs__msg__AllCloud__rosidl_typesupport_introspection_c__get_const_function__AllCloud__surf_less_flat(
  const void * untyped_member, size_t index)
{
  const other_msgs__msg__Point__Sequence * member =
    (const other_msgs__msg__Point__Sequence *)(untyped_member);
  return &member->data[index];
}

void * other_msgs__msg__AllCloud__rosidl_typesupport_introspection_c__get_function__AllCloud__surf_less_flat(
  void * untyped_member, size_t index)
{
  other_msgs__msg__Point__Sequence * member =
    (other_msgs__msg__Point__Sequence *)(untyped_member);
  return &member->data[index];
}

void other_msgs__msg__AllCloud__rosidl_typesupport_introspection_c__fetch_function__AllCloud__surf_less_flat(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const other_msgs__msg__Point * item =
    ((const other_msgs__msg__Point *)
    other_msgs__msg__AllCloud__rosidl_typesupport_introspection_c__get_const_function__AllCloud__surf_less_flat(untyped_member, index));
  other_msgs__msg__Point * value =
    (other_msgs__msg__Point *)(untyped_value);
  *value = *item;
}

void other_msgs__msg__AllCloud__rosidl_typesupport_introspection_c__assign_function__AllCloud__surf_less_flat(
  void * untyped_member, size_t index, const void * untyped_value)
{
  other_msgs__msg__Point * item =
    ((other_msgs__msg__Point *)
    other_msgs__msg__AllCloud__rosidl_typesupport_introspection_c__get_function__AllCloud__surf_less_flat(untyped_member, index));
  const other_msgs__msg__Point * value =
    (const other_msgs__msg__Point *)(untyped_value);
  *item = *value;
}

bool other_msgs__msg__AllCloud__rosidl_typesupport_introspection_c__resize_function__AllCloud__surf_less_flat(
  void * untyped_member, size_t size)
{
  other_msgs__msg__Point__Sequence * member =
    (other_msgs__msg__Point__Sequence *)(untyped_member);
  other_msgs__msg__Point__Sequence__fini(member);
  return other_msgs__msg__Point__Sequence__init(member, size);
}

size_t other_msgs__msg__AllCloud__rosidl_typesupport_introspection_c__size_function__AllCloud__ground_less_flat(
  const void * untyped_member)
{
  const other_msgs__msg__Point__Sequence * member =
    (const other_msgs__msg__Point__Sequence *)(untyped_member);
  return member->size;
}

const void * other_msgs__msg__AllCloud__rosidl_typesupport_introspection_c__get_const_function__AllCloud__ground_less_flat(
  const void * untyped_member, size_t index)
{
  const other_msgs__msg__Point__Sequence * member =
    (const other_msgs__msg__Point__Sequence *)(untyped_member);
  return &member->data[index];
}

void * other_msgs__msg__AllCloud__rosidl_typesupport_introspection_c__get_function__AllCloud__ground_less_flat(
  void * untyped_member, size_t index)
{
  other_msgs__msg__Point__Sequence * member =
    (other_msgs__msg__Point__Sequence *)(untyped_member);
  return &member->data[index];
}

void other_msgs__msg__AllCloud__rosidl_typesupport_introspection_c__fetch_function__AllCloud__ground_less_flat(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const other_msgs__msg__Point * item =
    ((const other_msgs__msg__Point *)
    other_msgs__msg__AllCloud__rosidl_typesupport_introspection_c__get_const_function__AllCloud__ground_less_flat(untyped_member, index));
  other_msgs__msg__Point * value =
    (other_msgs__msg__Point *)(untyped_value);
  *value = *item;
}

void other_msgs__msg__AllCloud__rosidl_typesupport_introspection_c__assign_function__AllCloud__ground_less_flat(
  void * untyped_member, size_t index, const void * untyped_value)
{
  other_msgs__msg__Point * item =
    ((other_msgs__msg__Point *)
    other_msgs__msg__AllCloud__rosidl_typesupport_introspection_c__get_function__AllCloud__ground_less_flat(untyped_member, index));
  const other_msgs__msg__Point * value =
    (const other_msgs__msg__Point *)(untyped_value);
  *item = *value;
}

bool other_msgs__msg__AllCloud__rosidl_typesupport_introspection_c__resize_function__AllCloud__ground_less_flat(
  void * untyped_member, size_t size)
{
  other_msgs__msg__Point__Sequence * member =
    (other_msgs__msg__Point__Sequence *)(untyped_member);
  other_msgs__msg__Point__Sequence__fini(member);
  return other_msgs__msg__Point__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember other_msgs__msg__AllCloud__rosidl_typesupport_introspection_c__AllCloud_message_member_array[5] = {
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
    "trans_form",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(other_msgs__msg__AllCloud, trans_form),  // bytes offset in struct
    NULL,  // default value
    other_msgs__msg__AllCloud__rosidl_typesupport_introspection_c__size_function__AllCloud__trans_form,  // size() function pointer
    other_msgs__msg__AllCloud__rosidl_typesupport_introspection_c__get_const_function__AllCloud__trans_form,  // get_const(index) function pointer
    other_msgs__msg__AllCloud__rosidl_typesupport_introspection_c__get_function__AllCloud__trans_form,  // get(index) function pointer
    other_msgs__msg__AllCloud__rosidl_typesupport_introspection_c__fetch_function__AllCloud__trans_form,  // fetch(index, &value) function pointer
    other_msgs__msg__AllCloud__rosidl_typesupport_introspection_c__assign_function__AllCloud__trans_form,  // assign(index, value) function pointer
    other_msgs__msg__AllCloud__rosidl_typesupport_introspection_c__resize_function__AllCloud__trans_form  // resize(index) function pointer
  },
  {
    "corner_less_sharp",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(other_msgs__msg__AllCloud, corner_less_sharp),  // bytes offset in struct
    NULL,  // default value
    other_msgs__msg__AllCloud__rosidl_typesupport_introspection_c__size_function__AllCloud__corner_less_sharp,  // size() function pointer
    other_msgs__msg__AllCloud__rosidl_typesupport_introspection_c__get_const_function__AllCloud__corner_less_sharp,  // get_const(index) function pointer
    other_msgs__msg__AllCloud__rosidl_typesupport_introspection_c__get_function__AllCloud__corner_less_sharp,  // get(index) function pointer
    other_msgs__msg__AllCloud__rosidl_typesupport_introspection_c__fetch_function__AllCloud__corner_less_sharp,  // fetch(index, &value) function pointer
    other_msgs__msg__AllCloud__rosidl_typesupport_introspection_c__assign_function__AllCloud__corner_less_sharp,  // assign(index, value) function pointer
    other_msgs__msg__AllCloud__rosidl_typesupport_introspection_c__resize_function__AllCloud__corner_less_sharp  // resize(index) function pointer
  },
  {
    "surf_less_flat",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(other_msgs__msg__AllCloud, surf_less_flat),  // bytes offset in struct
    NULL,  // default value
    other_msgs__msg__AllCloud__rosidl_typesupport_introspection_c__size_function__AllCloud__surf_less_flat,  // size() function pointer
    other_msgs__msg__AllCloud__rosidl_typesupport_introspection_c__get_const_function__AllCloud__surf_less_flat,  // get_const(index) function pointer
    other_msgs__msg__AllCloud__rosidl_typesupport_introspection_c__get_function__AllCloud__surf_less_flat,  // get(index) function pointer
    other_msgs__msg__AllCloud__rosidl_typesupport_introspection_c__fetch_function__AllCloud__surf_less_flat,  // fetch(index, &value) function pointer
    other_msgs__msg__AllCloud__rosidl_typesupport_introspection_c__assign_function__AllCloud__surf_less_flat,  // assign(index, value) function pointer
    other_msgs__msg__AllCloud__rosidl_typesupport_introspection_c__resize_function__AllCloud__surf_less_flat  // resize(index) function pointer
  },
  {
    "ground_less_flat",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(other_msgs__msg__AllCloud, ground_less_flat),  // bytes offset in struct
    NULL,  // default value
    other_msgs__msg__AllCloud__rosidl_typesupport_introspection_c__size_function__AllCloud__ground_less_flat,  // size() function pointer
    other_msgs__msg__AllCloud__rosidl_typesupport_introspection_c__get_const_function__AllCloud__ground_less_flat,  // get_const(index) function pointer
    other_msgs__msg__AllCloud__rosidl_typesupport_introspection_c__get_function__AllCloud__ground_less_flat,  // get(index) function pointer
    other_msgs__msg__AllCloud__rosidl_typesupport_introspection_c__fetch_function__AllCloud__ground_less_flat,  // fetch(index, &value) function pointer
    other_msgs__msg__AllCloud__rosidl_typesupport_introspection_c__assign_function__AllCloud__ground_less_flat,  // assign(index, value) function pointer
    other_msgs__msg__AllCloud__rosidl_typesupport_introspection_c__resize_function__AllCloud__ground_less_flat  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers other_msgs__msg__AllCloud__rosidl_typesupport_introspection_c__AllCloud_message_members = {
  "other_msgs__msg",  // message namespace
  "AllCloud",  // message name
  5,  // number of fields
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
  other_msgs__msg__AllCloud__rosidl_typesupport_introspection_c__AllCloud_message_member_array[2].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, other_msgs, msg, Point)();
  other_msgs__msg__AllCloud__rosidl_typesupport_introspection_c__AllCloud_message_member_array[3].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, other_msgs, msg, Point)();
  other_msgs__msg__AllCloud__rosidl_typesupport_introspection_c__AllCloud_message_member_array[4].members_ =
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
