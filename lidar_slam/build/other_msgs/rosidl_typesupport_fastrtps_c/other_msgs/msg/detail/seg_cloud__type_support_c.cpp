// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from other_msgs:msg/SegCloud.idl
// generated code does not contain a copyright notice
#include "other_msgs/msg/detail/seg_cloud__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "other_msgs/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "other_msgs/msg/detail/seg_cloud__struct.h"
#include "other_msgs/msg/detail/seg_cloud__functions.h"
#include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif

#include "other_msgs/msg/detail/point__functions.h"  // seg_cloud
#include "rosidl_runtime_c/primitives_sequence.h"  // is_ground, seg_cloud_col_ind, seg_range, seg_ring_end_ind, seg_ring_str_ind
#include "rosidl_runtime_c/primitives_sequence_functions.h"  // is_ground, seg_cloud_col_ind, seg_range, seg_ring_end_ind, seg_ring_str_ind
#include "std_msgs/msg/detail/header__functions.h"  // header

// forward declare type support functions
size_t get_serialized_size_other_msgs__msg__Point(
  const void * untyped_ros_message,
  size_t current_alignment);

size_t max_serialized_size_other_msgs__msg__Point(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, other_msgs, msg, Point)();
ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_other_msgs
size_t get_serialized_size_std_msgs__msg__Header(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_other_msgs
size_t max_serialized_size_std_msgs__msg__Header(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_other_msgs
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, std_msgs, msg, Header)();


using _SegCloud__ros_msg_type = other_msgs__msg__SegCloud;

static bool _SegCloud__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _SegCloud__ros_msg_type * ros_message = static_cast<const _SegCloud__ros_msg_type *>(untyped_ros_message);
  // Field name: header
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, std_msgs, msg, Header
      )()->data);
    if (!callbacks->cdr_serialize(
        &ros_message->header, cdr))
    {
      return false;
    }
  }

  // Field name: seg_ring_str_ind
  {
    size_t size = ros_message->seg_ring_str_ind.size;
    auto array_ptr = ros_message->seg_ring_str_ind.data;
    cdr << static_cast<uint32_t>(size);
    cdr.serializeArray(array_ptr, size);
  }

  // Field name: seg_ring_end_ind
  {
    size_t size = ros_message->seg_ring_end_ind.size;
    auto array_ptr = ros_message->seg_ring_end_ind.data;
    cdr << static_cast<uint32_t>(size);
    cdr.serializeArray(array_ptr, size);
  }

  // Field name: seg_cloud
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, other_msgs, msg, Point
      )()->data);
    size_t size = ros_message->seg_cloud.size;
    auto array_ptr = ros_message->seg_cloud.data;
    cdr << static_cast<uint32_t>(size);
    for (size_t i = 0; i < size; ++i) {
      if (!callbacks->cdr_serialize(
          &array_ptr[i], cdr))
      {
        return false;
      }
    }
  }

  // Field name: seg_range
  {
    size_t size = ros_message->seg_range.size;
    auto array_ptr = ros_message->seg_range.data;
    cdr << static_cast<uint32_t>(size);
    cdr.serializeArray(array_ptr, size);
  }

  // Field name: is_ground
  {
    size_t size = ros_message->is_ground.size;
    auto array_ptr = ros_message->is_ground.data;
    cdr << static_cast<uint32_t>(size);
    cdr.serializeArray(array_ptr, size);
  }

  // Field name: seg_cloud_col_ind
  {
    size_t size = ros_message->seg_cloud_col_ind.size;
    auto array_ptr = ros_message->seg_cloud_col_ind.data;
    cdr << static_cast<uint32_t>(size);
    cdr.serializeArray(array_ptr, size);
  }

  return true;
}

static bool _SegCloud__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _SegCloud__ros_msg_type * ros_message = static_cast<_SegCloud__ros_msg_type *>(untyped_ros_message);
  // Field name: header
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, std_msgs, msg, Header
      )()->data);
    if (!callbacks->cdr_deserialize(
        cdr, &ros_message->header))
    {
      return false;
    }
  }

  // Field name: seg_ring_str_ind
  {
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);
    if (ros_message->seg_ring_str_ind.data) {
      rosidl_runtime_c__int32__Sequence__fini(&ros_message->seg_ring_str_ind);
    }
    if (!rosidl_runtime_c__int32__Sequence__init(&ros_message->seg_ring_str_ind, size)) {
      fprintf(stderr, "failed to create array for field 'seg_ring_str_ind'");
      return false;
    }
    auto array_ptr = ros_message->seg_ring_str_ind.data;
    cdr.deserializeArray(array_ptr, size);
  }

  // Field name: seg_ring_end_ind
  {
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);
    if (ros_message->seg_ring_end_ind.data) {
      rosidl_runtime_c__int32__Sequence__fini(&ros_message->seg_ring_end_ind);
    }
    if (!rosidl_runtime_c__int32__Sequence__init(&ros_message->seg_ring_end_ind, size)) {
      fprintf(stderr, "failed to create array for field 'seg_ring_end_ind'");
      return false;
    }
    auto array_ptr = ros_message->seg_ring_end_ind.data;
    cdr.deserializeArray(array_ptr, size);
  }

  // Field name: seg_cloud
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, other_msgs, msg, Point
      )()->data);
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);
    if (ros_message->seg_cloud.data) {
      other_msgs__msg__Point__Sequence__fini(&ros_message->seg_cloud);
    }
    if (!other_msgs__msg__Point__Sequence__init(&ros_message->seg_cloud, size)) {
      fprintf(stderr, "failed to create array for field 'seg_cloud'");
      return false;
    }
    auto array_ptr = ros_message->seg_cloud.data;
    for (size_t i = 0; i < size; ++i) {
      if (!callbacks->cdr_deserialize(
          cdr, &array_ptr[i]))
      {
        return false;
      }
    }
  }

  // Field name: seg_range
  {
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);
    if (ros_message->seg_range.data) {
      rosidl_runtime_c__float__Sequence__fini(&ros_message->seg_range);
    }
    if (!rosidl_runtime_c__float__Sequence__init(&ros_message->seg_range, size)) {
      fprintf(stderr, "failed to create array for field 'seg_range'");
      return false;
    }
    auto array_ptr = ros_message->seg_range.data;
    cdr.deserializeArray(array_ptr, size);
  }

  // Field name: is_ground
  {
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);
    if (ros_message->is_ground.data) {
      rosidl_runtime_c__int32__Sequence__fini(&ros_message->is_ground);
    }
    if (!rosidl_runtime_c__int32__Sequence__init(&ros_message->is_ground, size)) {
      fprintf(stderr, "failed to create array for field 'is_ground'");
      return false;
    }
    auto array_ptr = ros_message->is_ground.data;
    cdr.deserializeArray(array_ptr, size);
  }

  // Field name: seg_cloud_col_ind
  {
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);
    if (ros_message->seg_cloud_col_ind.data) {
      rosidl_runtime_c__int32__Sequence__fini(&ros_message->seg_cloud_col_ind);
    }
    if (!rosidl_runtime_c__int32__Sequence__init(&ros_message->seg_cloud_col_ind, size)) {
      fprintf(stderr, "failed to create array for field 'seg_cloud_col_ind'");
      return false;
    }
    auto array_ptr = ros_message->seg_cloud_col_ind.data;
    cdr.deserializeArray(array_ptr, size);
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_other_msgs
size_t get_serialized_size_other_msgs__msg__SegCloud(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _SegCloud__ros_msg_type * ros_message = static_cast<const _SegCloud__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name header

  current_alignment += get_serialized_size_std_msgs__msg__Header(
    &(ros_message->header), current_alignment);
  // field.name seg_ring_str_ind
  {
    size_t array_size = ros_message->seg_ring_str_ind.size;
    auto array_ptr = ros_message->seg_ring_str_ind.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name seg_ring_end_ind
  {
    size_t array_size = ros_message->seg_ring_end_ind.size;
    auto array_ptr = ros_message->seg_ring_end_ind.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name seg_cloud
  {
    size_t array_size = ros_message->seg_cloud.size;
    auto array_ptr = ros_message->seg_cloud.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += get_serialized_size_other_msgs__msg__Point(
        &array_ptr[index], current_alignment);
    }
  }
  // field.name seg_range
  {
    size_t array_size = ros_message->seg_range.size;
    auto array_ptr = ros_message->seg_range.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name is_ground
  {
    size_t array_size = ros_message->is_ground.size;
    auto array_ptr = ros_message->is_ground.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name seg_cloud_col_ind
  {
    size_t array_size = ros_message->seg_cloud_col_ind.size;
    auto array_ptr = ros_message->seg_cloud_col_ind.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _SegCloud__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_other_msgs__msg__SegCloud(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_other_msgs
size_t max_serialized_size_other_msgs__msg__SegCloud(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;

  // member: header
  {
    size_t array_size = 1;


    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      current_alignment +=
        max_serialized_size_std_msgs__msg__Header(
        inner_full_bounded, inner_is_plain, current_alignment);
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }
  // member: seg_ring_str_ind
  {
    size_t array_size = 0;
    full_bounded = false;
    is_plain = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: seg_ring_end_ind
  {
    size_t array_size = 0;
    full_bounded = false;
    is_plain = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: seg_cloud
  {
    size_t array_size = 0;
    full_bounded = false;
    is_plain = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);


    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      current_alignment +=
        max_serialized_size_other_msgs__msg__Point(
        inner_full_bounded, inner_is_plain, current_alignment);
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }
  // member: seg_range
  {
    size_t array_size = 0;
    full_bounded = false;
    is_plain = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: is_ground
  {
    size_t array_size = 0;
    full_bounded = false;
    is_plain = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: seg_cloud_col_ind
  {
    size_t array_size = 0;
    full_bounded = false;
    is_plain = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  return current_alignment - initial_alignment;
}

static size_t _SegCloud__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_other_msgs__msg__SegCloud(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_SegCloud = {
  "other_msgs::msg",
  "SegCloud",
  _SegCloud__cdr_serialize,
  _SegCloud__cdr_deserialize,
  _SegCloud__get_serialized_size,
  _SegCloud__max_serialized_size
};

static rosidl_message_type_support_t _SegCloud__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_SegCloud,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, other_msgs, msg, SegCloud)() {
  return &_SegCloud__type_support;
}

#if defined(__cplusplus)
}
#endif
