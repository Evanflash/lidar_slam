// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from other_msgs:msg/SegCloud.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "other_msgs/msg/detail/seg_cloud__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace other_msgs
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void SegCloud_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) other_msgs::msg::SegCloud(_init);
}

void SegCloud_fini_function(void * message_memory)
{
  auto typed_message = static_cast<other_msgs::msg::SegCloud *>(message_memory);
  typed_message->~SegCloud();
}

size_t size_function__SegCloud__seg_ring_str_ind(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<int32_t> *>(untyped_member);
  return member->size();
}

const void * get_const_function__SegCloud__seg_ring_str_ind(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<int32_t> *>(untyped_member);
  return &member[index];
}

void * get_function__SegCloud__seg_ring_str_ind(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<int32_t> *>(untyped_member);
  return &member[index];
}

void fetch_function__SegCloud__seg_ring_str_ind(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const int32_t *>(
    get_const_function__SegCloud__seg_ring_str_ind(untyped_member, index));
  auto & value = *reinterpret_cast<int32_t *>(untyped_value);
  value = item;
}

void assign_function__SegCloud__seg_ring_str_ind(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<int32_t *>(
    get_function__SegCloud__seg_ring_str_ind(untyped_member, index));
  const auto & value = *reinterpret_cast<const int32_t *>(untyped_value);
  item = value;
}

void resize_function__SegCloud__seg_ring_str_ind(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<int32_t> *>(untyped_member);
  member->resize(size);
}

size_t size_function__SegCloud__seg_ring_end_ind(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<int32_t> *>(untyped_member);
  return member->size();
}

const void * get_const_function__SegCloud__seg_ring_end_ind(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<int32_t> *>(untyped_member);
  return &member[index];
}

void * get_function__SegCloud__seg_ring_end_ind(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<int32_t> *>(untyped_member);
  return &member[index];
}

void fetch_function__SegCloud__seg_ring_end_ind(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const int32_t *>(
    get_const_function__SegCloud__seg_ring_end_ind(untyped_member, index));
  auto & value = *reinterpret_cast<int32_t *>(untyped_value);
  value = item;
}

void assign_function__SegCloud__seg_ring_end_ind(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<int32_t *>(
    get_function__SegCloud__seg_ring_end_ind(untyped_member, index));
  const auto & value = *reinterpret_cast<const int32_t *>(untyped_value);
  item = value;
}

void resize_function__SegCloud__seg_ring_end_ind(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<int32_t> *>(untyped_member);
  member->resize(size);
}

size_t size_function__SegCloud__seg_cloud(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<other_msgs::msg::Point> *>(untyped_member);
  return member->size();
}

const void * get_const_function__SegCloud__seg_cloud(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<other_msgs::msg::Point> *>(untyped_member);
  return &member[index];
}

void * get_function__SegCloud__seg_cloud(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<other_msgs::msg::Point> *>(untyped_member);
  return &member[index];
}

void fetch_function__SegCloud__seg_cloud(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const other_msgs::msg::Point *>(
    get_const_function__SegCloud__seg_cloud(untyped_member, index));
  auto & value = *reinterpret_cast<other_msgs::msg::Point *>(untyped_value);
  value = item;
}

void assign_function__SegCloud__seg_cloud(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<other_msgs::msg::Point *>(
    get_function__SegCloud__seg_cloud(untyped_member, index));
  const auto & value = *reinterpret_cast<const other_msgs::msg::Point *>(untyped_value);
  item = value;
}

void resize_function__SegCloud__seg_cloud(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<other_msgs::msg::Point> *>(untyped_member);
  member->resize(size);
}

size_t size_function__SegCloud__seg_range(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<float> *>(untyped_member);
  return member->size();
}

const void * get_const_function__SegCloud__seg_range(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<float> *>(untyped_member);
  return &member[index];
}

void * get_function__SegCloud__seg_range(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<float> *>(untyped_member);
  return &member[index];
}

void fetch_function__SegCloud__seg_range(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const float *>(
    get_const_function__SegCloud__seg_range(untyped_member, index));
  auto & value = *reinterpret_cast<float *>(untyped_value);
  value = item;
}

void assign_function__SegCloud__seg_range(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<float *>(
    get_function__SegCloud__seg_range(untyped_member, index));
  const auto & value = *reinterpret_cast<const float *>(untyped_value);
  item = value;
}

void resize_function__SegCloud__seg_range(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<float> *>(untyped_member);
  member->resize(size);
}

size_t size_function__SegCloud__is_ground(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<int32_t> *>(untyped_member);
  return member->size();
}

const void * get_const_function__SegCloud__is_ground(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<int32_t> *>(untyped_member);
  return &member[index];
}

void * get_function__SegCloud__is_ground(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<int32_t> *>(untyped_member);
  return &member[index];
}

void fetch_function__SegCloud__is_ground(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const int32_t *>(
    get_const_function__SegCloud__is_ground(untyped_member, index));
  auto & value = *reinterpret_cast<int32_t *>(untyped_value);
  value = item;
}

void assign_function__SegCloud__is_ground(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<int32_t *>(
    get_function__SegCloud__is_ground(untyped_member, index));
  const auto & value = *reinterpret_cast<const int32_t *>(untyped_value);
  item = value;
}

void resize_function__SegCloud__is_ground(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<int32_t> *>(untyped_member);
  member->resize(size);
}

size_t size_function__SegCloud__seg_cloud_col_ind(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<int32_t> *>(untyped_member);
  return member->size();
}

const void * get_const_function__SegCloud__seg_cloud_col_ind(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<int32_t> *>(untyped_member);
  return &member[index];
}

void * get_function__SegCloud__seg_cloud_col_ind(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<int32_t> *>(untyped_member);
  return &member[index];
}

void fetch_function__SegCloud__seg_cloud_col_ind(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const int32_t *>(
    get_const_function__SegCloud__seg_cloud_col_ind(untyped_member, index));
  auto & value = *reinterpret_cast<int32_t *>(untyped_value);
  value = item;
}

void assign_function__SegCloud__seg_cloud_col_ind(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<int32_t *>(
    get_function__SegCloud__seg_cloud_col_ind(untyped_member, index));
  const auto & value = *reinterpret_cast<const int32_t *>(untyped_value);
  item = value;
}

void resize_function__SegCloud__seg_cloud_col_ind(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<int32_t> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember SegCloud_message_member_array[7] = {
  {
    "header",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<std_msgs::msg::Header>(),  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(other_msgs::msg::SegCloud, header),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "seg_ring_str_ind",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(other_msgs::msg::SegCloud, seg_ring_str_ind),  // bytes offset in struct
    nullptr,  // default value
    size_function__SegCloud__seg_ring_str_ind,  // size() function pointer
    get_const_function__SegCloud__seg_ring_str_ind,  // get_const(index) function pointer
    get_function__SegCloud__seg_ring_str_ind,  // get(index) function pointer
    fetch_function__SegCloud__seg_ring_str_ind,  // fetch(index, &value) function pointer
    assign_function__SegCloud__seg_ring_str_ind,  // assign(index, value) function pointer
    resize_function__SegCloud__seg_ring_str_ind  // resize(index) function pointer
  },
  {
    "seg_ring_end_ind",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(other_msgs::msg::SegCloud, seg_ring_end_ind),  // bytes offset in struct
    nullptr,  // default value
    size_function__SegCloud__seg_ring_end_ind,  // size() function pointer
    get_const_function__SegCloud__seg_ring_end_ind,  // get_const(index) function pointer
    get_function__SegCloud__seg_ring_end_ind,  // get(index) function pointer
    fetch_function__SegCloud__seg_ring_end_ind,  // fetch(index, &value) function pointer
    assign_function__SegCloud__seg_ring_end_ind,  // assign(index, value) function pointer
    resize_function__SegCloud__seg_ring_end_ind  // resize(index) function pointer
  },
  {
    "seg_cloud",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<other_msgs::msg::Point>(),  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(other_msgs::msg::SegCloud, seg_cloud),  // bytes offset in struct
    nullptr,  // default value
    size_function__SegCloud__seg_cloud,  // size() function pointer
    get_const_function__SegCloud__seg_cloud,  // get_const(index) function pointer
    get_function__SegCloud__seg_cloud,  // get(index) function pointer
    fetch_function__SegCloud__seg_cloud,  // fetch(index, &value) function pointer
    assign_function__SegCloud__seg_cloud,  // assign(index, value) function pointer
    resize_function__SegCloud__seg_cloud  // resize(index) function pointer
  },
  {
    "seg_range",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(other_msgs::msg::SegCloud, seg_range),  // bytes offset in struct
    nullptr,  // default value
    size_function__SegCloud__seg_range,  // size() function pointer
    get_const_function__SegCloud__seg_range,  // get_const(index) function pointer
    get_function__SegCloud__seg_range,  // get(index) function pointer
    fetch_function__SegCloud__seg_range,  // fetch(index, &value) function pointer
    assign_function__SegCloud__seg_range,  // assign(index, value) function pointer
    resize_function__SegCloud__seg_range  // resize(index) function pointer
  },
  {
    "is_ground",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(other_msgs::msg::SegCloud, is_ground),  // bytes offset in struct
    nullptr,  // default value
    size_function__SegCloud__is_ground,  // size() function pointer
    get_const_function__SegCloud__is_ground,  // get_const(index) function pointer
    get_function__SegCloud__is_ground,  // get(index) function pointer
    fetch_function__SegCloud__is_ground,  // fetch(index, &value) function pointer
    assign_function__SegCloud__is_ground,  // assign(index, value) function pointer
    resize_function__SegCloud__is_ground  // resize(index) function pointer
  },
  {
    "seg_cloud_col_ind",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(other_msgs::msg::SegCloud, seg_cloud_col_ind),  // bytes offset in struct
    nullptr,  // default value
    size_function__SegCloud__seg_cloud_col_ind,  // size() function pointer
    get_const_function__SegCloud__seg_cloud_col_ind,  // get_const(index) function pointer
    get_function__SegCloud__seg_cloud_col_ind,  // get(index) function pointer
    fetch_function__SegCloud__seg_cloud_col_ind,  // fetch(index, &value) function pointer
    assign_function__SegCloud__seg_cloud_col_ind,  // assign(index, value) function pointer
    resize_function__SegCloud__seg_cloud_col_ind  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers SegCloud_message_members = {
  "other_msgs::msg",  // message namespace
  "SegCloud",  // message name
  7,  // number of fields
  sizeof(other_msgs::msg::SegCloud),
  SegCloud_message_member_array,  // message members
  SegCloud_init_function,  // function to initialize message memory (memory has to be allocated)
  SegCloud_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t SegCloud_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &SegCloud_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace other_msgs


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<other_msgs::msg::SegCloud>()
{
  return &::other_msgs::msg::rosidl_typesupport_introspection_cpp::SegCloud_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, other_msgs, msg, SegCloud)() {
  return &::other_msgs::msg::rosidl_typesupport_introspection_cpp::SegCloud_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
