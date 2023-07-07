// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from other_msgs:msg/AllCloud.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "other_msgs/msg/detail/all_cloud__struct.hpp"
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

void AllCloud_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) other_msgs::msg::AllCloud(_init);
}

void AllCloud_fini_function(void * message_memory)
{
  auto typed_message = static_cast<other_msgs::msg::AllCloud *>(message_memory);
  typed_message->~AllCloud();
}

size_t size_function__AllCloud__cornersharp(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<other_msgs::msg::Point> *>(untyped_member);
  return member->size();
}

const void * get_const_function__AllCloud__cornersharp(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<other_msgs::msg::Point> *>(untyped_member);
  return &member[index];
}

void * get_function__AllCloud__cornersharp(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<other_msgs::msg::Point> *>(untyped_member);
  return &member[index];
}

void fetch_function__AllCloud__cornersharp(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const other_msgs::msg::Point *>(
    get_const_function__AllCloud__cornersharp(untyped_member, index));
  auto & value = *reinterpret_cast<other_msgs::msg::Point *>(untyped_value);
  value = item;
}

void assign_function__AllCloud__cornersharp(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<other_msgs::msg::Point *>(
    get_function__AllCloud__cornersharp(untyped_member, index));
  const auto & value = *reinterpret_cast<const other_msgs::msg::Point *>(untyped_value);
  item = value;
}

void resize_function__AllCloud__cornersharp(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<other_msgs::msg::Point> *>(untyped_member);
  member->resize(size);
}

size_t size_function__AllCloud__cornerlesssharp(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<other_msgs::msg::Point> *>(untyped_member);
  return member->size();
}

const void * get_const_function__AllCloud__cornerlesssharp(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<other_msgs::msg::Point> *>(untyped_member);
  return &member[index];
}

void * get_function__AllCloud__cornerlesssharp(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<other_msgs::msg::Point> *>(untyped_member);
  return &member[index];
}

void fetch_function__AllCloud__cornerlesssharp(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const other_msgs::msg::Point *>(
    get_const_function__AllCloud__cornerlesssharp(untyped_member, index));
  auto & value = *reinterpret_cast<other_msgs::msg::Point *>(untyped_value);
  value = item;
}

void assign_function__AllCloud__cornerlesssharp(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<other_msgs::msg::Point *>(
    get_function__AllCloud__cornerlesssharp(untyped_member, index));
  const auto & value = *reinterpret_cast<const other_msgs::msg::Point *>(untyped_value);
  item = value;
}

void resize_function__AllCloud__cornerlesssharp(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<other_msgs::msg::Point> *>(untyped_member);
  member->resize(size);
}

size_t size_function__AllCloud__surfflat(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<other_msgs::msg::Point> *>(untyped_member);
  return member->size();
}

const void * get_const_function__AllCloud__surfflat(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<other_msgs::msg::Point> *>(untyped_member);
  return &member[index];
}

void * get_function__AllCloud__surfflat(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<other_msgs::msg::Point> *>(untyped_member);
  return &member[index];
}

void fetch_function__AllCloud__surfflat(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const other_msgs::msg::Point *>(
    get_const_function__AllCloud__surfflat(untyped_member, index));
  auto & value = *reinterpret_cast<other_msgs::msg::Point *>(untyped_value);
  value = item;
}

void assign_function__AllCloud__surfflat(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<other_msgs::msg::Point *>(
    get_function__AllCloud__surfflat(untyped_member, index));
  const auto & value = *reinterpret_cast<const other_msgs::msg::Point *>(untyped_value);
  item = value;
}

void resize_function__AllCloud__surfflat(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<other_msgs::msg::Point> *>(untyped_member);
  member->resize(size);
}

size_t size_function__AllCloud__surflessflat(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<other_msgs::msg::Point> *>(untyped_member);
  return member->size();
}

const void * get_const_function__AllCloud__surflessflat(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<other_msgs::msg::Point> *>(untyped_member);
  return &member[index];
}

void * get_function__AllCloud__surflessflat(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<other_msgs::msg::Point> *>(untyped_member);
  return &member[index];
}

void fetch_function__AllCloud__surflessflat(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const other_msgs::msg::Point *>(
    get_const_function__AllCloud__surflessflat(untyped_member, index));
  auto & value = *reinterpret_cast<other_msgs::msg::Point *>(untyped_value);
  value = item;
}

void assign_function__AllCloud__surflessflat(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<other_msgs::msg::Point *>(
    get_function__AllCloud__surflessflat(untyped_member, index));
  const auto & value = *reinterpret_cast<const other_msgs::msg::Point *>(untyped_value);
  item = value;
}

void resize_function__AllCloud__surflessflat(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<other_msgs::msg::Point> *>(untyped_member);
  member->resize(size);
}

size_t size_function__AllCloud__fullpointres(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<other_msgs::msg::Point> *>(untyped_member);
  return member->size();
}

const void * get_const_function__AllCloud__fullpointres(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<other_msgs::msg::Point> *>(untyped_member);
  return &member[index];
}

void * get_function__AllCloud__fullpointres(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<other_msgs::msg::Point> *>(untyped_member);
  return &member[index];
}

void fetch_function__AllCloud__fullpointres(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const other_msgs::msg::Point *>(
    get_const_function__AllCloud__fullpointres(untyped_member, index));
  auto & value = *reinterpret_cast<other_msgs::msg::Point *>(untyped_value);
  value = item;
}

void assign_function__AllCloud__fullpointres(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<other_msgs::msg::Point *>(
    get_function__AllCloud__fullpointres(untyped_member, index));
  const auto & value = *reinterpret_cast<const other_msgs::msg::Point *>(untyped_value);
  item = value;
}

void resize_function__AllCloud__fullpointres(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<other_msgs::msg::Point> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember AllCloud_message_member_array[6] = {
  {
    "header",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<std_msgs::msg::Header>(),  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(other_msgs::msg::AllCloud, header),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "cornersharp",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<other_msgs::msg::Point>(),  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(other_msgs::msg::AllCloud, cornersharp),  // bytes offset in struct
    nullptr,  // default value
    size_function__AllCloud__cornersharp,  // size() function pointer
    get_const_function__AllCloud__cornersharp,  // get_const(index) function pointer
    get_function__AllCloud__cornersharp,  // get(index) function pointer
    fetch_function__AllCloud__cornersharp,  // fetch(index, &value) function pointer
    assign_function__AllCloud__cornersharp,  // assign(index, value) function pointer
    resize_function__AllCloud__cornersharp  // resize(index) function pointer
  },
  {
    "cornerlesssharp",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<other_msgs::msg::Point>(),  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(other_msgs::msg::AllCloud, cornerlesssharp),  // bytes offset in struct
    nullptr,  // default value
    size_function__AllCloud__cornerlesssharp,  // size() function pointer
    get_const_function__AllCloud__cornerlesssharp,  // get_const(index) function pointer
    get_function__AllCloud__cornerlesssharp,  // get(index) function pointer
    fetch_function__AllCloud__cornerlesssharp,  // fetch(index, &value) function pointer
    assign_function__AllCloud__cornerlesssharp,  // assign(index, value) function pointer
    resize_function__AllCloud__cornerlesssharp  // resize(index) function pointer
  },
  {
    "surfflat",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<other_msgs::msg::Point>(),  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(other_msgs::msg::AllCloud, surfflat),  // bytes offset in struct
    nullptr,  // default value
    size_function__AllCloud__surfflat,  // size() function pointer
    get_const_function__AllCloud__surfflat,  // get_const(index) function pointer
    get_function__AllCloud__surfflat,  // get(index) function pointer
    fetch_function__AllCloud__surfflat,  // fetch(index, &value) function pointer
    assign_function__AllCloud__surfflat,  // assign(index, value) function pointer
    resize_function__AllCloud__surfflat  // resize(index) function pointer
  },
  {
    "surflessflat",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<other_msgs::msg::Point>(),  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(other_msgs::msg::AllCloud, surflessflat),  // bytes offset in struct
    nullptr,  // default value
    size_function__AllCloud__surflessflat,  // size() function pointer
    get_const_function__AllCloud__surflessflat,  // get_const(index) function pointer
    get_function__AllCloud__surflessflat,  // get(index) function pointer
    fetch_function__AllCloud__surflessflat,  // fetch(index, &value) function pointer
    assign_function__AllCloud__surflessflat,  // assign(index, value) function pointer
    resize_function__AllCloud__surflessflat  // resize(index) function pointer
  },
  {
    "fullpointres",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<other_msgs::msg::Point>(),  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(other_msgs::msg::AllCloud, fullpointres),  // bytes offset in struct
    nullptr,  // default value
    size_function__AllCloud__fullpointres,  // size() function pointer
    get_const_function__AllCloud__fullpointres,  // get_const(index) function pointer
    get_function__AllCloud__fullpointres,  // get(index) function pointer
    fetch_function__AllCloud__fullpointres,  // fetch(index, &value) function pointer
    assign_function__AllCloud__fullpointres,  // assign(index, value) function pointer
    resize_function__AllCloud__fullpointres  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers AllCloud_message_members = {
  "other_msgs::msg",  // message namespace
  "AllCloud",  // message name
  6,  // number of fields
  sizeof(other_msgs::msg::AllCloud),
  AllCloud_message_member_array,  // message members
  AllCloud_init_function,  // function to initialize message memory (memory has to be allocated)
  AllCloud_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t AllCloud_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &AllCloud_message_members,
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
get_message_type_support_handle<other_msgs::msg::AllCloud>()
{
  return &::other_msgs::msg::rosidl_typesupport_introspection_cpp::AllCloud_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, other_msgs, msg, AllCloud)() {
  return &::other_msgs::msg::rosidl_typesupport_introspection_cpp::AllCloud_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
