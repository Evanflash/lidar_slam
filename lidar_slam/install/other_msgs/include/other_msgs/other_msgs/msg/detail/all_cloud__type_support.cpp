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

size_t size_function__AllCloud__corner_sharp(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<other_msgs::msg::Point> *>(untyped_member);
  return member->size();
}

const void * get_const_function__AllCloud__corner_sharp(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<other_msgs::msg::Point> *>(untyped_member);
  return &member[index];
}

void * get_function__AllCloud__corner_sharp(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<other_msgs::msg::Point> *>(untyped_member);
  return &member[index];
}

void fetch_function__AllCloud__corner_sharp(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const other_msgs::msg::Point *>(
    get_const_function__AllCloud__corner_sharp(untyped_member, index));
  auto & value = *reinterpret_cast<other_msgs::msg::Point *>(untyped_value);
  value = item;
}

void assign_function__AllCloud__corner_sharp(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<other_msgs::msg::Point *>(
    get_function__AllCloud__corner_sharp(untyped_member, index));
  const auto & value = *reinterpret_cast<const other_msgs::msg::Point *>(untyped_value);
  item = value;
}

void resize_function__AllCloud__corner_sharp(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<other_msgs::msg::Point> *>(untyped_member);
  member->resize(size);
}

size_t size_function__AllCloud__corner_less_sharp(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<other_msgs::msg::Point> *>(untyped_member);
  return member->size();
}

const void * get_const_function__AllCloud__corner_less_sharp(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<other_msgs::msg::Point> *>(untyped_member);
  return &member[index];
}

void * get_function__AllCloud__corner_less_sharp(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<other_msgs::msg::Point> *>(untyped_member);
  return &member[index];
}

void fetch_function__AllCloud__corner_less_sharp(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const other_msgs::msg::Point *>(
    get_const_function__AllCloud__corner_less_sharp(untyped_member, index));
  auto & value = *reinterpret_cast<other_msgs::msg::Point *>(untyped_value);
  value = item;
}

void assign_function__AllCloud__corner_less_sharp(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<other_msgs::msg::Point *>(
    get_function__AllCloud__corner_less_sharp(untyped_member, index));
  const auto & value = *reinterpret_cast<const other_msgs::msg::Point *>(untyped_value);
  item = value;
}

void resize_function__AllCloud__corner_less_sharp(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<other_msgs::msg::Point> *>(untyped_member);
  member->resize(size);
}

size_t size_function__AllCloud__surf_flat(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<other_msgs::msg::Point> *>(untyped_member);
  return member->size();
}

const void * get_const_function__AllCloud__surf_flat(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<other_msgs::msg::Point> *>(untyped_member);
  return &member[index];
}

void * get_function__AllCloud__surf_flat(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<other_msgs::msg::Point> *>(untyped_member);
  return &member[index];
}

void fetch_function__AllCloud__surf_flat(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const other_msgs::msg::Point *>(
    get_const_function__AllCloud__surf_flat(untyped_member, index));
  auto & value = *reinterpret_cast<other_msgs::msg::Point *>(untyped_value);
  value = item;
}

void assign_function__AllCloud__surf_flat(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<other_msgs::msg::Point *>(
    get_function__AllCloud__surf_flat(untyped_member, index));
  const auto & value = *reinterpret_cast<const other_msgs::msg::Point *>(untyped_value);
  item = value;
}

void resize_function__AllCloud__surf_flat(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<other_msgs::msg::Point> *>(untyped_member);
  member->resize(size);
}

size_t size_function__AllCloud__surf_less_flat(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<other_msgs::msg::Point> *>(untyped_member);
  return member->size();
}

const void * get_const_function__AllCloud__surf_less_flat(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<other_msgs::msg::Point> *>(untyped_member);
  return &member[index];
}

void * get_function__AllCloud__surf_less_flat(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<other_msgs::msg::Point> *>(untyped_member);
  return &member[index];
}

void fetch_function__AllCloud__surf_less_flat(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const other_msgs::msg::Point *>(
    get_const_function__AllCloud__surf_less_flat(untyped_member, index));
  auto & value = *reinterpret_cast<other_msgs::msg::Point *>(untyped_value);
  value = item;
}

void assign_function__AllCloud__surf_less_flat(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<other_msgs::msg::Point *>(
    get_function__AllCloud__surf_less_flat(untyped_member, index));
  const auto & value = *reinterpret_cast<const other_msgs::msg::Point *>(untyped_value);
  item = value;
}

void resize_function__AllCloud__surf_less_flat(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<other_msgs::msg::Point> *>(untyped_member);
  member->resize(size);
}

size_t size_function__AllCloud__full_point_res(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<other_msgs::msg::Point> *>(untyped_member);
  return member->size();
}

const void * get_const_function__AllCloud__full_point_res(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<other_msgs::msg::Point> *>(untyped_member);
  return &member[index];
}

void * get_function__AllCloud__full_point_res(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<other_msgs::msg::Point> *>(untyped_member);
  return &member[index];
}

void fetch_function__AllCloud__full_point_res(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const other_msgs::msg::Point *>(
    get_const_function__AllCloud__full_point_res(untyped_member, index));
  auto & value = *reinterpret_cast<other_msgs::msg::Point *>(untyped_value);
  value = item;
}

void assign_function__AllCloud__full_point_res(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<other_msgs::msg::Point *>(
    get_function__AllCloud__full_point_res(untyped_member, index));
  const auto & value = *reinterpret_cast<const other_msgs::msg::Point *>(untyped_value);
  item = value;
}

void resize_function__AllCloud__full_point_res(void * untyped_member, size_t size)
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
    "corner_sharp",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<other_msgs::msg::Point>(),  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(other_msgs::msg::AllCloud, corner_sharp),  // bytes offset in struct
    nullptr,  // default value
    size_function__AllCloud__corner_sharp,  // size() function pointer
    get_const_function__AllCloud__corner_sharp,  // get_const(index) function pointer
    get_function__AllCloud__corner_sharp,  // get(index) function pointer
    fetch_function__AllCloud__corner_sharp,  // fetch(index, &value) function pointer
    assign_function__AllCloud__corner_sharp,  // assign(index, value) function pointer
    resize_function__AllCloud__corner_sharp  // resize(index) function pointer
  },
  {
    "corner_less_sharp",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<other_msgs::msg::Point>(),  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(other_msgs::msg::AllCloud, corner_less_sharp),  // bytes offset in struct
    nullptr,  // default value
    size_function__AllCloud__corner_less_sharp,  // size() function pointer
    get_const_function__AllCloud__corner_less_sharp,  // get_const(index) function pointer
    get_function__AllCloud__corner_less_sharp,  // get(index) function pointer
    fetch_function__AllCloud__corner_less_sharp,  // fetch(index, &value) function pointer
    assign_function__AllCloud__corner_less_sharp,  // assign(index, value) function pointer
    resize_function__AllCloud__corner_less_sharp  // resize(index) function pointer
  },
  {
    "surf_flat",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<other_msgs::msg::Point>(),  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(other_msgs::msg::AllCloud, surf_flat),  // bytes offset in struct
    nullptr,  // default value
    size_function__AllCloud__surf_flat,  // size() function pointer
    get_const_function__AllCloud__surf_flat,  // get_const(index) function pointer
    get_function__AllCloud__surf_flat,  // get(index) function pointer
    fetch_function__AllCloud__surf_flat,  // fetch(index, &value) function pointer
    assign_function__AllCloud__surf_flat,  // assign(index, value) function pointer
    resize_function__AllCloud__surf_flat  // resize(index) function pointer
  },
  {
    "surf_less_flat",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<other_msgs::msg::Point>(),  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(other_msgs::msg::AllCloud, surf_less_flat),  // bytes offset in struct
    nullptr,  // default value
    size_function__AllCloud__surf_less_flat,  // size() function pointer
    get_const_function__AllCloud__surf_less_flat,  // get_const(index) function pointer
    get_function__AllCloud__surf_less_flat,  // get(index) function pointer
    fetch_function__AllCloud__surf_less_flat,  // fetch(index, &value) function pointer
    assign_function__AllCloud__surf_less_flat,  // assign(index, value) function pointer
    resize_function__AllCloud__surf_less_flat  // resize(index) function pointer
  },
  {
    "full_point_res",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<other_msgs::msg::Point>(),  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(other_msgs::msg::AllCloud, full_point_res),  // bytes offset in struct
    nullptr,  // default value
    size_function__AllCloud__full_point_res,  // size() function pointer
    get_const_function__AllCloud__full_point_res,  // get_const(index) function pointer
    get_function__AllCloud__full_point_res,  // get(index) function pointer
    fetch_function__AllCloud__full_point_res,  // fetch(index, &value) function pointer
    assign_function__AllCloud__full_point_res,  // assign(index, value) function pointer
    resize_function__AllCloud__full_point_res  // resize(index) function pointer
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
