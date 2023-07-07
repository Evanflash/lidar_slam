// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from other_msgs:msg/AllCloud.idl
// generated code does not contain a copyright notice

#ifndef OTHER_MSGS__MSG__DETAIL__ALL_CLOUD__BUILDER_HPP_
#define OTHER_MSGS__MSG__DETAIL__ALL_CLOUD__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "other_msgs/msg/detail/all_cloud__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace other_msgs
{

namespace msg
{

namespace builder
{

class Init_AllCloud_full_point_res
{
public:
  explicit Init_AllCloud_full_point_res(::other_msgs::msg::AllCloud & msg)
  : msg_(msg)
  {}
  ::other_msgs::msg::AllCloud full_point_res(::other_msgs::msg::AllCloud::_full_point_res_type arg)
  {
    msg_.full_point_res = std::move(arg);
    return std::move(msg_);
  }

private:
  ::other_msgs::msg::AllCloud msg_;
};

class Init_AllCloud_surf_less_flat
{
public:
  explicit Init_AllCloud_surf_less_flat(::other_msgs::msg::AllCloud & msg)
  : msg_(msg)
  {}
  Init_AllCloud_full_point_res surf_less_flat(::other_msgs::msg::AllCloud::_surf_less_flat_type arg)
  {
    msg_.surf_less_flat = std::move(arg);
    return Init_AllCloud_full_point_res(msg_);
  }

private:
  ::other_msgs::msg::AllCloud msg_;
};

class Init_AllCloud_surf_flat
{
public:
  explicit Init_AllCloud_surf_flat(::other_msgs::msg::AllCloud & msg)
  : msg_(msg)
  {}
  Init_AllCloud_surf_less_flat surf_flat(::other_msgs::msg::AllCloud::_surf_flat_type arg)
  {
    msg_.surf_flat = std::move(arg);
    return Init_AllCloud_surf_less_flat(msg_);
  }

private:
  ::other_msgs::msg::AllCloud msg_;
};

class Init_AllCloud_corner_less_sharp
{
public:
  explicit Init_AllCloud_corner_less_sharp(::other_msgs::msg::AllCloud & msg)
  : msg_(msg)
  {}
  Init_AllCloud_surf_flat corner_less_sharp(::other_msgs::msg::AllCloud::_corner_less_sharp_type arg)
  {
    msg_.corner_less_sharp = std::move(arg);
    return Init_AllCloud_surf_flat(msg_);
  }

private:
  ::other_msgs::msg::AllCloud msg_;
};

class Init_AllCloud_corner_sharp
{
public:
  explicit Init_AllCloud_corner_sharp(::other_msgs::msg::AllCloud & msg)
  : msg_(msg)
  {}
  Init_AllCloud_corner_less_sharp corner_sharp(::other_msgs::msg::AllCloud::_corner_sharp_type arg)
  {
    msg_.corner_sharp = std::move(arg);
    return Init_AllCloud_corner_less_sharp(msg_);
  }

private:
  ::other_msgs::msg::AllCloud msg_;
};

class Init_AllCloud_header
{
public:
  Init_AllCloud_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_AllCloud_corner_sharp header(::other_msgs::msg::AllCloud::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_AllCloud_corner_sharp(msg_);
  }

private:
  ::other_msgs::msg::AllCloud msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::other_msgs::msg::AllCloud>()
{
  return other_msgs::msg::builder::Init_AllCloud_header();
}

}  // namespace other_msgs

#endif  // OTHER_MSGS__MSG__DETAIL__ALL_CLOUD__BUILDER_HPP_
