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

class Init_AllCloud_fullpointres
{
public:
  explicit Init_AllCloud_fullpointres(::other_msgs::msg::AllCloud & msg)
  : msg_(msg)
  {}
  ::other_msgs::msg::AllCloud fullpointres(::other_msgs::msg::AllCloud::_fullpointres_type arg)
  {
    msg_.fullpointres = std::move(arg);
    return std::move(msg_);
  }

private:
  ::other_msgs::msg::AllCloud msg_;
};

class Init_AllCloud_surflessflat
{
public:
  explicit Init_AllCloud_surflessflat(::other_msgs::msg::AllCloud & msg)
  : msg_(msg)
  {}
  Init_AllCloud_fullpointres surflessflat(::other_msgs::msg::AllCloud::_surflessflat_type arg)
  {
    msg_.surflessflat = std::move(arg);
    return Init_AllCloud_fullpointres(msg_);
  }

private:
  ::other_msgs::msg::AllCloud msg_;
};

class Init_AllCloud_surfflat
{
public:
  explicit Init_AllCloud_surfflat(::other_msgs::msg::AllCloud & msg)
  : msg_(msg)
  {}
  Init_AllCloud_surflessflat surfflat(::other_msgs::msg::AllCloud::_surfflat_type arg)
  {
    msg_.surfflat = std::move(arg);
    return Init_AllCloud_surflessflat(msg_);
  }

private:
  ::other_msgs::msg::AllCloud msg_;
};

class Init_AllCloud_cornerlesssharp
{
public:
  explicit Init_AllCloud_cornerlesssharp(::other_msgs::msg::AllCloud & msg)
  : msg_(msg)
  {}
  Init_AllCloud_surfflat cornerlesssharp(::other_msgs::msg::AllCloud::_cornerlesssharp_type arg)
  {
    msg_.cornerlesssharp = std::move(arg);
    return Init_AllCloud_surfflat(msg_);
  }

private:
  ::other_msgs::msg::AllCloud msg_;
};

class Init_AllCloud_cornersharp
{
public:
  explicit Init_AllCloud_cornersharp(::other_msgs::msg::AllCloud & msg)
  : msg_(msg)
  {}
  Init_AllCloud_cornerlesssharp cornersharp(::other_msgs::msg::AllCloud::_cornersharp_type arg)
  {
    msg_.cornersharp = std::move(arg);
    return Init_AllCloud_cornerlesssharp(msg_);
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
  Init_AllCloud_cornersharp header(::other_msgs::msg::AllCloud::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_AllCloud_cornersharp(msg_);
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
