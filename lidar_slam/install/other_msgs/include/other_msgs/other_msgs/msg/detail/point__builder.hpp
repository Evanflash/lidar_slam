// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from other_msgs:msg/Point.idl
// generated code does not contain a copyright notice

#ifndef OTHER_MSGS__MSG__DETAIL__POINT__BUILDER_HPP_
#define OTHER_MSGS__MSG__DETAIL__POINT__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "other_msgs/msg/detail/point__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace other_msgs
{

namespace msg
{

namespace builder
{

class Init_Point_i
{
public:
  explicit Init_Point_i(::other_msgs::msg::Point & msg)
  : msg_(msg)
  {}
  ::other_msgs::msg::Point i(::other_msgs::msg::Point::_i_type arg)
  {
    msg_.i = std::move(arg);
    return std::move(msg_);
  }

private:
  ::other_msgs::msg::Point msg_;
};

class Init_Point_z
{
public:
  explicit Init_Point_z(::other_msgs::msg::Point & msg)
  : msg_(msg)
  {}
  Init_Point_i z(::other_msgs::msg::Point::_z_type arg)
  {
    msg_.z = std::move(arg);
    return Init_Point_i(msg_);
  }

private:
  ::other_msgs::msg::Point msg_;
};

class Init_Point_y
{
public:
  explicit Init_Point_y(::other_msgs::msg::Point & msg)
  : msg_(msg)
  {}
  Init_Point_z y(::other_msgs::msg::Point::_y_type arg)
  {
    msg_.y = std::move(arg);
    return Init_Point_z(msg_);
  }

private:
  ::other_msgs::msg::Point msg_;
};

class Init_Point_x
{
public:
  Init_Point_x()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Point_y x(::other_msgs::msg::Point::_x_type arg)
  {
    msg_.x = std::move(arg);
    return Init_Point_y(msg_);
  }

private:
  ::other_msgs::msg::Point msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::other_msgs::msg::Point>()
{
  return other_msgs::msg::builder::Init_Point_x();
}

}  // namespace other_msgs

#endif  // OTHER_MSGS__MSG__DETAIL__POINT__BUILDER_HPP_
