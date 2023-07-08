// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from other_msgs:msg/SegCloud.idl
// generated code does not contain a copyright notice

#ifndef OTHER_MSGS__MSG__DETAIL__SEG_CLOUD__BUILDER_HPP_
#define OTHER_MSGS__MSG__DETAIL__SEG_CLOUD__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "other_msgs/msg/detail/seg_cloud__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace other_msgs
{

namespace msg
{

namespace builder
{

class Init_SegCloud_ground_range
{
public:
  explicit Init_SegCloud_ground_range(::other_msgs::msg::SegCloud & msg)
  : msg_(msg)
  {}
  ::other_msgs::msg::SegCloud ground_range(::other_msgs::msg::SegCloud::_ground_range_type arg)
  {
    msg_.ground_range = std::move(arg);
    return std::move(msg_);
  }

private:
  ::other_msgs::msg::SegCloud msg_;
};

class Init_SegCloud_ground_cloud
{
public:
  explicit Init_SegCloud_ground_cloud(::other_msgs::msg::SegCloud & msg)
  : msg_(msg)
  {}
  Init_SegCloud_ground_range ground_cloud(::other_msgs::msg::SegCloud::_ground_cloud_type arg)
  {
    msg_.ground_cloud = std::move(arg);
    return Init_SegCloud_ground_range(msg_);
  }

private:
  ::other_msgs::msg::SegCloud msg_;
};

class Init_SegCloud_grd_ring_end_ind
{
public:
  explicit Init_SegCloud_grd_ring_end_ind(::other_msgs::msg::SegCloud & msg)
  : msg_(msg)
  {}
  Init_SegCloud_ground_cloud grd_ring_end_ind(::other_msgs::msg::SegCloud::_grd_ring_end_ind_type arg)
  {
    msg_.grd_ring_end_ind = std::move(arg);
    return Init_SegCloud_ground_cloud(msg_);
  }

private:
  ::other_msgs::msg::SegCloud msg_;
};

class Init_SegCloud_grd_ring_str_ind
{
public:
  explicit Init_SegCloud_grd_ring_str_ind(::other_msgs::msg::SegCloud & msg)
  : msg_(msg)
  {}
  Init_SegCloud_grd_ring_end_ind grd_ring_str_ind(::other_msgs::msg::SegCloud::_grd_ring_str_ind_type arg)
  {
    msg_.grd_ring_str_ind = std::move(arg);
    return Init_SegCloud_grd_ring_end_ind(msg_);
  }

private:
  ::other_msgs::msg::SegCloud msg_;
};

class Init_SegCloud_seg_range
{
public:
  explicit Init_SegCloud_seg_range(::other_msgs::msg::SegCloud & msg)
  : msg_(msg)
  {}
  Init_SegCloud_grd_ring_str_ind seg_range(::other_msgs::msg::SegCloud::_seg_range_type arg)
  {
    msg_.seg_range = std::move(arg);
    return Init_SegCloud_grd_ring_str_ind(msg_);
  }

private:
  ::other_msgs::msg::SegCloud msg_;
};

class Init_SegCloud_seg_cloud
{
public:
  explicit Init_SegCloud_seg_cloud(::other_msgs::msg::SegCloud & msg)
  : msg_(msg)
  {}
  Init_SegCloud_seg_range seg_cloud(::other_msgs::msg::SegCloud::_seg_cloud_type arg)
  {
    msg_.seg_cloud = std::move(arg);
    return Init_SegCloud_seg_range(msg_);
  }

private:
  ::other_msgs::msg::SegCloud msg_;
};

class Init_SegCloud_seg_ring_end_ind
{
public:
  explicit Init_SegCloud_seg_ring_end_ind(::other_msgs::msg::SegCloud & msg)
  : msg_(msg)
  {}
  Init_SegCloud_seg_cloud seg_ring_end_ind(::other_msgs::msg::SegCloud::_seg_ring_end_ind_type arg)
  {
    msg_.seg_ring_end_ind = std::move(arg);
    return Init_SegCloud_seg_cloud(msg_);
  }

private:
  ::other_msgs::msg::SegCloud msg_;
};

class Init_SegCloud_seg_ring_str_ind
{
public:
  explicit Init_SegCloud_seg_ring_str_ind(::other_msgs::msg::SegCloud & msg)
  : msg_(msg)
  {}
  Init_SegCloud_seg_ring_end_ind seg_ring_str_ind(::other_msgs::msg::SegCloud::_seg_ring_str_ind_type arg)
  {
    msg_.seg_ring_str_ind = std::move(arg);
    return Init_SegCloud_seg_ring_end_ind(msg_);
  }

private:
  ::other_msgs::msg::SegCloud msg_;
};

class Init_SegCloud_header
{
public:
  Init_SegCloud_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SegCloud_seg_ring_str_ind header(::other_msgs::msg::SegCloud::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_SegCloud_seg_ring_str_ind(msg_);
  }

private:
  ::other_msgs::msg::SegCloud msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::other_msgs::msg::SegCloud>()
{
  return other_msgs::msg::builder::Init_SegCloud_header();
}

}  // namespace other_msgs

#endif  // OTHER_MSGS__MSG__DETAIL__SEG_CLOUD__BUILDER_HPP_
