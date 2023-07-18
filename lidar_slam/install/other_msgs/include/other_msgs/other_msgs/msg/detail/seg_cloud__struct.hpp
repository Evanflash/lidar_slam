// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from other_msgs:msg/SegCloud.idl
// generated code does not contain a copyright notice

#ifndef OTHER_MSGS__MSG__DETAIL__SEG_CLOUD__STRUCT_HPP_
#define OTHER_MSGS__MSG__DETAIL__SEG_CLOUD__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.hpp"
// Member 'seg_cloud'
#include "other_msgs/msg/detail/point__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__other_msgs__msg__SegCloud __attribute__((deprecated))
#else
# define DEPRECATED__other_msgs__msg__SegCloud __declspec(deprecated)
#endif

namespace other_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct SegCloud_
{
  using Type = SegCloud_<ContainerAllocator>;

  explicit SegCloud_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    (void)_init;
  }

  explicit SegCloud_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _seg_ring_str_ind_type =
    std::vector<int32_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<int32_t>>;
  _seg_ring_str_ind_type seg_ring_str_ind;
  using _seg_ring_end_ind_type =
    std::vector<int32_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<int32_t>>;
  _seg_ring_end_ind_type seg_ring_end_ind;
  using _seg_cloud_type =
    std::vector<other_msgs::msg::Point_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<other_msgs::msg::Point_<ContainerAllocator>>>;
  _seg_cloud_type seg_cloud;
  using _seg_range_type =
    std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>>;
  _seg_range_type seg_range;
  using _is_ground_type =
    std::vector<int32_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<int32_t>>;
  _is_ground_type is_ground;
  using _seg_cloud_col_ind_type =
    std::vector<int32_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<int32_t>>;
  _seg_cloud_col_ind_type seg_cloud_col_ind;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__seg_ring_str_ind(
    const std::vector<int32_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<int32_t>> & _arg)
  {
    this->seg_ring_str_ind = _arg;
    return *this;
  }
  Type & set__seg_ring_end_ind(
    const std::vector<int32_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<int32_t>> & _arg)
  {
    this->seg_ring_end_ind = _arg;
    return *this;
  }
  Type & set__seg_cloud(
    const std::vector<other_msgs::msg::Point_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<other_msgs::msg::Point_<ContainerAllocator>>> & _arg)
  {
    this->seg_cloud = _arg;
    return *this;
  }
  Type & set__seg_range(
    const std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>> & _arg)
  {
    this->seg_range = _arg;
    return *this;
  }
  Type & set__is_ground(
    const std::vector<int32_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<int32_t>> & _arg)
  {
    this->is_ground = _arg;
    return *this;
  }
  Type & set__seg_cloud_col_ind(
    const std::vector<int32_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<int32_t>> & _arg)
  {
    this->seg_cloud_col_ind = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    other_msgs::msg::SegCloud_<ContainerAllocator> *;
  using ConstRawPtr =
    const other_msgs::msg::SegCloud_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<other_msgs::msg::SegCloud_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<other_msgs::msg::SegCloud_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      other_msgs::msg::SegCloud_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<other_msgs::msg::SegCloud_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      other_msgs::msg::SegCloud_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<other_msgs::msg::SegCloud_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<other_msgs::msg::SegCloud_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<other_msgs::msg::SegCloud_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__other_msgs__msg__SegCloud
    std::shared_ptr<other_msgs::msg::SegCloud_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__other_msgs__msg__SegCloud
    std::shared_ptr<other_msgs::msg::SegCloud_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SegCloud_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->seg_ring_str_ind != other.seg_ring_str_ind) {
      return false;
    }
    if (this->seg_ring_end_ind != other.seg_ring_end_ind) {
      return false;
    }
    if (this->seg_cloud != other.seg_cloud) {
      return false;
    }
    if (this->seg_range != other.seg_range) {
      return false;
    }
    if (this->is_ground != other.is_ground) {
      return false;
    }
    if (this->seg_cloud_col_ind != other.seg_cloud_col_ind) {
      return false;
    }
    return true;
  }
  bool operator!=(const SegCloud_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SegCloud_

// alias to use template instance with default allocator
using SegCloud =
  other_msgs::msg::SegCloud_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace other_msgs

#endif  // OTHER_MSGS__MSG__DETAIL__SEG_CLOUD__STRUCT_HPP_
