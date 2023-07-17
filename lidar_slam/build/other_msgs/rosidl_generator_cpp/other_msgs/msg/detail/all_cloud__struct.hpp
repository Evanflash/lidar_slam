// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from other_msgs:msg/AllCloud.idl
// generated code does not contain a copyright notice

#ifndef OTHER_MSGS__MSG__DETAIL__ALL_CLOUD__STRUCT_HPP_
#define OTHER_MSGS__MSG__DETAIL__ALL_CLOUD__STRUCT_HPP_

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
// Member 'corner_less_sharp'
// Member 'surf_less_flat'
// Member 'ground_less_flat'
#include "other_msgs/msg/detail/point__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__other_msgs__msg__AllCloud __attribute__((deprecated))
#else
# define DEPRECATED__other_msgs__msg__AllCloud __declspec(deprecated)
#endif

namespace other_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct AllCloud_
{
  using Type = AllCloud_<ContainerAllocator>;

  explicit AllCloud_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    (void)_init;
  }

  explicit AllCloud_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _trans_form_type =
    std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>>;
  _trans_form_type trans_form;
  using _corner_less_sharp_type =
    std::vector<other_msgs::msg::Point_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<other_msgs::msg::Point_<ContainerAllocator>>>;
  _corner_less_sharp_type corner_less_sharp;
  using _surf_less_flat_type =
    std::vector<other_msgs::msg::Point_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<other_msgs::msg::Point_<ContainerAllocator>>>;
  _surf_less_flat_type surf_less_flat;
  using _ground_less_flat_type =
    std::vector<other_msgs::msg::Point_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<other_msgs::msg::Point_<ContainerAllocator>>>;
  _ground_less_flat_type ground_less_flat;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__trans_form(
    const std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>> & _arg)
  {
    this->trans_form = _arg;
    return *this;
  }
  Type & set__corner_less_sharp(
    const std::vector<other_msgs::msg::Point_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<other_msgs::msg::Point_<ContainerAllocator>>> & _arg)
  {
    this->corner_less_sharp = _arg;
    return *this;
  }
  Type & set__surf_less_flat(
    const std::vector<other_msgs::msg::Point_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<other_msgs::msg::Point_<ContainerAllocator>>> & _arg)
  {
    this->surf_less_flat = _arg;
    return *this;
  }
  Type & set__ground_less_flat(
    const std::vector<other_msgs::msg::Point_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<other_msgs::msg::Point_<ContainerAllocator>>> & _arg)
  {
    this->ground_less_flat = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    other_msgs::msg::AllCloud_<ContainerAllocator> *;
  using ConstRawPtr =
    const other_msgs::msg::AllCloud_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<other_msgs::msg::AllCloud_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<other_msgs::msg::AllCloud_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      other_msgs::msg::AllCloud_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<other_msgs::msg::AllCloud_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      other_msgs::msg::AllCloud_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<other_msgs::msg::AllCloud_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<other_msgs::msg::AllCloud_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<other_msgs::msg::AllCloud_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__other_msgs__msg__AllCloud
    std::shared_ptr<other_msgs::msg::AllCloud_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__other_msgs__msg__AllCloud
    std::shared_ptr<other_msgs::msg::AllCloud_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const AllCloud_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->trans_form != other.trans_form) {
      return false;
    }
    if (this->corner_less_sharp != other.corner_less_sharp) {
      return false;
    }
    if (this->surf_less_flat != other.surf_less_flat) {
      return false;
    }
    if (this->ground_less_flat != other.ground_less_flat) {
      return false;
    }
    return true;
  }
  bool operator!=(const AllCloud_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct AllCloud_

// alias to use template instance with default allocator
using AllCloud =
  other_msgs::msg::AllCloud_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace other_msgs

#endif  // OTHER_MSGS__MSG__DETAIL__ALL_CLOUD__STRUCT_HPP_
