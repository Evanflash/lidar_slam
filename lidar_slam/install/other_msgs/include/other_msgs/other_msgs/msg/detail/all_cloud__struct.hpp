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
// Member 'cornersharp'
// Member 'cornerlesssharp'
// Member 'surfflat'
// Member 'surflessflat'
// Member 'fullpointres'
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
  using _cornersharp_type =
    std::vector<other_msgs::msg::Point_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<other_msgs::msg::Point_<ContainerAllocator>>>;
  _cornersharp_type cornersharp;
  using _cornerlesssharp_type =
    std::vector<other_msgs::msg::Point_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<other_msgs::msg::Point_<ContainerAllocator>>>;
  _cornerlesssharp_type cornerlesssharp;
  using _surfflat_type =
    std::vector<other_msgs::msg::Point_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<other_msgs::msg::Point_<ContainerAllocator>>>;
  _surfflat_type surfflat;
  using _surflessflat_type =
    std::vector<other_msgs::msg::Point_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<other_msgs::msg::Point_<ContainerAllocator>>>;
  _surflessflat_type surflessflat;
  using _fullpointres_type =
    std::vector<other_msgs::msg::Point_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<other_msgs::msg::Point_<ContainerAllocator>>>;
  _fullpointres_type fullpointres;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__cornersharp(
    const std::vector<other_msgs::msg::Point_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<other_msgs::msg::Point_<ContainerAllocator>>> & _arg)
  {
    this->cornersharp = _arg;
    return *this;
  }
  Type & set__cornerlesssharp(
    const std::vector<other_msgs::msg::Point_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<other_msgs::msg::Point_<ContainerAllocator>>> & _arg)
  {
    this->cornerlesssharp = _arg;
    return *this;
  }
  Type & set__surfflat(
    const std::vector<other_msgs::msg::Point_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<other_msgs::msg::Point_<ContainerAllocator>>> & _arg)
  {
    this->surfflat = _arg;
    return *this;
  }
  Type & set__surflessflat(
    const std::vector<other_msgs::msg::Point_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<other_msgs::msg::Point_<ContainerAllocator>>> & _arg)
  {
    this->surflessflat = _arg;
    return *this;
  }
  Type & set__fullpointres(
    const std::vector<other_msgs::msg::Point_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<other_msgs::msg::Point_<ContainerAllocator>>> & _arg)
  {
    this->fullpointres = _arg;
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
    if (this->cornersharp != other.cornersharp) {
      return false;
    }
    if (this->cornerlesssharp != other.cornerlesssharp) {
      return false;
    }
    if (this->surfflat != other.surfflat) {
      return false;
    }
    if (this->surflessflat != other.surflessflat) {
      return false;
    }
    if (this->fullpointres != other.fullpointres) {
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
