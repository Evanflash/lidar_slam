// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from other_msgs:msg/SegCloud.idl
// generated code does not contain a copyright notice

#ifndef OTHER_MSGS__MSG__DETAIL__SEG_CLOUD__TRAITS_HPP_
#define OTHER_MSGS__MSG__DETAIL__SEG_CLOUD__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "other_msgs/msg/detail/seg_cloud__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'seg_cloud'
#include "other_msgs/msg/detail/point__traits.hpp"

namespace other_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const SegCloud & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: seg_ring_str_ind
  {
    if (msg.seg_ring_str_ind.size() == 0) {
      out << "seg_ring_str_ind: []";
    } else {
      out << "seg_ring_str_ind: [";
      size_t pending_items = msg.seg_ring_str_ind.size();
      for (auto item : msg.seg_ring_str_ind) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: seg_ring_end_ind
  {
    if (msg.seg_ring_end_ind.size() == 0) {
      out << "seg_ring_end_ind: []";
    } else {
      out << "seg_ring_end_ind: [";
      size_t pending_items = msg.seg_ring_end_ind.size();
      for (auto item : msg.seg_ring_end_ind) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: seg_cloud
  {
    if (msg.seg_cloud.size() == 0) {
      out << "seg_cloud: []";
    } else {
      out << "seg_cloud: [";
      size_t pending_items = msg.seg_cloud.size();
      for (auto item : msg.seg_cloud) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: seg_range
  {
    if (msg.seg_range.size() == 0) {
      out << "seg_range: []";
    } else {
      out << "seg_range: [";
      size_t pending_items = msg.seg_range.size();
      for (auto item : msg.seg_range) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: is_ground
  {
    if (msg.is_ground.size() == 0) {
      out << "is_ground: []";
    } else {
      out << "is_ground: [";
      size_t pending_items = msg.is_ground.size();
      for (auto item : msg.is_ground) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const SegCloud & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: header
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "header:\n";
    to_block_style_yaml(msg.header, out, indentation + 2);
  }

  // member: seg_ring_str_ind
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.seg_ring_str_ind.size() == 0) {
      out << "seg_ring_str_ind: []\n";
    } else {
      out << "seg_ring_str_ind:\n";
      for (auto item : msg.seg_ring_str_ind) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: seg_ring_end_ind
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.seg_ring_end_ind.size() == 0) {
      out << "seg_ring_end_ind: []\n";
    } else {
      out << "seg_ring_end_ind:\n";
      for (auto item : msg.seg_ring_end_ind) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: seg_cloud
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.seg_cloud.size() == 0) {
      out << "seg_cloud: []\n";
    } else {
      out << "seg_cloud:\n";
      for (auto item : msg.seg_cloud) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }

  // member: seg_range
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.seg_range.size() == 0) {
      out << "seg_range: []\n";
    } else {
      out << "seg_range:\n";
      for (auto item : msg.seg_range) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: is_ground
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.is_ground.size() == 0) {
      out << "is_ground: []\n";
    } else {
      out << "is_ground:\n";
      for (auto item : msg.is_ground) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const SegCloud & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace other_msgs

namespace rosidl_generator_traits
{

[[deprecated("use other_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const other_msgs::msg::SegCloud & msg,
  std::ostream & out, size_t indentation = 0)
{
  other_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use other_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const other_msgs::msg::SegCloud & msg)
{
  return other_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<other_msgs::msg::SegCloud>()
{
  return "other_msgs::msg::SegCloud";
}

template<>
inline const char * name<other_msgs::msg::SegCloud>()
{
  return "other_msgs/msg/SegCloud";
}

template<>
struct has_fixed_size<other_msgs::msg::SegCloud>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<other_msgs::msg::SegCloud>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<other_msgs::msg::SegCloud>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // OTHER_MSGS__MSG__DETAIL__SEG_CLOUD__TRAITS_HPP_
