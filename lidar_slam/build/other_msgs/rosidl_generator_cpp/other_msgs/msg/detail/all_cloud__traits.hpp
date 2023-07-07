// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from other_msgs:msg/AllCloud.idl
// generated code does not contain a copyright notice

#ifndef OTHER_MSGS__MSG__DETAIL__ALL_CLOUD__TRAITS_HPP_
#define OTHER_MSGS__MSG__DETAIL__ALL_CLOUD__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "other_msgs/msg/detail/all_cloud__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'corner_sharp'
// Member 'corner_less_sharp'
// Member 'surf_flat'
// Member 'surf_less_flat'
// Member 'full_point_res'
#include "other_msgs/msg/detail/point__traits.hpp"

namespace other_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const AllCloud & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: corner_sharp
  {
    if (msg.corner_sharp.size() == 0) {
      out << "corner_sharp: []";
    } else {
      out << "corner_sharp: [";
      size_t pending_items = msg.corner_sharp.size();
      for (auto item : msg.corner_sharp) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: corner_less_sharp
  {
    if (msg.corner_less_sharp.size() == 0) {
      out << "corner_less_sharp: []";
    } else {
      out << "corner_less_sharp: [";
      size_t pending_items = msg.corner_less_sharp.size();
      for (auto item : msg.corner_less_sharp) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: surf_flat
  {
    if (msg.surf_flat.size() == 0) {
      out << "surf_flat: []";
    } else {
      out << "surf_flat: [";
      size_t pending_items = msg.surf_flat.size();
      for (auto item : msg.surf_flat) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: surf_less_flat
  {
    if (msg.surf_less_flat.size() == 0) {
      out << "surf_less_flat: []";
    } else {
      out << "surf_less_flat: [";
      size_t pending_items = msg.surf_less_flat.size();
      for (auto item : msg.surf_less_flat) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: full_point_res
  {
    if (msg.full_point_res.size() == 0) {
      out << "full_point_res: []";
    } else {
      out << "full_point_res: [";
      size_t pending_items = msg.full_point_res.size();
      for (auto item : msg.full_point_res) {
        to_flow_style_yaml(item, out);
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
  const AllCloud & msg,
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

  // member: corner_sharp
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.corner_sharp.size() == 0) {
      out << "corner_sharp: []\n";
    } else {
      out << "corner_sharp:\n";
      for (auto item : msg.corner_sharp) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }

  // member: corner_less_sharp
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.corner_less_sharp.size() == 0) {
      out << "corner_less_sharp: []\n";
    } else {
      out << "corner_less_sharp:\n";
      for (auto item : msg.corner_less_sharp) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }

  // member: surf_flat
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.surf_flat.size() == 0) {
      out << "surf_flat: []\n";
    } else {
      out << "surf_flat:\n";
      for (auto item : msg.surf_flat) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }

  // member: surf_less_flat
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.surf_less_flat.size() == 0) {
      out << "surf_less_flat: []\n";
    } else {
      out << "surf_less_flat:\n";
      for (auto item : msg.surf_less_flat) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }

  // member: full_point_res
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.full_point_res.size() == 0) {
      out << "full_point_res: []\n";
    } else {
      out << "full_point_res:\n";
      for (auto item : msg.full_point_res) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const AllCloud & msg, bool use_flow_style = false)
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
  const other_msgs::msg::AllCloud & msg,
  std::ostream & out, size_t indentation = 0)
{
  other_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use other_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const other_msgs::msg::AllCloud & msg)
{
  return other_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<other_msgs::msg::AllCloud>()
{
  return "other_msgs::msg::AllCloud";
}

template<>
inline const char * name<other_msgs::msg::AllCloud>()
{
  return "other_msgs/msg/AllCloud";
}

template<>
struct has_fixed_size<other_msgs::msg::AllCloud>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<other_msgs::msg::AllCloud>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<other_msgs::msg::AllCloud>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // OTHER_MSGS__MSG__DETAIL__ALL_CLOUD__TRAITS_HPP_
