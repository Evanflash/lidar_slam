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
// Member 'surf_flat'
// Member 'surf_less_flat'
// Member 'ground_flat'
// Member 'ground_less_flat'
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

  // member: trans_form
  {
    if (msg.trans_form.size() == 0) {
      out << "trans_form: []";
    } else {
      out << "trans_form: [";
      size_t pending_items = msg.trans_form.size();
      for (auto item : msg.trans_form) {
        rosidl_generator_traits::value_to_yaml(item, out);
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

  // member: ground_flat
  {
    if (msg.ground_flat.size() == 0) {
      out << "ground_flat: []";
    } else {
      out << "ground_flat: [";
      size_t pending_items = msg.ground_flat.size();
      for (auto item : msg.ground_flat) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: ground_less_flat
  {
    if (msg.ground_less_flat.size() == 0) {
      out << "ground_less_flat: []";
    } else {
      out << "ground_less_flat: [";
      size_t pending_items = msg.ground_less_flat.size();
      for (auto item : msg.ground_less_flat) {
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

  // member: trans_form
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.trans_form.size() == 0) {
      out << "trans_form: []\n";
    } else {
      out << "trans_form:\n";
      for (auto item : msg.trans_form) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
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

  // member: ground_flat
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.ground_flat.size() == 0) {
      out << "ground_flat: []\n";
    } else {
      out << "ground_flat:\n";
      for (auto item : msg.ground_flat) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }

  // member: ground_less_flat
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.ground_less_flat.size() == 0) {
      out << "ground_less_flat: []\n";
    } else {
      out << "ground_less_flat:\n";
      for (auto item : msg.ground_less_flat) {
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
