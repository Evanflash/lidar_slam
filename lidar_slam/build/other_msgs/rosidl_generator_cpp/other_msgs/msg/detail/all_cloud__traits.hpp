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
// Member 'cornersharp'
// Member 'cornerlesssharp'
// Member 'surfflat'
// Member 'surflessflat'
// Member 'fullpointres'
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

  // member: cornersharp
  {
    if (msg.cornersharp.size() == 0) {
      out << "cornersharp: []";
    } else {
      out << "cornersharp: [";
      size_t pending_items = msg.cornersharp.size();
      for (auto item : msg.cornersharp) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: cornerlesssharp
  {
    if (msg.cornerlesssharp.size() == 0) {
      out << "cornerlesssharp: []";
    } else {
      out << "cornerlesssharp: [";
      size_t pending_items = msg.cornerlesssharp.size();
      for (auto item : msg.cornerlesssharp) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: surfflat
  {
    if (msg.surfflat.size() == 0) {
      out << "surfflat: []";
    } else {
      out << "surfflat: [";
      size_t pending_items = msg.surfflat.size();
      for (auto item : msg.surfflat) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: surflessflat
  {
    if (msg.surflessflat.size() == 0) {
      out << "surflessflat: []";
    } else {
      out << "surflessflat: [";
      size_t pending_items = msg.surflessflat.size();
      for (auto item : msg.surflessflat) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: fullpointres
  {
    if (msg.fullpointres.size() == 0) {
      out << "fullpointres: []";
    } else {
      out << "fullpointres: [";
      size_t pending_items = msg.fullpointres.size();
      for (auto item : msg.fullpointres) {
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

  // member: cornersharp
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.cornersharp.size() == 0) {
      out << "cornersharp: []\n";
    } else {
      out << "cornersharp:\n";
      for (auto item : msg.cornersharp) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }

  // member: cornerlesssharp
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.cornerlesssharp.size() == 0) {
      out << "cornerlesssharp: []\n";
    } else {
      out << "cornerlesssharp:\n";
      for (auto item : msg.cornerlesssharp) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }

  // member: surfflat
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.surfflat.size() == 0) {
      out << "surfflat: []\n";
    } else {
      out << "surfflat:\n";
      for (auto item : msg.surfflat) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }

  // member: surflessflat
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.surflessflat.size() == 0) {
      out << "surflessflat: []\n";
    } else {
      out << "surflessflat:\n";
      for (auto item : msg.surflessflat) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }

  // member: fullpointres
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.fullpointres.size() == 0) {
      out << "fullpointres: []\n";
    } else {
      out << "fullpointres:\n";
      for (auto item : msg.fullpointres) {
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
