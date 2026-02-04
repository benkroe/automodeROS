// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from automode_interfaces:msg/RobotState.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "automode_interfaces/msg/robot_state.hpp"


#ifndef AUTOMODE_INTERFACES__MSG__DETAIL__ROBOT_STATE__TRAITS_HPP_
#define AUTOMODE_INTERFACES__MSG__DETAIL__ROBOT_STATE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "automode_interfaces/msg/detail/robot_state__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__traits.hpp"

namespace automode_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const RobotState & msg,
  std::ostream & out)
{
  out << "{";
  // member: robot_id
  {
    out << "robot_id: ";
    rosidl_generator_traits::value_to_yaml(msg.robot_id, out);
    out << ", ";
  }

  // member: stamp
  {
    out << "stamp: ";
    to_flow_style_yaml(msg.stamp, out);
    out << ", ";
  }

  // member: floor_color
  {
    out << "floor_color: ";
    rosidl_generator_traits::value_to_yaml(msg.floor_color, out);
    out << ", ";
  }

  // member: proximity_magnitude
  {
    out << "proximity_magnitude: ";
    rosidl_generator_traits::value_to_yaml(msg.proximity_magnitude, out);
    out << ", ";
  }

  // member: proximity_angle
  {
    out << "proximity_angle: ";
    rosidl_generator_traits::value_to_yaml(msg.proximity_angle, out);
    out << ", ";
  }

  // member: light_magnitude
  {
    out << "light_magnitude: ";
    rosidl_generator_traits::value_to_yaml(msg.light_magnitude, out);
    out << ", ";
  }

  // member: light_angle
  {
    out << "light_angle: ";
    rosidl_generator_traits::value_to_yaml(msg.light_angle, out);
    out << ", ";
  }

  // member: target_magnitude
  {
    out << "target_magnitude: ";
    rosidl_generator_traits::value_to_yaml(msg.target_magnitude, out);
    out << ", ";
  }

  // member: target_position
  {
    out << "target_position: ";
    rosidl_generator_traits::value_to_yaml(msg.target_position, out);
    out << ", ";
  }

  // member: attraction_angle
  {
    out << "attraction_angle: ";
    rosidl_generator_traits::value_to_yaml(msg.attraction_angle, out);
    out << ", ";
  }

  // member: neighbour_count
  {
    out << "neighbour_count: ";
    rosidl_generator_traits::value_to_yaml(msg.neighbour_count, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const RobotState & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: robot_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "robot_id: ";
    rosidl_generator_traits::value_to_yaml(msg.robot_id, out);
    out << "\n";
  }

  // member: stamp
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "stamp:\n";
    to_block_style_yaml(msg.stamp, out, indentation + 2);
  }

  // member: floor_color
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "floor_color: ";
    rosidl_generator_traits::value_to_yaml(msg.floor_color, out);
    out << "\n";
  }

  // member: proximity_magnitude
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "proximity_magnitude: ";
    rosidl_generator_traits::value_to_yaml(msg.proximity_magnitude, out);
    out << "\n";
  }

  // member: proximity_angle
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "proximity_angle: ";
    rosidl_generator_traits::value_to_yaml(msg.proximity_angle, out);
    out << "\n";
  }

  // member: light_magnitude
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "light_magnitude: ";
    rosidl_generator_traits::value_to_yaml(msg.light_magnitude, out);
    out << "\n";
  }

  // member: light_angle
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "light_angle: ";
    rosidl_generator_traits::value_to_yaml(msg.light_angle, out);
    out << "\n";
  }

  // member: target_magnitude
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "target_magnitude: ";
    rosidl_generator_traits::value_to_yaml(msg.target_magnitude, out);
    out << "\n";
  }

  // member: target_position
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "target_position: ";
    rosidl_generator_traits::value_to_yaml(msg.target_position, out);
    out << "\n";
  }

  // member: attraction_angle
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "attraction_angle: ";
    rosidl_generator_traits::value_to_yaml(msg.attraction_angle, out);
    out << "\n";
  }

  // member: neighbour_count
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "neighbour_count: ";
    rosidl_generator_traits::value_to_yaml(msg.neighbour_count, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const RobotState & msg, bool use_flow_style = false)
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

}  // namespace automode_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use automode_interfaces::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const automode_interfaces::msg::RobotState & msg,
  std::ostream & out, size_t indentation = 0)
{
  automode_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use automode_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const automode_interfaces::msg::RobotState & msg)
{
  return automode_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<automode_interfaces::msg::RobotState>()
{
  return "automode_interfaces::msg::RobotState";
}

template<>
inline const char * name<automode_interfaces::msg::RobotState>()
{
  return "automode_interfaces/msg/RobotState";
}

template<>
struct has_fixed_size<automode_interfaces::msg::RobotState>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<automode_interfaces::msg::RobotState>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<automode_interfaces::msg::RobotState>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // AUTOMODE_INTERFACES__MSG__DETAIL__ROBOT_STATE__TRAITS_HPP_
