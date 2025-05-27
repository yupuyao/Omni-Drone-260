// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from px4_msgs:msg/RoverVelocityStatus.idl
// generated code does not contain a copyright notice

#ifndef PX4_MSGS__MSG__DETAIL__ROVER_VELOCITY_STATUS__TRAITS_HPP_
#define PX4_MSGS__MSG__DETAIL__ROVER_VELOCITY_STATUS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "px4_msgs/msg/detail/rover_velocity_status__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace px4_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const RoverVelocityStatus & msg,
  std::ostream & out)
{
  out << "{";
  // member: timestamp
  {
    out << "timestamp: ";
    rosidl_generator_traits::value_to_yaml(msg.timestamp, out);
    out << ", ";
  }

  // member: measured_speed_body_x
  {
    out << "measured_speed_body_x: ";
    rosidl_generator_traits::value_to_yaml(msg.measured_speed_body_x, out);
    out << ", ";
  }

  // member: speed_body_x_setpoint
  {
    out << "speed_body_x_setpoint: ";
    rosidl_generator_traits::value_to_yaml(msg.speed_body_x_setpoint, out);
    out << ", ";
  }

  // member: adjusted_speed_body_x_setpoint
  {
    out << "adjusted_speed_body_x_setpoint: ";
    rosidl_generator_traits::value_to_yaml(msg.adjusted_speed_body_x_setpoint, out);
    out << ", ";
  }

  // member: measured_speed_body_y
  {
    out << "measured_speed_body_y: ";
    rosidl_generator_traits::value_to_yaml(msg.measured_speed_body_y, out);
    out << ", ";
  }

  // member: speed_body_y_setpoint
  {
    out << "speed_body_y_setpoint: ";
    rosidl_generator_traits::value_to_yaml(msg.speed_body_y_setpoint, out);
    out << ", ";
  }

  // member: adjusted_speed_body_y_setpoint
  {
    out << "adjusted_speed_body_y_setpoint: ";
    rosidl_generator_traits::value_to_yaml(msg.adjusted_speed_body_y_setpoint, out);
    out << ", ";
  }

  // member: pid_throttle_body_x_integral
  {
    out << "pid_throttle_body_x_integral: ";
    rosidl_generator_traits::value_to_yaml(msg.pid_throttle_body_x_integral, out);
    out << ", ";
  }

  // member: pid_throttle_body_y_integral
  {
    out << "pid_throttle_body_y_integral: ";
    rosidl_generator_traits::value_to_yaml(msg.pid_throttle_body_y_integral, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const RoverVelocityStatus & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: timestamp
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "timestamp: ";
    rosidl_generator_traits::value_to_yaml(msg.timestamp, out);
    out << "\n";
  }

  // member: measured_speed_body_x
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "measured_speed_body_x: ";
    rosidl_generator_traits::value_to_yaml(msg.measured_speed_body_x, out);
    out << "\n";
  }

  // member: speed_body_x_setpoint
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "speed_body_x_setpoint: ";
    rosidl_generator_traits::value_to_yaml(msg.speed_body_x_setpoint, out);
    out << "\n";
  }

  // member: adjusted_speed_body_x_setpoint
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "adjusted_speed_body_x_setpoint: ";
    rosidl_generator_traits::value_to_yaml(msg.adjusted_speed_body_x_setpoint, out);
    out << "\n";
  }

  // member: measured_speed_body_y
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "measured_speed_body_y: ";
    rosidl_generator_traits::value_to_yaml(msg.measured_speed_body_y, out);
    out << "\n";
  }

  // member: speed_body_y_setpoint
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "speed_body_y_setpoint: ";
    rosidl_generator_traits::value_to_yaml(msg.speed_body_y_setpoint, out);
    out << "\n";
  }

  // member: adjusted_speed_body_y_setpoint
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "adjusted_speed_body_y_setpoint: ";
    rosidl_generator_traits::value_to_yaml(msg.adjusted_speed_body_y_setpoint, out);
    out << "\n";
  }

  // member: pid_throttle_body_x_integral
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pid_throttle_body_x_integral: ";
    rosidl_generator_traits::value_to_yaml(msg.pid_throttle_body_x_integral, out);
    out << "\n";
  }

  // member: pid_throttle_body_y_integral
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pid_throttle_body_y_integral: ";
    rosidl_generator_traits::value_to_yaml(msg.pid_throttle_body_y_integral, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const RoverVelocityStatus & msg, bool use_flow_style = false)
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

}  // namespace px4_msgs

namespace rosidl_generator_traits
{

[[deprecated("use px4_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const px4_msgs::msg::RoverVelocityStatus & msg,
  std::ostream & out, size_t indentation = 0)
{
  px4_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use px4_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const px4_msgs::msg::RoverVelocityStatus & msg)
{
  return px4_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<px4_msgs::msg::RoverVelocityStatus>()
{
  return "px4_msgs::msg::RoverVelocityStatus";
}

template<>
inline const char * name<px4_msgs::msg::RoverVelocityStatus>()
{
  return "px4_msgs/msg/RoverVelocityStatus";
}

template<>
struct has_fixed_size<px4_msgs::msg::RoverVelocityStatus>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<px4_msgs::msg::RoverVelocityStatus>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<px4_msgs::msg::RoverVelocityStatus>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // PX4_MSGS__MSG__DETAIL__ROVER_VELOCITY_STATUS__TRAITS_HPP_
