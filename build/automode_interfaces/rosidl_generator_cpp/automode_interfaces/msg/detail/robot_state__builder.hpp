// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from automode_interfaces:msg/RobotState.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "automode_interfaces/msg/robot_state.hpp"


#ifndef AUTOMODE_INTERFACES__MSG__DETAIL__ROBOT_STATE__BUILDER_HPP_
#define AUTOMODE_INTERFACES__MSG__DETAIL__ROBOT_STATE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "automode_interfaces/msg/detail/robot_state__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace automode_interfaces
{

namespace msg
{

namespace builder
{

class Init_RobotState_neighbour_count
{
public:
  explicit Init_RobotState_neighbour_count(::automode_interfaces::msg::RobotState & msg)
  : msg_(msg)
  {}
  ::automode_interfaces::msg::RobotState neighbour_count(::automode_interfaces::msg::RobotState::_neighbour_count_type arg)
  {
    msg_.neighbour_count = std::move(arg);
    return std::move(msg_);
  }

private:
  ::automode_interfaces::msg::RobotState msg_;
};

class Init_RobotState_attraction_angle
{
public:
  explicit Init_RobotState_attraction_angle(::automode_interfaces::msg::RobotState & msg)
  : msg_(msg)
  {}
  Init_RobotState_neighbour_count attraction_angle(::automode_interfaces::msg::RobotState::_attraction_angle_type arg)
  {
    msg_.attraction_angle = std::move(arg);
    return Init_RobotState_neighbour_count(msg_);
  }

private:
  ::automode_interfaces::msg::RobotState msg_;
};

class Init_RobotState_target_position
{
public:
  explicit Init_RobotState_target_position(::automode_interfaces::msg::RobotState & msg)
  : msg_(msg)
  {}
  Init_RobotState_attraction_angle target_position(::automode_interfaces::msg::RobotState::_target_position_type arg)
  {
    msg_.target_position = std::move(arg);
    return Init_RobotState_attraction_angle(msg_);
  }

private:
  ::automode_interfaces::msg::RobotState msg_;
};

class Init_RobotState_target_magnitude
{
public:
  explicit Init_RobotState_target_magnitude(::automode_interfaces::msg::RobotState & msg)
  : msg_(msg)
  {}
  Init_RobotState_target_position target_magnitude(::automode_interfaces::msg::RobotState::_target_magnitude_type arg)
  {
    msg_.target_magnitude = std::move(arg);
    return Init_RobotState_target_position(msg_);
  }

private:
  ::automode_interfaces::msg::RobotState msg_;
};

class Init_RobotState_light_angle
{
public:
  explicit Init_RobotState_light_angle(::automode_interfaces::msg::RobotState & msg)
  : msg_(msg)
  {}
  Init_RobotState_target_magnitude light_angle(::automode_interfaces::msg::RobotState::_light_angle_type arg)
  {
    msg_.light_angle = std::move(arg);
    return Init_RobotState_target_magnitude(msg_);
  }

private:
  ::automode_interfaces::msg::RobotState msg_;
};

class Init_RobotState_light_magnitude
{
public:
  explicit Init_RobotState_light_magnitude(::automode_interfaces::msg::RobotState & msg)
  : msg_(msg)
  {}
  Init_RobotState_light_angle light_magnitude(::automode_interfaces::msg::RobotState::_light_magnitude_type arg)
  {
    msg_.light_magnitude = std::move(arg);
    return Init_RobotState_light_angle(msg_);
  }

private:
  ::automode_interfaces::msg::RobotState msg_;
};

class Init_RobotState_proximity_angle
{
public:
  explicit Init_RobotState_proximity_angle(::automode_interfaces::msg::RobotState & msg)
  : msg_(msg)
  {}
  Init_RobotState_light_magnitude proximity_angle(::automode_interfaces::msg::RobotState::_proximity_angle_type arg)
  {
    msg_.proximity_angle = std::move(arg);
    return Init_RobotState_light_magnitude(msg_);
  }

private:
  ::automode_interfaces::msg::RobotState msg_;
};

class Init_RobotState_proximity_magnitude
{
public:
  explicit Init_RobotState_proximity_magnitude(::automode_interfaces::msg::RobotState & msg)
  : msg_(msg)
  {}
  Init_RobotState_proximity_angle proximity_magnitude(::automode_interfaces::msg::RobotState::_proximity_magnitude_type arg)
  {
    msg_.proximity_magnitude = std::move(arg);
    return Init_RobotState_proximity_angle(msg_);
  }

private:
  ::automode_interfaces::msg::RobotState msg_;
};

class Init_RobotState_floor_color
{
public:
  explicit Init_RobotState_floor_color(::automode_interfaces::msg::RobotState & msg)
  : msg_(msg)
  {}
  Init_RobotState_proximity_magnitude floor_color(::automode_interfaces::msg::RobotState::_floor_color_type arg)
  {
    msg_.floor_color = std::move(arg);
    return Init_RobotState_proximity_magnitude(msg_);
  }

private:
  ::automode_interfaces::msg::RobotState msg_;
};

class Init_RobotState_stamp
{
public:
  explicit Init_RobotState_stamp(::automode_interfaces::msg::RobotState & msg)
  : msg_(msg)
  {}
  Init_RobotState_floor_color stamp(::automode_interfaces::msg::RobotState::_stamp_type arg)
  {
    msg_.stamp = std::move(arg);
    return Init_RobotState_floor_color(msg_);
  }

private:
  ::automode_interfaces::msg::RobotState msg_;
};

class Init_RobotState_robot_id
{
public:
  Init_RobotState_robot_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_RobotState_stamp robot_id(::automode_interfaces::msg::RobotState::_robot_id_type arg)
  {
    msg_.robot_id = std::move(arg);
    return Init_RobotState_stamp(msg_);
  }

private:
  ::automode_interfaces::msg::RobotState msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::automode_interfaces::msg::RobotState>()
{
  return automode_interfaces::msg::builder::Init_RobotState_robot_id();
}

}  // namespace automode_interfaces

#endif  // AUTOMODE_INTERFACES__MSG__DETAIL__ROBOT_STATE__BUILDER_HPP_
