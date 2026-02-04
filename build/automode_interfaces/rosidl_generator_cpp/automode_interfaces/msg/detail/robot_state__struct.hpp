// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from automode_interfaces:msg/RobotState.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "automode_interfaces/msg/robot_state.hpp"


#ifndef AUTOMODE_INTERFACES__MSG__DETAIL__ROBOT_STATE__STRUCT_HPP_
#define AUTOMODE_INTERFACES__MSG__DETAIL__ROBOT_STATE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__automode_interfaces__msg__RobotState __attribute__((deprecated))
#else
# define DEPRECATED__automode_interfaces__msg__RobotState __declspec(deprecated)
#endif

namespace automode_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct RobotState_
{
  using Type = RobotState_<ContainerAllocator>;

  explicit RobotState_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : stamp(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->robot_id = 0ul;
      this->floor_color = "";
      this->proximity_magnitude = 0.0;
      this->proximity_angle = 0.0;
      this->light_magnitude = 0.0;
      this->light_angle = 0.0;
      this->target_magnitude = 0.0;
      this->target_position = 0.0;
      this->attraction_angle = 0.0;
      this->neighbour_count = 0ul;
    }
  }

  explicit RobotState_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : stamp(_alloc, _init),
    floor_color(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->robot_id = 0ul;
      this->floor_color = "";
      this->proximity_magnitude = 0.0;
      this->proximity_angle = 0.0;
      this->light_magnitude = 0.0;
      this->light_angle = 0.0;
      this->target_magnitude = 0.0;
      this->target_position = 0.0;
      this->attraction_angle = 0.0;
      this->neighbour_count = 0ul;
    }
  }

  // field types and members
  using _robot_id_type =
    uint32_t;
  _robot_id_type robot_id;
  using _stamp_type =
    builtin_interfaces::msg::Time_<ContainerAllocator>;
  _stamp_type stamp;
  using _floor_color_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _floor_color_type floor_color;
  using _proximity_magnitude_type =
    double;
  _proximity_magnitude_type proximity_magnitude;
  using _proximity_angle_type =
    double;
  _proximity_angle_type proximity_angle;
  using _light_magnitude_type =
    double;
  _light_magnitude_type light_magnitude;
  using _light_angle_type =
    double;
  _light_angle_type light_angle;
  using _target_magnitude_type =
    double;
  _target_magnitude_type target_magnitude;
  using _target_position_type =
    double;
  _target_position_type target_position;
  using _attraction_angle_type =
    double;
  _attraction_angle_type attraction_angle;
  using _neighbour_count_type =
    uint32_t;
  _neighbour_count_type neighbour_count;

  // setters for named parameter idiom
  Type & set__robot_id(
    const uint32_t & _arg)
  {
    this->robot_id = _arg;
    return *this;
  }
  Type & set__stamp(
    const builtin_interfaces::msg::Time_<ContainerAllocator> & _arg)
  {
    this->stamp = _arg;
    return *this;
  }
  Type & set__floor_color(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->floor_color = _arg;
    return *this;
  }
  Type & set__proximity_magnitude(
    const double & _arg)
  {
    this->proximity_magnitude = _arg;
    return *this;
  }
  Type & set__proximity_angle(
    const double & _arg)
  {
    this->proximity_angle = _arg;
    return *this;
  }
  Type & set__light_magnitude(
    const double & _arg)
  {
    this->light_magnitude = _arg;
    return *this;
  }
  Type & set__light_angle(
    const double & _arg)
  {
    this->light_angle = _arg;
    return *this;
  }
  Type & set__target_magnitude(
    const double & _arg)
  {
    this->target_magnitude = _arg;
    return *this;
  }
  Type & set__target_position(
    const double & _arg)
  {
    this->target_position = _arg;
    return *this;
  }
  Type & set__attraction_angle(
    const double & _arg)
  {
    this->attraction_angle = _arg;
    return *this;
  }
  Type & set__neighbour_count(
    const uint32_t & _arg)
  {
    this->neighbour_count = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    automode_interfaces::msg::RobotState_<ContainerAllocator> *;
  using ConstRawPtr =
    const automode_interfaces::msg::RobotState_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<automode_interfaces::msg::RobotState_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<automode_interfaces::msg::RobotState_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      automode_interfaces::msg::RobotState_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<automode_interfaces::msg::RobotState_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      automode_interfaces::msg::RobotState_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<automode_interfaces::msg::RobotState_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<automode_interfaces::msg::RobotState_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<automode_interfaces::msg::RobotState_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__automode_interfaces__msg__RobotState
    std::shared_ptr<automode_interfaces::msg::RobotState_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__automode_interfaces__msg__RobotState
    std::shared_ptr<automode_interfaces::msg::RobotState_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const RobotState_ & other) const
  {
    if (this->robot_id != other.robot_id) {
      return false;
    }
    if (this->stamp != other.stamp) {
      return false;
    }
    if (this->floor_color != other.floor_color) {
      return false;
    }
    if (this->proximity_magnitude != other.proximity_magnitude) {
      return false;
    }
    if (this->proximity_angle != other.proximity_angle) {
      return false;
    }
    if (this->light_magnitude != other.light_magnitude) {
      return false;
    }
    if (this->light_angle != other.light_angle) {
      return false;
    }
    if (this->target_magnitude != other.target_magnitude) {
      return false;
    }
    if (this->target_position != other.target_position) {
      return false;
    }
    if (this->attraction_angle != other.attraction_angle) {
      return false;
    }
    if (this->neighbour_count != other.neighbour_count) {
      return false;
    }
    return true;
  }
  bool operator!=(const RobotState_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct RobotState_

// alias to use template instance with default allocator
using RobotState =
  automode_interfaces::msg::RobotState_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace automode_interfaces

#endif  // AUTOMODE_INTERFACES__MSG__DETAIL__ROBOT_STATE__STRUCT_HPP_
