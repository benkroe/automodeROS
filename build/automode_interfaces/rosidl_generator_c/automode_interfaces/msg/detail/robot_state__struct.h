// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from automode_interfaces:msg/RobotState.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "automode_interfaces/msg/robot_state.h"


#ifndef AUTOMODE_INTERFACES__MSG__DETAIL__ROBOT_STATE__STRUCT_H_
#define AUTOMODE_INTERFACES__MSG__DETAIL__ROBOT_STATE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.h"
// Member 'floor_color'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/RobotState in the package automode_interfaces.
/**
  * Robot state information for swarm robotics
 */
typedef struct automode_interfaces__msg__RobotState
{
  /// Unique identifier for the robot in the swarm
  uint32_t robot_id;
  /// Timestamp when the message was generated
  builtin_interfaces__msg__Time stamp;
  /// Detected floor color ("black", "white", "gray", etc.)
  rosidl_runtime_c__String floor_color;
  /// Strength of the detected obstacle vector
  double proximity_magnitude;
  /// Direction of the strongest proximity signal -- ROS2 convention - rad (-right, +left)
  double proximity_angle;
  /// Magnitude of detected ambient light
  double light_magnitude;
  /// Direction of the light source -- ROS2 convention - rad (-right, +left)
  double light_angle;
  /// Magnitude of detected target (normalized area or confidence)
  double target_magnitude;
  /// Horizontal position of the target (-1 left, 0 center, 1 right)
  double target_position;
  /// Direction toward neighbouring robots -- ROS2 convention - rad (-right, +left)
  double attraction_angle;
  /// Number of neighbouring robots detected
  uint32_t neighbour_count;
} automode_interfaces__msg__RobotState;

// Struct for a sequence of automode_interfaces__msg__RobotState.
typedef struct automode_interfaces__msg__RobotState__Sequence
{
  automode_interfaces__msg__RobotState * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} automode_interfaces__msg__RobotState__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // AUTOMODE_INTERFACES__MSG__DETAIL__ROBOT_STATE__STRUCT_H_
