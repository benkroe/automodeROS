// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from automode_interfaces:action/Condition.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "automode_interfaces/action/condition.h"


#ifndef AUTOMODE_INTERFACES__ACTION__DETAIL__CONDITION__STRUCT_H_
#define AUTOMODE_INTERFACES__ACTION__DETAIL__CONDITION__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'condition_name'
// Member 'params'
#include "rosidl_runtime_c/string.h"

/// Struct defined in action/Condition in the package automode_interfaces.
typedef struct automode_interfaces__action__Condition_Goal
{
  rosidl_runtime_c__String condition_name;
  rosidl_runtime_c__String__Sequence params;
} automode_interfaces__action__Condition_Goal;

// Struct for a sequence of automode_interfaces__action__Condition_Goal.
typedef struct automode_interfaces__action__Condition_Goal__Sequence
{
  automode_interfaces__action__Condition_Goal * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} automode_interfaces__action__Condition_Goal__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'message'
// already included above
// #include "rosidl_runtime_c/string.h"

/// Struct defined in action/Condition in the package automode_interfaces.
typedef struct automode_interfaces__action__Condition_Result
{
  bool success;
  rosidl_runtime_c__String message;
} automode_interfaces__action__Condition_Result;

// Struct for a sequence of automode_interfaces__action__Condition_Result.
typedef struct automode_interfaces__action__Condition_Result__Sequence
{
  automode_interfaces__action__Condition_Result * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} automode_interfaces__action__Condition_Result__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'current_status'
// already included above
// #include "rosidl_runtime_c/string.h"

/// Struct defined in action/Condition in the package automode_interfaces.
typedef struct automode_interfaces__action__Condition_Feedback
{
  bool condition_met;
  rosidl_runtime_c__String current_status;
} automode_interfaces__action__Condition_Feedback;

// Struct for a sequence of automode_interfaces__action__Condition_Feedback.
typedef struct automode_interfaces__action__Condition_Feedback__Sequence
{
  automode_interfaces__action__Condition_Feedback * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} automode_interfaces__action__Condition_Feedback__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'goal'
#include "automode_interfaces/action/detail/condition__struct.h"

/// Struct defined in action/Condition in the package automode_interfaces.
typedef struct automode_interfaces__action__Condition_SendGoal_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
  automode_interfaces__action__Condition_Goal goal;
} automode_interfaces__action__Condition_SendGoal_Request;

// Struct for a sequence of automode_interfaces__action__Condition_SendGoal_Request.
typedef struct automode_interfaces__action__Condition_SendGoal_Request__Sequence
{
  automode_interfaces__action__Condition_SendGoal_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} automode_interfaces__action__Condition_SendGoal_Request__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.h"

/// Struct defined in action/Condition in the package automode_interfaces.
typedef struct automode_interfaces__action__Condition_SendGoal_Response
{
  bool accepted;
  builtin_interfaces__msg__Time stamp;
} automode_interfaces__action__Condition_SendGoal_Response;

// Struct for a sequence of automode_interfaces__action__Condition_SendGoal_Response.
typedef struct automode_interfaces__action__Condition_SendGoal_Response__Sequence
{
  automode_interfaces__action__Condition_SendGoal_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} automode_interfaces__action__Condition_SendGoal_Response__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.h"

// constants for array fields with an upper bound
// request
enum
{
  automode_interfaces__action__Condition_SendGoal_Event__request__MAX_SIZE = 1
};
// response
enum
{
  automode_interfaces__action__Condition_SendGoal_Event__response__MAX_SIZE = 1
};

/// Struct defined in action/Condition in the package automode_interfaces.
typedef struct automode_interfaces__action__Condition_SendGoal_Event
{
  service_msgs__msg__ServiceEventInfo info;
  automode_interfaces__action__Condition_SendGoal_Request__Sequence request;
  automode_interfaces__action__Condition_SendGoal_Response__Sequence response;
} automode_interfaces__action__Condition_SendGoal_Event;

// Struct for a sequence of automode_interfaces__action__Condition_SendGoal_Event.
typedef struct automode_interfaces__action__Condition_SendGoal_Event__Sequence
{
  automode_interfaces__action__Condition_SendGoal_Event * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} automode_interfaces__action__Condition_SendGoal_Event__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"

/// Struct defined in action/Condition in the package automode_interfaces.
typedef struct automode_interfaces__action__Condition_GetResult_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
} automode_interfaces__action__Condition_GetResult_Request;

// Struct for a sequence of automode_interfaces__action__Condition_GetResult_Request.
typedef struct automode_interfaces__action__Condition_GetResult_Request__Sequence
{
  automode_interfaces__action__Condition_GetResult_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} automode_interfaces__action__Condition_GetResult_Request__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'result'
// already included above
// #include "automode_interfaces/action/detail/condition__struct.h"

/// Struct defined in action/Condition in the package automode_interfaces.
typedef struct automode_interfaces__action__Condition_GetResult_Response
{
  int8_t status;
  automode_interfaces__action__Condition_Result result;
} automode_interfaces__action__Condition_GetResult_Response;

// Struct for a sequence of automode_interfaces__action__Condition_GetResult_Response.
typedef struct automode_interfaces__action__Condition_GetResult_Response__Sequence
{
  automode_interfaces__action__Condition_GetResult_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} automode_interfaces__action__Condition_GetResult_Response__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'info'
// already included above
// #include "service_msgs/msg/detail/service_event_info__struct.h"

// constants for array fields with an upper bound
// request
enum
{
  automode_interfaces__action__Condition_GetResult_Event__request__MAX_SIZE = 1
};
// response
enum
{
  automode_interfaces__action__Condition_GetResult_Event__response__MAX_SIZE = 1
};

/// Struct defined in action/Condition in the package automode_interfaces.
typedef struct automode_interfaces__action__Condition_GetResult_Event
{
  service_msgs__msg__ServiceEventInfo info;
  automode_interfaces__action__Condition_GetResult_Request__Sequence request;
  automode_interfaces__action__Condition_GetResult_Response__Sequence response;
} automode_interfaces__action__Condition_GetResult_Event;

// Struct for a sequence of automode_interfaces__action__Condition_GetResult_Event.
typedef struct automode_interfaces__action__Condition_GetResult_Event__Sequence
{
  automode_interfaces__action__Condition_GetResult_Event * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} automode_interfaces__action__Condition_GetResult_Event__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'feedback'
// already included above
// #include "automode_interfaces/action/detail/condition__struct.h"

/// Struct defined in action/Condition in the package automode_interfaces.
typedef struct automode_interfaces__action__Condition_FeedbackMessage
{
  unique_identifier_msgs__msg__UUID goal_id;
  automode_interfaces__action__Condition_Feedback feedback;
} automode_interfaces__action__Condition_FeedbackMessage;

// Struct for a sequence of automode_interfaces__action__Condition_FeedbackMessage.
typedef struct automode_interfaces__action__Condition_FeedbackMessage__Sequence
{
  automode_interfaces__action__Condition_FeedbackMessage * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} automode_interfaces__action__Condition_FeedbackMessage__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // AUTOMODE_INTERFACES__ACTION__DETAIL__CONDITION__STRUCT_H_
