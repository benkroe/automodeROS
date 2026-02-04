// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from automode_interfaces:msg/RobotState.idl
// generated code does not contain a copyright notice
#include "automode_interfaces/msg/detail/robot_state__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `stamp`
#include "builtin_interfaces/msg/detail/time__functions.h"
// Member `floor_color`
#include "rosidl_runtime_c/string_functions.h"

bool
automode_interfaces__msg__RobotState__init(automode_interfaces__msg__RobotState * msg)
{
  if (!msg) {
    return false;
  }
  // robot_id
  // stamp
  if (!builtin_interfaces__msg__Time__init(&msg->stamp)) {
    automode_interfaces__msg__RobotState__fini(msg);
    return false;
  }
  // floor_color
  if (!rosidl_runtime_c__String__init(&msg->floor_color)) {
    automode_interfaces__msg__RobotState__fini(msg);
    return false;
  }
  // proximity_magnitude
  // proximity_angle
  // light_magnitude
  // light_angle
  // target_magnitude
  // target_position
  // attraction_angle
  // neighbour_count
  return true;
}

void
automode_interfaces__msg__RobotState__fini(automode_interfaces__msg__RobotState * msg)
{
  if (!msg) {
    return;
  }
  // robot_id
  // stamp
  builtin_interfaces__msg__Time__fini(&msg->stamp);
  // floor_color
  rosidl_runtime_c__String__fini(&msg->floor_color);
  // proximity_magnitude
  // proximity_angle
  // light_magnitude
  // light_angle
  // target_magnitude
  // target_position
  // attraction_angle
  // neighbour_count
}

bool
automode_interfaces__msg__RobotState__are_equal(const automode_interfaces__msg__RobotState * lhs, const automode_interfaces__msg__RobotState * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // robot_id
  if (lhs->robot_id != rhs->robot_id) {
    return false;
  }
  // stamp
  if (!builtin_interfaces__msg__Time__are_equal(
      &(lhs->stamp), &(rhs->stamp)))
  {
    return false;
  }
  // floor_color
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->floor_color), &(rhs->floor_color)))
  {
    return false;
  }
  // proximity_magnitude
  if (lhs->proximity_magnitude != rhs->proximity_magnitude) {
    return false;
  }
  // proximity_angle
  if (lhs->proximity_angle != rhs->proximity_angle) {
    return false;
  }
  // light_magnitude
  if (lhs->light_magnitude != rhs->light_magnitude) {
    return false;
  }
  // light_angle
  if (lhs->light_angle != rhs->light_angle) {
    return false;
  }
  // target_magnitude
  if (lhs->target_magnitude != rhs->target_magnitude) {
    return false;
  }
  // target_position
  if (lhs->target_position != rhs->target_position) {
    return false;
  }
  // attraction_angle
  if (lhs->attraction_angle != rhs->attraction_angle) {
    return false;
  }
  // neighbour_count
  if (lhs->neighbour_count != rhs->neighbour_count) {
    return false;
  }
  return true;
}

bool
automode_interfaces__msg__RobotState__copy(
  const automode_interfaces__msg__RobotState * input,
  automode_interfaces__msg__RobotState * output)
{
  if (!input || !output) {
    return false;
  }
  // robot_id
  output->robot_id = input->robot_id;
  // stamp
  if (!builtin_interfaces__msg__Time__copy(
      &(input->stamp), &(output->stamp)))
  {
    return false;
  }
  // floor_color
  if (!rosidl_runtime_c__String__copy(
      &(input->floor_color), &(output->floor_color)))
  {
    return false;
  }
  // proximity_magnitude
  output->proximity_magnitude = input->proximity_magnitude;
  // proximity_angle
  output->proximity_angle = input->proximity_angle;
  // light_magnitude
  output->light_magnitude = input->light_magnitude;
  // light_angle
  output->light_angle = input->light_angle;
  // target_magnitude
  output->target_magnitude = input->target_magnitude;
  // target_position
  output->target_position = input->target_position;
  // attraction_angle
  output->attraction_angle = input->attraction_angle;
  // neighbour_count
  output->neighbour_count = input->neighbour_count;
  return true;
}

automode_interfaces__msg__RobotState *
automode_interfaces__msg__RobotState__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  automode_interfaces__msg__RobotState * msg = (automode_interfaces__msg__RobotState *)allocator.allocate(sizeof(automode_interfaces__msg__RobotState), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(automode_interfaces__msg__RobotState));
  bool success = automode_interfaces__msg__RobotState__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
automode_interfaces__msg__RobotState__destroy(automode_interfaces__msg__RobotState * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    automode_interfaces__msg__RobotState__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
automode_interfaces__msg__RobotState__Sequence__init(automode_interfaces__msg__RobotState__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  automode_interfaces__msg__RobotState * data = NULL;

  if (size) {
    data = (automode_interfaces__msg__RobotState *)allocator.zero_allocate(size, sizeof(automode_interfaces__msg__RobotState), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = automode_interfaces__msg__RobotState__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        automode_interfaces__msg__RobotState__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
automode_interfaces__msg__RobotState__Sequence__fini(automode_interfaces__msg__RobotState__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      automode_interfaces__msg__RobotState__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

automode_interfaces__msg__RobotState__Sequence *
automode_interfaces__msg__RobotState__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  automode_interfaces__msg__RobotState__Sequence * array = (automode_interfaces__msg__RobotState__Sequence *)allocator.allocate(sizeof(automode_interfaces__msg__RobotState__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = automode_interfaces__msg__RobotState__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
automode_interfaces__msg__RobotState__Sequence__destroy(automode_interfaces__msg__RobotState__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    automode_interfaces__msg__RobotState__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
automode_interfaces__msg__RobotState__Sequence__are_equal(const automode_interfaces__msg__RobotState__Sequence * lhs, const automode_interfaces__msg__RobotState__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!automode_interfaces__msg__RobotState__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
automode_interfaces__msg__RobotState__Sequence__copy(
  const automode_interfaces__msg__RobotState__Sequence * input,
  automode_interfaces__msg__RobotState__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(automode_interfaces__msg__RobotState);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    automode_interfaces__msg__RobotState * data =
      (automode_interfaces__msg__RobotState *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!automode_interfaces__msg__RobotState__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          automode_interfaces__msg__RobotState__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!automode_interfaces__msg__RobotState__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
