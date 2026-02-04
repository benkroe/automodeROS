// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from automode_interfaces:msg/RobotState.idl
// generated code does not contain a copyright notice

#include "automode_interfaces/msg/detail/robot_state__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_automode_interfaces
const rosidl_type_hash_t *
automode_interfaces__msg__RobotState__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x55, 0x2b, 0x2e, 0x78, 0xe6, 0x17, 0xc8, 0xbc,
      0x7c, 0xc0, 0x4e, 0x06, 0xd9, 0x1b, 0x8d, 0x02,
      0x8e, 0xa3, 0xcb, 0xec, 0xe0, 0xed, 0x77, 0x1c,
      0xd1, 0x90, 0x2f, 0xe8, 0xa3, 0x03, 0xed, 0xf7,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types
#include "builtin_interfaces/msg/detail/time__functions.h"

// Hashes for external referenced types
#ifndef NDEBUG
static const rosidl_type_hash_t builtin_interfaces__msg__Time__EXPECTED_HASH = {1, {
    0xb1, 0x06, 0x23, 0x5e, 0x25, 0xa4, 0xc5, 0xed,
    0x35, 0x09, 0x8a, 0xa0, 0xa6, 0x1a, 0x3e, 0xe9,
    0xc9, 0xb1, 0x8d, 0x19, 0x7f, 0x39, 0x8b, 0x0e,
    0x42, 0x06, 0xce, 0xa9, 0xac, 0xf9, 0xc1, 0x97,
  }};
#endif

static char automode_interfaces__msg__RobotState__TYPE_NAME[] = "automode_interfaces/msg/RobotState";
static char builtin_interfaces__msg__Time__TYPE_NAME[] = "builtin_interfaces/msg/Time";

// Define type names, field names, and default values
static char automode_interfaces__msg__RobotState__FIELD_NAME__robot_id[] = "robot_id";
static char automode_interfaces__msg__RobotState__FIELD_NAME__stamp[] = "stamp";
static char automode_interfaces__msg__RobotState__FIELD_NAME__floor_color[] = "floor_color";
static char automode_interfaces__msg__RobotState__FIELD_NAME__proximity_magnitude[] = "proximity_magnitude";
static char automode_interfaces__msg__RobotState__FIELD_NAME__proximity_angle[] = "proximity_angle";
static char automode_interfaces__msg__RobotState__FIELD_NAME__light_magnitude[] = "light_magnitude";
static char automode_interfaces__msg__RobotState__FIELD_NAME__light_angle[] = "light_angle";
static char automode_interfaces__msg__RobotState__FIELD_NAME__target_magnitude[] = "target_magnitude";
static char automode_interfaces__msg__RobotState__FIELD_NAME__target_position[] = "target_position";
static char automode_interfaces__msg__RobotState__FIELD_NAME__attraction_angle[] = "attraction_angle";
static char automode_interfaces__msg__RobotState__FIELD_NAME__neighbour_count[] = "neighbour_count";

static rosidl_runtime_c__type_description__Field automode_interfaces__msg__RobotState__FIELDS[] = {
  {
    {automode_interfaces__msg__RobotState__FIELD_NAME__robot_id, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {automode_interfaces__msg__RobotState__FIELD_NAME__stamp, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    },
    {NULL, 0, 0},
  },
  {
    {automode_interfaces__msg__RobotState__FIELD_NAME__floor_color, 11, 11},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {automode_interfaces__msg__RobotState__FIELD_NAME__proximity_magnitude, 19, 19},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {automode_interfaces__msg__RobotState__FIELD_NAME__proximity_angle, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {automode_interfaces__msg__RobotState__FIELD_NAME__light_magnitude, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {automode_interfaces__msg__RobotState__FIELD_NAME__light_angle, 11, 11},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {automode_interfaces__msg__RobotState__FIELD_NAME__target_magnitude, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {automode_interfaces__msg__RobotState__FIELD_NAME__target_position, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {automode_interfaces__msg__RobotState__FIELD_NAME__attraction_angle, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {automode_interfaces__msg__RobotState__FIELD_NAME__neighbour_count, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription automode_interfaces__msg__RobotState__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
automode_interfaces__msg__RobotState__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {automode_interfaces__msg__RobotState__TYPE_NAME, 34, 34},
      {automode_interfaces__msg__RobotState__FIELDS, 11, 11},
    },
    {automode_interfaces__msg__RobotState__REFERENCED_TYPE_DESCRIPTIONS, 1, 1},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# Robot state information for swarm robotics\n"
  "uint32 robot_id                    # Unique identifier for the robot in the swarm\n"
  "builtin_interfaces/Time stamp      # Timestamp when the message was generated\n"
  "string floor_color            # Detected floor color (\"black\", \"white\", \"gray\", etc.)\n"
  "float64 proximity_magnitude        # Strength of the detected obstacle vector\n"
  "float64 proximity_angle           # Direction of the strongest proximity signal -- ROS2 convention - rad (-right, +left)\n"
  "float64 light_magnitude           # Magnitude of detected ambient light\n"
  "float64 light_angle              # Direction of the light source -- ROS2 convention - rad (-right, +left)\n"
  "float64 target_magnitude         # Magnitude of detected target (normalized area or confidence)\n"
  "float64 target_position          # Horizontal position of the target (-1 left, 0 center, 1 right)\n"
  "float64 attraction_angle         # Direction toward neighbouring robots -- ROS2 convention - rad (-right, +left)\n"
  "uint32 neighbour_count            # Number of neighbouring robots detected";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
automode_interfaces__msg__RobotState__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {automode_interfaces__msg__RobotState__TYPE_NAME, 34, 34},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 1049, 1049},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
automode_interfaces__msg__RobotState__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[2];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 2, 2};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *automode_interfaces__msg__RobotState__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
