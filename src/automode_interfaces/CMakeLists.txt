cmake_minimum_required(VERSION 3.16)
project(automode_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

# Generate interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/Behavior.action"
  "action/Condition.action"
  "msg/RobotState.msg"
  "msg/Position.msg"
  "msg/PositionList.msg"
  DEPENDENCIES std_msgs action_msgs builtin_interfaces
)

ament_package()