# AutoMoDe-Interfaces

ROS 2 message and action definitions shared across the AutoMoDe packages.

## Actions

### `Behavior.action`
Used by the controller to start a behaviour on the `behavior_node`.

```
# Goal
string behavior_name
string[] params
---
# Result
bool success
string message
---
# Feedback
(empty)
```

### `Condition.action`
Used by the controller to monitor a condition on the `condition_node`. The condition node sends `condition_met = true` via feedback as soon as the condition triggers.

```
# Goal
string condition_name
string[] params
---
# Result
bool success
string message
---
# Feedback
bool condition_met
string current_status
```

## Messages

### `RobotState.msg`
Published by the reference model node. Contains all sensor readings for one robot.

```
uint32 robot_id               # Unique robot ID in the swarm
Time   stamp                  # Message timestamp

string floor_color            # Detected floor colour: "black", "white", "gray", ...

float64 proximity_magnitude   # Obstacle proximity strength
float64 proximity_angle       # Obstacle direction (rad, ROS2 convention: -right, +left)

float64 light_magnitude       # Ambient light strength
float64 light_angle           # Light source direction (rad)

float64 target_magnitude      # Target detection confidence / normalised area
float64 target_position       # Target horizontal position (-1 left, 0 centre, 1 right)

float64 attraction_angle      # Direction toward neighbouring robots (rad)
uint32  neighbour_count       # Number of neighbours detected
```
