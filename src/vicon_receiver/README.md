# vicon_receiver

This package contains **only ROS 2 message type definitions** for the Vicon motion capture system.

It is **not** a full Vicon driver. The actual Vicon receiver node runs on a separate machine and publishes these message types over the network. This package exists solely so that the `automode_controller` can subscribe to those topics on the real robots without needing the full driver installed.

---

## Message types

### `vicon_receiver/msg/Position`
Pose of a single tracked object:

| Field | Type | Description |
|---|---|---|
| `x_trans`, `y_trans`, `z_trans` | `float32` | Position (translation) |
| `x_rot`, `y_rot`, `z_rot`, `w` | `float32` | Orientation as quaternion |
| `x_rot_euler`, `y_rot_euler`, `z_rot_euler` | `float32` | Orientation as Euler angles |
| `segment_name` | `string` | Specific component of the tracked object |
| `subject_name` | `string` | Name of the entire tracked object |
| `frame_number` | `int32` | Vicon capture frame index |
| `translation_type` | `string` | `"Local"` or `"Global"` |

### `vicon_receiver/msg/PositionList`
List of positions published in one frame:

| Field | Type | Description |
|---|---|---|
| `header` | `std_msgs/Header` | Timestamp and frame id |
| `n` | `int32` | Number of positions in the list |
| `positions` | `Position[]` | Array of `Position` messages |

---

## Usage

This package is **only needed on the real robots**, not in simulation. The Gazebo simulation uses `ground_truth_odom` instead.

### Build

Build together with the rest of the workspace from the repo root:

```bash
cd /home/ben/ros2_ws/automodeROS
colcon build --symlink-install
```

### Source

```bash
source install/setup.bash
```

The message types are then available to any node in the workspace, allowing the controller to deserialise topics published by the Vicon receiver running on the external machine.
