# AutoMoDe-Controller

This package is the brain of the AutoMoDe swarm robotics framework. It parses a compact configuration string into either a **Finite State Machine (FSM)** or a **Behaviour Tree (BT)** and drives a single TurtleBot4 robot by sending action goals to the behaviour and condition nodes.

## Overview

This package contains **all nodes needed to run a single robot**: the controller (FSM or BT), the behaviour action server, the condition action server, and the reference model node. Each robot in a swarm runs its own instance of the full stack.

At startup:

1. The behaviour and condition servers scan the configured `module_package` (e.g. `basic_modules`) to discover available modules, and advertise them via `behaviors/list_srv` and `conditions/list_srv`.
2. The controller reads a configuration string (from a YAML file or launch argument) and queries those services to build the FSM or BT.
3. The controller starts running the machine / ticking the tree, sending `Behavior` and `Condition` action goals to the servers as required.

## Nodes

### Controllers

Choose one controller per robot. The controller type is set by the `controller_type` parameter (`fsm` or `bt`).

| Executable | Description |
|------------|-------------|
| `controller_node` | FSM controller. Reads `fsm_config`, runs states and monitors outgoing conditions. Transitions when a condition is met. |
| `controller_bt_node` | BT controller. Reads `bt_config` (or `bt_config_file`), builds a `py_trees` tree, and ticks it on a timer. |

**Key parameters**

| Parameter | Default | Description |
|-----------|---------|-------------|
| `fsm_config` | `simple_fsm` | FSM configuration string |
| `bt_config` | `''` | BT configuration string |
| `bt_config_file` | `''` | Path to a file containing the BT config string (alternative to `bt_config`) |
| `use_sim_time` | `true` | Use simulation clock |

---

### Behaviour and condition servers

These two nodes are always launched alongside the controller. They serve as the execution layer for the behaviours and conditions defined in the `module_package`.

| Executable | Action server | Description |
|------------|---------------|-------------|
| `behavior_node` | `behavior_action` | Loads and runs behaviour modules. Only one behaviour is active at a time; a new goal replaces the current one. |
| `condition_node` | `condition_action` | Loads and monitors condition modules. Multiple conditions can run concurrently (one per FSM edge). |

At startup, both servers scan the `module_package` for available modules and advertise the list via `behaviors/list_srv` and `conditions/list_srv` (`std_srvs/Trigger`). The controller queries these services to resolve the numeric type IDs in the config string to actual module names.

**Key parameter**

| Parameter | Default | Description |
|-----------|---------|-------------|
| `module_package` | `basic_modules` | ROS 2 package to scan for behaviour/condition modules |

---

### Reference model nodes

The reference model is an abstraction layer between the controller software and the concrete robot or simulator being used. It subscribes to the commands issued by behaviours (such as wheel velocity setpoints) and translates them into the topic and service calls appropriate for the target platform. It also publishes the sensor abstractions (proximity, floor colour, light, neighbours) that condition modules read. Swapping the reference model is all that is needed to run the same controller on a different platform.

| Executable | Target |
|------------|--------|
| `ref_model_turtlebot4_gz` | Works with `automode_tb4_sim` (original simulation package) |
| `ref_model_turtlebot4_gz_nav` | Works with [benkroe/nav2_minimal_turtlebot_simulation](https://github.com/benkroe/nav2_minimal_turtlebot_simulation) (used in thesis) |
| `ref_model_turtlebot4_wb` | Webots simulator |
| `ref_model_turtlebot4_real` | Physical TurtleBot4 |

### Real-robot reference model — required parameter

`ref_model_turtlebot4_real` uses Vicon for position tracking. It must know which Vicon subject corresponds to the robot it is running on. **This must be set per robot** via the `subject_name` ROS parameter:

```bash
ros2 run automode_controller ref_model_turtlebot4_real \
  --ros-args -p subject_name:=turtlebot4_11
```

Or inside a launch file:

```python
Node(
    package='automode_controller',
    executable='ref_model_turtlebot4_real',
    parameters=[{'subject_name': 'turtlebot4_11'}],
)
```

The default value is `turtlebot4_11`. Running without setting this parameter on a different robot will cause the node to track the wrong Vicon subject — it will silently receive no pose updates.

---

## Launch files

| File | Purpose |
|------|---------|
| `automode_tb4.launch.py` | Standard launch for the [benkroe/nav2_minimal_turtlebot_simulation](https://github.com/benkroe/nav2_minimal_turtlebot_simulation) environment (main entry point for thesis experiments) |
| `automode_tb4_real.launch.py` | Physical TurtleBot4 launch (no sim time) |
| `automode_simulator.launch.py` | Lightweight controller-only launch for use with the built-in `simulator.py` |

### `automode_simulator.launch.py`

This launch file starts only the AutoMoDe controller stack — `behavior_node`, `condition_node`, and the selected controller (`controller_node` or `controller_bt_node`). It does **not** start Gazebo or a reference model node.

It is designed to be used together with the standalone `simulator` node from the `automode_tools` package, which provides a lightweight physics simulation of the robots (differential drive, wall collisions, all sensors). The simulator publishes sensor data and consumes velocity commands directly, bypassing the need for Gazebo or a reference model entirely. This makes it useful for fast controller development and testing.

**Usage:**

Terminal 1 — start the simulator:
```bash
cd src/automode_tools/automode_tools
python3 simulator.py
```

Terminal 2 — start the controller:
```bash
ros2 launch automode_controller automode_simulator.launch.py \
  robot_namespace:=tb1 \
  config_file:=config_foraging.yaml
```

---

All launch files accept the following arguments, which override the values in the YAML config file:

| Argument | Description |
|----------|-------------|
| `config_file` | YAML config file name (relative to `launch/`) or absolute path |
| `robot_namespace` | Namespace for the robot (e.g. `tb1`) |
| `controller_type` | `fsm` or `bt` |
| `fsm_config` | FSM config string (overrides YAML) |
| `bt_config` | BT config string (overrides YAML) |
| `module_package` | Behaviour/condition module package |
| `ref_model` | Reference model executable name |

**Example:**
```bash
ros2 launch automode_controller automode_tb4.launch.py \
  robot_namespace:=tb1 \
  config_file:=config_foraging.yaml
```

---

## Config YAML files

Stored in `launch/`. Each file sets default parameters for one mission type.

| File | Mission |
|------|---------|
| `config_foraging.yaml` | Foraging (collect objects, return to nest) |
| `config_aggregation.yaml` | Aggregation (gather in one zone) |
| `config_flocking.yaml` | Flocking (move cohesively as a swarm) |
| `config_recruitment.yaml` | Recruitment (guide others to a target) |
| `config_dispersion.yaml` | Dispersion (spread out in the arena) |

**Minimal config file structure:**
```yaml
robot_namespace: ""
module_package: "basic_modules"
controller_type: "fsm"          # or "bt"
ref_model: "ref_model_turtlebot4_gz_nav"
fsm_config: "--fsm-config --nstates 1 --s0 0 --rwm0 50"
bt_config: ""
```

---

## Config string format

Config strings (FSM and BT) should be created using the **AutoMoDe-Editor** by Jonas Kuckling, a graphical tool that lets you design FSMs and BTs and export the corresponding config string:

> [https://github.com/demiurge-project/AutoMoDe-Editor](https://github.com/demiurge-project/AutoMoDe-Editor)

If you add new behaviours or conditions to `basic_modules`, use the tools in the **`automode_tools`** package to update the editor with the new modules so that it stays consistent with your setup.

---

## File reference

| File | Role |
|------|------|
| `controller_node.py` | FSM controller node |
| `controller_bt_node.py` | BT controller node |
| `behavior_node.py` | Behaviour action server |
| `condition_node.py` | Condition action server |
| `fsm.py` | `FSM` / `FSMState` / `FSMEdge` dataclasses + `create_fsm_from_config()` parser |
| `bt_parser.py` | `parse_bt_config()` -- tokenises and validates BT config strings |
| `bt_leaves.py` | `BehaviorLeaf` and `ConditionLeaf` py_trees wrappers |
| `ref_model_turtlebot4_*.py` | Platform-specific reference model nodes |
