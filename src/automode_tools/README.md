# AutoMoDe-Tools

This package provides the mission launch files, scoring, data collection, and testing utilities for AutoMoDe swarm robotics experiments.

## Launch files

Each mission has a dedicated launch file that starts the full experiment stack for 4 robots: simulation, sensors, reference model, automode controller, scorer/evaluator, position collector, and a timer that shuts everything down after 10 simulated minutes.

| File | Mission |
|------|---------|
| `start_foraging.launch.py` | Foraging |
| `start_aggregation.launch.py` | Aggregation |
| `start_flocking.launch.py` | Flocking |
| `start_recruitment.launch.py` | Recruitment |
| `start_dispersion.launch.py` | Dispersion |
| `start_all.launch.py` | Starts all missions at once |

**Example:**
```bash
ros2 launch automode_tools start_foraging.launch.py
```

The controller configuration (FSM or BT config string, module package, ref model) is read from the corresponding YAML file in `automode_controller/launch/`.

---

## Nodes

### Mission infrastructure

| Executable | Description |
|------------|-------------|
| `timer_shutdown` | Monitors `/clock` and shuts down the entire launch after 10 simulated minutes (600 s). Logs real elapsed time and RTF on exit. |
| `position_collector` | Subscribes to `/{robot}/ground_truth_odom` for all 4 robots and writes positions to `data/positions_log.csv` at 10 Hz. Used for post-experiment trajectory and cohesion analysis. |

### Scorers

| Executable | Description |
|------------|-------------|
| `scorer_node_foraging` | Tracks each robot's floor colour. Increments a score when a robot transitions black→white (picked up and deposited a marker). Logs scores every second. |
| `scorer_node_aggregation` | Evaluates aggregation performance based on robot states. |
| `group_evaluator` | Subscribes to `/tb1/robotState` and monitors neighbour count to track whether the swarm is in group formation. Publishes a running count on `group_count`. |

### Testing and development

| Executable | Description |
|------------|-------------|
| `behavior_test_node` | Action server wrapper for manually testing a single behaviour without running a full experiment. |
| `condition_test_node` | Action server wrapper for manually testing a single condition. |
| `robotState_test_pub` | Publishes a fake `RobotState` message for testing controllers without a running simulation. |
| `simulator` | Lightweight standalone physics simulator. See below. |

#### `simulator` — proof-of-concept lightweight physics simulator (no Gazebo)

The AutoMoDe architecture is explicitly designed so that the **reference model is the only platform-dependent component**. Behaviours and conditions communicate exclusively through the `RobotState` message and the `wheels_speed` topic — they have no knowledge of what lies beneath. This means any simulator that publishes `RobotState` and subscribes to `wheels_speed` can fully replace Gazebo, without touching a single line of controller code.

The `simulator` node in this package is a **proof-of-concept** implementation of that idea. It is a self-contained Python process that simulates 4 robots in a 4 × 4 m arena:

- Differential drive kinematics
- Wall collision detection
- Sensor simulation: proximity (IR-style, front-facing), floor colour (patch-based), light (distance to a point source), neighbour detection
- Publishes `RobotState` for each robot and a `/clock` topic for simulation time
- Visualises the arena and robots via RViz markers

Because there is no physics engine overhead, the simulator runs far faster than real time — making it well suited for rapid controller iteration or preliminary parameter search before committing to full Gazebo runs.

To use it, start the simulator first, then launch the controller stack with `automode_simulator.launch.py` (see `automode_controller` README):

```bash
# Terminal 1 — physics + sensors
cd src/automode_tools/automode_tools
python3 simulator.py

# Terminal 2 — controller
ros2 launch automode_controller automode_simulator.launch.py \
  robot_namespace:=tb1 \
  config_file:=config_foraging.yaml
```

To visualise what is happening, open RViz2 in a third terminal:

```bash
rviz2
```

In RViz2:
- Set **Fixed Frame** to `world`
- Add a **MarkerArray** display and set its topic to `/visualization_marker_array`

The simulator publishes markers for the arena boundary, the robots, the light source, and the floor colour patches. Robot positions update in real time as the controller drives them.

### Editor integration

| Executable | Description |
|------------|-------------|
| `categories_creator` | Queries the `behaviors/list_srv` and `conditions/list_srv` services and generates `nodeCategories.json` and `edgeCategories.json` files for the [AutoMoDe-Editor](https://github.com/demiurge-project/AutoMoDe-Editor). Run this after adding new modules to `basic_modules` to keep the editor in sync. **The `behavior_node` and `condition_node` must already be running with the updated module package loaded** before executing this tool, as it relies on their list services to discover the available modules. |

### Real-robot utilities

| Executable | Description |
|------------|-------------|
| `light` | Reads three physical VEML7700 ambient light sensors over I2C and publishes their values (real TurtleBot4 only, not used in simulation). |
| `vicon_terminal_viz_node` | Subscribes to `/vicon/default/data` and prints Vicon position data in the terminal. Used for quick sanity checks when running experiments with the real robots and the Vicon tracking system. |

---

## Utility modules

| File | Description |
|------|-------------|
| `positon.py` | Example snippet showing how to subscribe to Vicon `PositionList` messages and extract a specific robot's position by name. |
