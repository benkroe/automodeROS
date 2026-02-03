# AutoMoDe-Tools

This package provides tools for testing, evaluation, and launching AutoMoDe swarm robotics experiments in ROS2.

## Overview

AutoMoDe-Tools contains utilities for:
- Testing individual behaviors and conditions
- Launching multi-robot missions (foraging, aggregation, flocking, dispersion)
- Evaluating swarm performance and collecting data
- Visualizing and scoring mission outcomes

## Test Nodes

### behavior_test_node.py
Tests individual behaviors by sending action goals and monitoring execution.

**Usage:**
```bash
ros2 run automode_tools behavior_test_node
```

**Parameters:**
- `behavior_name`: Name of the behavior to test (e.g., "exploration")
- `params`: List of parameters for the behavior

**Purpose:** Allows isolated testing of behavior modules without running full missions.

### condition_test_node.py
Tests individual conditions by sending action goals and checking condition satisfaction.

**Usage:**
```bash
ros2 run automode_tools condition_test_node
```

**Parameters:**
- `condition_name`: Name of the condition to test (e.g., "neighbor_count")
- `params`: List of parameters for the condition

**Purpose:** Validates condition logic and feedback independently.

### robotState_test_pub.py
Publishes mock RobotState messages for testing.

**Usage:**
```bash
ros2 run automode_tools robotState_test_pub
```

**Purpose:** Simulates robot state data when real sensors are unavailable, useful for development and testing.

## Launch Files

### start_foraging.launch.py
Launches a foraging mission where robots collect items based on floor colors.

**Usage:**
```bash
ros2 launch automode_tools start_foraging.launch.py
```

**Features:**
- Multi-robot setup with navigation stack
- Position collection and scoring
- 10-minute simulation timer

### start_aggregation.launch.py
Launches an aggregation mission where robots cluster together.

**Usage:**
```bash
ros2 launch automode_tools start_aggregation.launch.py
```

**Features:**
- Evaluates group formation
- Position logging for analysis

### start_flocking.launch.py
Launches a flocking mission for coordinated group movement.

**Usage:**
```bash
ros2 launch automode_tools start_flocking.launch.py
```

**Features:**
- Tests attraction and repulsion behaviors
- Collects trajectory data

### start_dispersion.launch.py
Launches a dispersion mission for area coverage.

**Usage:**
```bash
ros2 launch automode_tools start_dispersion.launch.py
```

**Features:**
- Evaluates spatial distribution
- Position heatmap generation

### start_all.launch.py
Launches all mission types sequentially or in parallel.

**Usage:**
```bash
ros2 launch automode_tools start_all.launch.py
```

**Purpose:** Comprehensive testing across all scenarios.

## Other Nodes

### categories_creator.py
Generates and publishes lists of available behaviors and conditions.

**Usage:** Requires `behavior_node` and `condition_node` to be running. Used to create categories for the AutoMoDe visualizer by Kuckling.

**Purpose:** Helps with discovery and validation of loaded modules for visualization tools.

### group_evaluator.py
Analyzes group dynamics from position data.

**Purpose:** Computes metrics like cohesion, separation, and alignment for swarm evaluation.

### position_collector.py
Collects and logs robot positions during missions.

**Usage:** Automatically started in launch files. Outputs CSV files for analysis.

**Output:** `data/positions_log.csv` and similar files.

### scorer_node_aggregation.py
Scores aggregation performance based on cluster formation.

**Metrics:** Cluster size, density, etc.

### scorer_node_foraging.py
Scores foraging performance based on items collected.

**Metrics:** Collection rate, efficiency.

### timer_shutdown.py
Shuts down the simulation after a set time (default: 10 minutes).

**Purpose:** Ensures experiments have consistent duration.

### vicon_terminal_viz_node.py
Provides terminal-based visualization of Vicon-tracked robot positions.

**Usage:** For real-robot experiments with motion capture.

## Usage Examples

### Testing a Behavior
```bash
# Start behavior node
ros2 run automode_tools behavior_test_node --ros-args -p behavior_name:=exploration -p params:=[rwm,50]

# Monitor output
ros2 topic echo /wheels_speed
```

### Running a Mission
```bash
# Launch foraging with 4 robots
ros2 launch automode_tools start_foraging.launch.py

# View scores
ros2 topic echo /foraging_score
```

### Analyzing Results
```bash
# Run position analysis scripts
python3 scripts/plot_positions.py
python3 scripts/heatmap_positions.py
```

## Dependencies

- ROS2 Jazzy
- AutoMoDe core packages (automode_controller, automode_interfaces, etc.)
- PyTrees for behavior trees
- OpenCV for Vicon visualization
- NumPy for data processing

## Configuration

Mission configurations are in `automode_controller/launch/` YAML files. Modify `bt_config` or `fsm_config` for different controllers.
