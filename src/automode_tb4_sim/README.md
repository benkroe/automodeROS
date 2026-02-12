# AutoMoDe-TB4-Sim

This package was the original Gazebo simulation environment for TurtleBot4 swarm experiments. It provides standalone sensor simulation nodes that bridge raw Gazebo/TF data to the sensor abstractions consumed by the AutoMoDe controller, along with SDF world files and launch files to spawn robots.

> **Note:** Current experiments (thesis) use `ref_model_turtlebot4_gz_nav` together with a separate minimal simulation model: [benkroe/nav2_minimal_turtlebot_simulation](https://github.com/benkroe/nav2_minimal_turtlebot_simulation). This package predates that approach and is kept for reference.

## Sensor nodes

All sensor nodes use TF2 to determine robot positions and publish at the topic names that the reference model (`automode_controller`) expects. They are always launched inside the robot's namespace (e.g. `tb1/`).

| Executable | Topic | Description |
|------------|-------|-------------|
| `ground_sensor_node` | `ground_sensor_center` | Reads the robot's TF position and maps it to a floor colour (`"black"`, `"white"`, `"gray"`) based on the patch layout of the loaded world. |
| `light_sensors_node` | `light_sensor_front_left`, `light_sensor_front_right`, `light_sensor_back` | Emulates 3 light sensors. Computes distance from each sensor TF frame to the hardcoded light source position and publishes intensity accordingly. |
| `ir_sensors_node` | IR sensor topics | Reads proximity data from Gazebo and republishes it in the AutoMoDe format. |
| `neighbour_sensor_node` | `neighbours_info` | Inspects TF poses of all known robots (`tb1`–`tb4`) and publishes neighbour count and attraction angle for those within the detection radius (0.7 m). |

> **Note:** The world-specific constants (patch coordinates, light source position, base TF frame) are hardcoded at the top of each sensor node file. When switching between `mission.sdf` and `white.sdf`, update these constants accordingly.

## Worlds

| File | Description |
|------|-------------|
| `worlds/mission.sdf` | Main foraging arena: rectangular white nest zone (6 × 2 m), two circular black patches, light source at (0, 3.5). |
| `worlds/white.sdf` | Simple white arena with a single black patch, used for basic testing. |

## Launch files

### `turtlebot4_simulation.launch.py`
Spawns a single TurtleBot4 in Gazebo together with its sensor nodes.

```bash
ros2 launch automode_tb4_sim turtlebot4_simulation.launch.py \
  turtlebot4_id:=tb1 \
  world:=mission
```

| Argument | Default | Description |
|----------|---------|-------------|
| `turtlebot4_id` | `tb1` | Robot namespace (must match the TF frames) |
| `world` | `mission` | SDF world name (`mission` or `white`) |

### `add_turtlebot4_simulation.launch.py`
Spawns an additional robot into an already-running Gazebo instance (without restarting the world).

```bash
ros2 launch automode_tb4_sim add_turtlebot4_simulation.launch.py \
  turtlebot4_id:=tb2 \
  x:=1.0 \
  y:=1.0 \
  world:=mission
```

| Argument | Default | Description |
|----------|---------|-------------|
| `turtlebot4_id` | `tb2` | Namespace for the new robot |
| `x` / `y` | `1.0` | Spawn position in the world |
| `world` | `mission` | Must match the already-loaded world |

> In practice, the mission-level launch files in `automode_tools` (e.g. `start_foraging.launch.py`) call these internally — you rarely need to invoke them directly.
