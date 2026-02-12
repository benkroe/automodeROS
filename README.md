# AutoMoDe-ROS2 for TurtleBot4

A **ROS 2 (Jazzy)** implementation of [AutoMoDe](https://github.com/demiurge-project) (Automatic Modular Design) for the **TurtleBot4** swarm robotics platform. Robots are controlled by either a **Finite State Machine (FSM)** or a **Behaviour Tree (BT)**, both defined via compact configuration strings. The architecture is fully modular: behaviours, conditions, simulators, and robot platforms are all swappable without changing the controller.

---

## Packages

| Package | Description |
|---------|-------------|
| [`automode_controller`](src/automode_controller/README.md) | Core controller package. Contains the FSM and BT controllers, behaviour/condition action servers, and reference model nodes for each supported platform. |
| [`automode_interfaces`](src/automode_interfaces/README.md) | ROS 2 message and action definitions shared across all packages (`Behavior.action`, `Condition.action`, `RobotState.msg`). |
| [`automode_tools`](src/automode_tools/README.md) | Mission launch files, scorers, position collector, testing nodes, and the lightweight proof-of-concept simulator. |
| [`automode_tb4_sim`](src/automode_tb4_sim/README.md) | Original Gazebo simulation environment: sensor simulation nodes (ground, light, IR, neighbours) and SDF world files. Predates the current Nav2-based setup. |
| [`basic_modules`](src/basic_modules/) | Pluggable behaviour and condition modules loaded at runtime (exploration, phototaxis, anti-phototaxis, attraction, repulsion, floor colour conditions, neighbour conditions, …). |
| [`vicon_receiver`](src/vicon_receiver/README.md) | Message definitions for the Vicon motion-capture system. Used only with physical robots. |

---

## Simulation setups

Two simulation approaches are supported, each paired with a different reference model:

### 1. Nav2 minimal simulation (used in thesis)

Uses [benkroe/nav2_minimal_turtlebot_simulation](https://github.com/benkroe/nav2_minimal_turtlebot_simulation) as the robot model.  
Reference model: `ref_model_turtlebot4_gz_nav` in `automode_controller`.  
Missions are started via `automode_tools` launch files (e.g. `start_foraging.launch.py`).

### 2. Original sensor-node simulation (`automode_tb4_sim`)

Uses the `automode_tb4_sim` package with standalone sensor nodes.  
Reference model: `ref_model_turtlebot4_gz`.  
See [`automode_tb4_sim`](src/automode_tb4_sim/README.md) for details.

### 3. Lightweight simulator (no Gazebo)

The `simulator` node in `automode_tools` is a self-contained Python physics simulator that requires no Gazebo at all. It demonstrates the architectural property that any component speaking `RobotState` / `wheels_speed` can replace the full simulation stack. See [`automode_tools`](src/automode_tools/README.md) for usage.

---

## Installation

Tested on **Ubuntu 24.04 (arm64 / x86_64)**.

### 1. Install ROS 2 Jazzy

Follow the [official ROS 2 Jazzy installation instructions](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html) and source after installation.

### 2. Install Gazebo Harmonic

```bash
sudo apt install ros-dev-tools
sudo apt-get install curl
sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt-get update
sudo apt-get install gz-harmonic
```

### 3. Create ROS 2 workspace

```bash
mkdir ros2_ws && cd ros2_ws
sudo rosdep init
rosdep update
```

### 4. Install TurtleBot4 simulation dependencies

Choose one option depending on which simulation setup you want to use.

#### Option A — Nav2 minimal model (thesis setup, recommended)

```bash
git clone https://github.com/benkroe/nav2_minimal_turtlebot_simulation.git
```

Follow the setup instructions in that repository.

#### Option B — Full TurtleBot4 simulator with Create3 (original setup)

The full TurtleBot4 Gazebo simulator is built on the iRobot Create3 base. Both need to be cloned and built.

**Create3 dependencies:**

```bash
mkdir create3_ws && cd create3_ws && mkdir src && cd src
git clone https://github.com/paoloelle/create3_sim.git -b jazzy
git clone https://github.com/iRobotEducation/irobot_create_msgs -b jazzy
```

Before building, open `create3.urdf.xacro` and change line 288 to:
```xml
<publish_link_pose> true </publish_link_pose>
```

```bash
cd ..
rosdep install --from-path src -yi --rosdistro jazzy
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/local_setup.bash
cd ..
```

**TurtleBot4 packages:**

```bash
mkdir turtlebot4_ws && cd turtlebot4_ws && mkdir src && cd src
git clone --branch jazzy https://github.com/turtlebot/turtlebot4.git
git clone --branch jazzy https://github.com/turtlebot/turtlebot4_setup.git
git clone --branch jazzy https://github.com/turtlebot/turtlebot4_desktop.git
git clone --branch jazzy https://github.com/turtlebot/turtlebot4_simulator.git
```

Apply the following patches to `sim.launch.py`:

1. Below line 59, add:
   ```python
   get_package_share_directory('automode_tb4_sim')
   ```

2. Below line 68, add:
   ```python
   os.path.join(pkg_automode_tb4_sim, 'worlds'),
   ```

3. For better performance, comment out lines 37–50 in `rplidar.urdf.xacro` if LiDAR is not needed.

```bash
cd ..
rosdep install --from-path src -yi --rosdistro jazzy
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/local_setup.bash
cd ..
```

### 5. Install AutoMoDe

```bash
git clone https://github.com/benkroe/automodeROS.git
cd automodeROS
pip install -r requirements.txt
colcon build --symlink-install
source install/setup.bash
```

### 6. Add sources to `.bashrc`

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/create3_ws/install/setup.bash
source ~/ros2_ws/turtlebot4_ws/install/setup.bash   # or nav2_minimal path
source ~/ros2_ws/automodeROS/install/setup.bash
```

---

## Running on real robots

The entire software stack runs **on each TurtleBot4's on-board computer**. The codebase is cloned and built directly on the robot, and all nodes (controller, behaviour/condition servers, reference model) execute locally. The robot only receives external information from the Vicon motion-capture tracking system.

### Prerequisites

- The full repository is cloned and built on each robot (same steps as the installation above).
- The Vicon system is running and publishing position data. The reference model (`ref_model_turtlebot4_real`) uses Vicon positions to determine the robot's location and — since the real TurtleBot4 has no ground colour sensor — to infer the floor colour from its position in the arena.

### Starting the controller (per robot, via SSH)

SSH into the robot, then launch:

```bash
ros2 launch automode_controller automode_tb4_real.launch.py \
  robot_namespace:=tb1 \
  config_file:=config_foraging.yaml
```

Use `use_sim_time` is automatically set to `false` by this launch file.

### Physical light sensors

The TurtleBot4 real setup uses three VEML7700 ambient light sensors connected over I2C. These are not started by the launch file and must be started manually in a separate SSH session on the robot:

```bash
ros2 run automode_tools light
```

### Notes

- Repeat the above steps for each robot in the swarm (with the appropriate `robot_namespace`).
- The `vicon_receiver` package provides the `PositionList` message type consumed by the reference model. See [`vicon_receiver`](src/vicon_receiver/README.md) for details.
- There is no `timer_shutdown` or scorer running on real robots — experiment timing and evaluation are handled externally.

---

## Running a mission (simulation)

The recommended way to run experiments is via the mission launch files in `automode_tools`. Each launch file starts the full 4-robot stack (simulation, sensors, reference model, controller, scorer, position collector, timer).

```bash
ros2 launch automode_tools start_foraging.launch.py
```

The controller configuration (FSM/BT config string, module package, reference model) is read from the corresponding YAML file in `automode_controller/launch/`. See the [`automode_controller` README](src/automode_controller/README.md) for how to customise or override parameters.

---

## Creating config strings

FSM and BT configuration strings are created using the **AutoMoDe-Editor** by Jonas Kuckling:

> [https://github.com/demiurge-project/AutoMoDe-Editor](https://github.com/demiurge-project/AutoMoDe-Editor)

After adding new modules to `basic_modules`, run `categories_creator` from `automode_tools` to regenerate the editor's category files. See the [`automode_tools` README](src/automode_tools/README.md) for details.

---

## Extending

- Add new behaviour or condition modules in `basic_modules` (or a new module package).
- Run `categories_creator` to update the AutoMoDe-Editor.
- Update the YAML config file with the new config string.
- Use `behavior_test_node` / `condition_test_node` from `automode_tools` to validate new modules in isolation before running a full mission.

---

## License

MIT License. See `LICENSE` for details.
