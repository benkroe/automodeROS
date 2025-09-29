# AutoMoDe-ROS2 for TurtleBot4

This repository provides a ROS 2 (Jazzy) implementation of **AutoMoDe** (Automatic Modular Design) for the **TurtleBot4** in **Gazebo Harmonic**.  
It enables modular behavior and condition based robot control with finite state machines.  

## Features
- ROS 2 Jazzy based implementation in Python
- Five main packages:
  - `automode_controller` – main controller and finite state machine
  - `automode_interface` – messages and actions
  - `automode_tools` – testing utilities
  - `automode_tb4_sim` – simulation setup with additional sensors
  - `basic_modules` – example behaviors and conditions
- Easily extensible for new behaviors, conditions, and robot platforms
- Works in Gazebo Harmonic with TurtleBot4 simulation

## Installation

Follow these steps on **Ubuntu 24.04 (arm64/x86_64)**:

### 1. Install ROS 2 Jazzy
Follow the [official ROS 2 Jazzy installation instructions](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html) and make sure to source after installation.

### 2. Install Development Tools and Gazebo Harmonic

```bash
sudo apt install ros-dev-tools
sudo apt-get install curl
sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt-get update
sudo apt-get install gz-harmonic
```

### 3. Create ROS 2 Workspace

```bash
mkdir ros2_ws && cd ros2_ws
sudo rosdep init
rosdep update
```

### 4. Install Create3 Dependencies

```bash
mkdir create3_ws && cd create3_ws
mkdir src && cd src
git clone https://github.com/paoloelle/create3_sim.git -b jazzy
git clone https://github.com/iRobotEducation/irobot_create_msgs -b jazzy
```

Before building, find the file `create3.urdf.xacro` and change line 288 to:
```xml
<publish_link_pose> true </publish_link_pose>
```

Then build:
```bash
cd ..
rosdep install --from-path src -yi --rosdistro jazzy
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/local_setup.bash
cd ..
```

### 5. Install TurtleBot4 Dependencies

```bash
mkdir turtlebot4_ws && cd turtlebot4_ws
mkdir src && cd src
git clone --branch jazzy https://github.com/turtlebot/turtlebot4.git
git clone --branch jazzy https://github.com/turtlebot/turtlebot4_setup.git
git clone --branch jazzy https://github.com/turtlebot/turtlebot4_desktop.git
git clone --branch jazzy https://github.com/turtlebot/turtlebot4_simulator.git
```

Make the following changes:

1. In `sim.launch.py` add below line 59:
   ```python
   get_package_share_directory('automode_tb4_sim')
   ```

2. Also in `sim.launch.py` add below line 68:
   ```python
   os.path.join(pkg_automode_tb4_sim, 'worlds'),
   ```

3. For better performance, comment out or delete lines 37-50 in `rplidar.urdf.xacro` if LiDAR is not used.

Then build:
```bash
cd ..
rosdep install --from-path src -yi --rosdistro jazzy
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/local_setup.bash
cd ..
```

### 6. Install AutoMoDe

```bash
git clone https://github.com/benkroe/automodeROS.git
cd automodeROS
colcon build --symlink-install
source install/setup.bash
```

### 7. Optional: Add Sources to .bashrc

Add the following lines to the end of your `~/.bashrc` file:

```bash
# Source ROS 2 and workspace setup files
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/create3_ws/install/setup.bash
source ~/ros2_ws/turtlebot4_ws/install/setup.bash
source ~/ros2_ws/automodeROS/install/setup.bash
```

## Running Example Missions

### Start simulation with two robots
```bash
ros2 launch automode_tb4_sim turtlebot4_simulation.launch.py turtlebot4_id:=tb1 world:=mission
ros2 launch automode_tb4_sim add_turtlebot4_simulation.launch.py turtlebot4_id:=tb2 world:=mission
```

### Start AutoMoDe controller
```bash
ros2 launch automode_controller automode_tb4.launch.py \
  robot_namespace:=tb1 module_package:=basic_modules \
  fsm_config:="--fsm-config --nstates 3 --s0 0 --rwm0 50 --n0 1 --n0x0 0 --c0x0 2 --p0x0 1 --s1 2 --n1 1 --n1x0 1 --c1x0 1 --p1x0 1 --s2 3 --n2 1 --n2x0 0 --c2x0 0 --color2x0 2 --p2x0 1 "
```

## Extending
- Add new **behavior** or **condition** modules in a dedicated package.  
- Update `fsm_config` accordingly.  
- Use the `automode_tools` testing nodes to validate before deploying.

## License
This repository is licensed under the MIT License. See the `LICENSE` file for details.