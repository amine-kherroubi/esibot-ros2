# EsiBot — Project README

This document is the canonical project README. It describes system requirements, installation, build and launch procedures, package dependencies, architecture, and key ROS 2 topics used by our EsiBot workspace.

## Contents

- Overview
- System requirements
- Installation & build
- Launch examples (full bringup)
- Packages and dependencies
- Architecture and topic map
- Useful commands

## Overview

EsiBot is an autonomous differential-drive robot platform integrating a Raspberry Pi 4 (ROS 2) and an ESP32-CAM. The software stack provides device drivers, sensor processing, camera/vision, SLAM (slam_toolbox), Nav2-based navigation, a browser dashboard, and optional Gazebo simulation.

## System requirements

- Ubuntu 24.04 LTS
- ROS 2 Jazzy
- Python 3 (system Python provided by Ubuntu 24.04)
- `colcon` and `rosdep` (development tools)

The instructions below provide a complete set of platform prerequisites and a concise ROS 2 Jazzy installation procedure so this README is self-contained.

### Platform prerequisites

- ROS 2 Jazzy requires Ubuntu 24.04. Use one of the following environments:
    - Native Ubuntu 24.04 (recommended)
    - WSL2 on Windows with Ubuntu-24.04 (suitable for development; hardware access is limited)
    - Virtual machine (VirtualBox, VMware) — acceptable but has reduced simulator performance

### System preparation

Update the system and install essential tools:

```bash
sudo apt update && sudo apt upgrade -y
sudo apt install software-properties-common curl git python3-pip -y
sudo add-apt-repository universe
```

### Install ROS 2 Jazzy (concise steps)

1. Import the ROS 2 archive signing key:

```bash
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg
```

2. Register the ROS 2 repository and install the desktop package:

```bash
echo "deb [arch=$(dpkg --print-architecture) \
    signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
    http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
    | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install ros-jazzy-desktop -y
```

3. Install development tools and initialise `rosdep`:

```bash
sudo apt install python3-colcon-common-extensions python3-rosdep -y
sudo rosdep init || true
rosdep update --rosdistro jazzy
rosdep update
```

4. Configure the shell to source ROS 2 on startup and set the project `ROS_DOMAIN_ID` convention:

```bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
echo "export ROS_DOMAIN_ID=67" >> ~/.bashrc
source ~/.bashrc
```

### Optional packages commonly used by this project

Install convenience and simulation packages used by the workspace (examples):

```bash
sudo apt install -y \
    ros-jazzy-turtlebot3-gazebo \
    ros-jazzy-turtlebot3-teleop \
    ros-jazzy-slam-toolbox \
    ros-jazzy-navigation2 ros-jazzy-nav2-bringup \
    ros-jazzy-ros2-control ros-jazzy-ros2-controllers \
    ros-jazzy-ros-gz ros-jazzy-ros-gz-bridge ros-jazzy-ros-gz-image \
    ros-jazzy-foxglove-bridge ros-jazzy-teleop-twist-keyboard
```

### Recommended utilities and verification

- Install `tmux` for terminal session management: `sudo apt install tmux -y`.
- Verify ROS 2 communication in two separate terminals:

```bash
# Terminal 1
ros2 run demo_nodes_cpp talker

# Terminal 2
ros2 run demo_nodes_py listener
```

If the listener prints messages from the talker then the ROS 2 DDS layer is operating correctly.

### Workspace setup summary

Create the workspace, install package dependencies and build:

```bash
mkdir -p ~/robot_ws/src
cd ~/robot_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

Clone the project into `~/robot_ws/src` if needed:

```bash
cd ~/robot_ws/src
git clone https://github.com/amine-kherroubi/robot.git .
```

These steps make this README self-contained with respect to platform and ROS 2 installation.

## Installation & build

Follow the ROS 2 setup guide first, then install workspace-level dependencies and build:

```bash
# (1) Update system
sudo apt update && sudo apt upgrade -y

# (2) Configure ROS 2 repository and install ROS 2 Jazzy (see setup guide for full steps)
# Example (register key + install desktop):
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) \
    signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
    http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
    | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install ros-jazzy-desktop -y

# (3) Developer tools
sudo apt install python3-colcon-common-extensions python3-rosdep -y
sudo rosdep init || true
rosdep update --rosdistro jazzy

# (4) Install package-level system dependencies and build the workspace
cd ~/robot_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

Notes:

- Add `source /opt/ros/jazzy/setup.bash` and `export ROS_DOMAIN_ID=67` to `~/.bashrc` if desired (project convention uses `ROS_DOMAIN_ID=67`).
- For iterative development build only the package under change: `colcon build --packages-select <pkg> --symlink-install`.

## Launch (full bringup)

The canonical workspace bringup is `esibot_description/launch/full.launch.py`. It coordinates the robot description, driver, sensors, vision, SLAM and Nav2. Launch examples:

```bash
# Simulation + SLAM (build a new map)
ros2 launch esibot_description full.launch.py sim_mode:=true mode:=slam

# Simulation + Nav2 (navigate on an existing map)
ros2 launch esibot_description full.launch.py sim_mode:=true mode:=nav

# Camera + vision only (hardware ESP32-CAM required for real mode)
ros2 launch esibot_description full.launch.py mode:=vision

# Camera + vision in simulation
ros2 launch esibot_description full.launch.py sim_mode:=true mode:=vision
```

Launch arguments (summary):

- `mode` (choices: `slam`, `nav`, `vision`).
    - `slam`: run `slam_toolbox` to build a map (Nav2 inactive).
    - `nav`: run Nav2 to navigate on a saved map (slam_toolbox inactive).
    - `vision`: camera and vision nodes only.
- `sim_mode` (`true` / `false`): when `true` hardware interfaces (ESP32, GPIO) are disabled and sensors are simulated.
- `use_foxglove` (`true` / `false`): when `true` launches `foxglove_bridge` (default `true`).

Typical launch order (logical overview):

1. `robot_state_publisher` — publishes `robot_description` and `/tf_static`.
2. `foxglove_bridge` — WebSocket bridge for browser visualization.
3. `esibot_driver` (bringup) — hardware bridge, publishes `/odom` and `/tf`.
4. Sensor nodes (`radar_node`, `camera_node`) — publish `/scan`, `/camera/*`.
5. Perception (`esibot_vision`), SLAM (`slam_toolbox`) and Nav2 as selected by `mode`.

See `esibot_description/launch/full.launch.py` for exact timing and inclusion details.

## Packages and dependencies

Dependencies are declared in each package `package.xml`. Use `rosdep` to install system dependencies:

```bash
rosdep install --from-paths src --ignore-src -r -y
```

Key runtime dependencies used across the workspace include (representative, non-exhaustive):

- ROS packages: `rclpy`, `rclcpp`, `robot_state_publisher`, `xacro`, `tf2_ros`, `slam_toolbox`, `nav2_bringup`, `nav2_map_server`, `ros_gz_sim`, `ros_gz_bridge`, `ros_gz_image`, `foxglove_bridge`, `rosbridge_server`, `rosbridge_msgs`, `rosapi`, `rviz2`, `topic_tools`.
- System Python packages: `python3-opencv`, `python3-numpy`, `python3-serial`, `python3-colcon-common-extensions`.

Refer to each package's `package.xml` for exact build-time and run-time requirements.


## Architecture and topic map

The system is organized in logical layers: hardware, drivers, sensors, perception, mapping (SLAM), planning (Nav2), actuation, and visualization. Components communicate via ROS 2 topics and actions; bridge components expose selected topics to external tools and the browser-based dashboard.

Primary interactions (producer → consumer):

- `esibot_driver`: publishes `/odom` and `/tf`; subscribes to `/cmd_vel`.
- Sensor nodes: `radar_node` publishes `/scan`; `esibot_camera` publishes `/camera/image_raw`, `/camera/camera_info`, and `/camera/compressed`.
- Perception: `esibot_vision` consumes camera topics and publishes processed outputs such as `/camera/image_annotated`.
- SLAM: `slam_toolbox` consumes `/scan` and `/odom`, publishes `/map` and the `map→odom` transform.
- Navigation (Nav2): consumes `/map`, `/odom`, and goal inputs; outputs velocity commands (e.g. `/cmd_vel_nav`) that are smoothed and relayed to `/cmd_vel` for execution by the driver.
- UI and bridges: `foxglove_bridge` and `rosbridge_server` expose topics, services, and actions to remote clients and the dashboard.

Important ROS topics (representative):

- `/robot_description` — URDF (produced by `robot_state_publisher`).
- `/tf_static`, `/tf` — transform data.
- `/odom` (`nav_msgs/Odometry`) — produced by `esibot_driver`; consumed by SLAM, Nav2 and UI components.
- `/scan` (`sensor_msgs/LaserScan`) — produced by `radar_node`; consumed by SLAM, Nav2 and visualization.
- `/map` (`nav_msgs/OccupancyGrid`) — produced by `slam_toolbox`; consumed by Nav2 and UI.
- `/camera/image_raw`, `/camera/camera_info`, `/camera/compressed` — camera streams consumed by perception and UI.
- `/camera/image_annotated` — vision outputs containing detections and annotations.
- `/cmd_vel` (`geometry_msgs/Twist`) — actuator command inbound to `esibot_driver`.
- `/cmd_vel_nav`, `/cmd_vel_smoothed` — intermediary topics in the Nav2 → smoother → relay → driver pipeline.
- `/nav_goal` (`geometry_msgs/PoseStamped`) — navigation goal input from the UI; handled by a proxy that invokes Nav2 actions.
- `/nav_goal_status` (`std_msgs/String`) — navigation status published to the UI.
- `/save_map` (`std_msgs/Empty`) and `/save_map_status` — control and status for saving maps to disk.
- `/battery_state` (`sensor_msgs/BatteryState`) — battery telemetry published by the driver.

Representative nodes: `robot_state_publisher`, `esibot_driver`, `radar_node`, `esibot_camera`, `esibot_vision`, `slam_toolbox`, `nav2` (with lifecycle manager), `foxglove_bridge`, `rosbridge_server`, `dashboard_node`, `nav_goal_proxy`, and `map_saver_node`.

## Useful commands

```bash
# Build and source
colcon build --symlink-install
source install/setup.bash

# List topics and nodes
ros2 topic list
ros2 node list

# Inspect topic rate
ros2 topic hz /scan

# Relay: forward smoothed cmd_vel to driver
ros2 run topic_tools relay /cmd_vel_smoothed /cmd_vel

# Record / replay bag
ros2 bag record -a -o session_$(date +%Y%m%d_%H%M)
ros2 bag play <bagfile>
```
