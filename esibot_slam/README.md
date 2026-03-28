# esibot_slam

**ROS 2 Package — Task 3.5: SLAM with slam_toolbox**
EsiBot Robot | ROS 2 Jazzy | Gazebo Harmonic | slam_toolbox (online_async)

---

## Overview

`esibot_slam` configures and launches SLAM for the EsiBot robot using `slam_toolbox` in `online_async` mode, adapted to EsiBot's rotating ultrasound radar (HC-SR04 + SG90 servo).

```
/scan  ──→  slam_toolbox  ──→  /map        (OccupancyGrid — live 2D map)
/odom  ──↗                ──→  TF map→odom (robot position in the map)
/tf    ──↗
```

### Related packages

| Package                 | Role                                                   |
| ----------------------- | ------------------------------------------------------ |
| `esibot_sensors`        | Publishes `/scan` via `radar_node`                     |
| `esibot_bringup`        | Publishes `/odom` via `esibot_driver`                  |
| `esibot_description`    | URDF + TF tree via `robot_state_publisher`             |
| `esibot_gazebo`         | Gazebo Harmonic simulation                             |
| `esibot_bringup` (Nav2) | Autonomous navigation — consumes `/map` and `map→odom` |

---

## Package structure

```
esibot_slam/
├── CMakeLists.txt
├── package.xml
├── config/
│   ├── slam_params_sim.yaml
│   ├── slam_params_hw.yaml
│   └── esibot_slam.rviz
├── launch/
│   ├── slam.launch.py
│   ├── slam_sim.launch.py
│   └── save_map.launch.py
└── maps/
```

---

## Installation

```bash

cd ~/esibot_ws
colcon build --packages-select esibot_slam --symlink-install
source install/setup.bash
```

---

## Usage

### Option A — Simulation (Gazebo already running)

```bash
# Terminal 1: Gazebo
ros2 launch esibot_gazebo sim.launch.py

# Terminal 2: SLAM
ros2 launch esibot_slam slam.launch.py use_rviz:=true

# Terminal 3 (optional): teleoperation
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### Option B — Real hardware (Raspberry Pi 4)

```bash
# Terminal 1: bringup
ros2 launch esibot_bringup bringup.launch.py

# Terminal 2: SLAM
ros2 launch esibot_slam slam.launch.py mode:=hw use_rviz:=true teleop:=true
```

### Save the map

```bash
ros2 launch esibot_slam save_map.launch.py

# Custom name
ros2 launch esibot_slam save_map.launch.py map_name:=lab_final
```

Produces `maps/esibot_map.pgm` and `maps/esibot_map.yaml`.

---

## Key parameters

| Parameter               | Simulation       | Hardware         |
| ----------------------- | ---------------- | ---------------- |
| `use_sim_time`          | `true`           | `false`          |
| `max_laser_range`       | `4.0 m`          | `4.0 m`          |
| `resolution`            | `0.05 m`         | `0.05 m`         |
| `minimum_time_interval` | `0.3 s`          | `0.5 s`          |
| `map_update_interval`   | `2.0 s`          | `5.0 s`          |
| `transform_timeout`     | `0.5 s`          | `1.0 s`          |
| `base_frame`            | `base_footprint` | `base_footprint` |
| `scan_topic`            | `/scan`          | `/scan`          |

---

## Integration interface

### Inputs

| Topic    | Type                    | Producer                               |
| -------- | ----------------------- | -------------------------------------- |
| `/scan`  | `sensor_msgs/LaserScan` | `esibot_sensors`                       |
| `/odom`  | `nav_msgs/Odometry`     | `esibot_bringup`                       |
| `/tf`    | TF tree                 | `esibot_description` / `esibot_gazebo` |
| `/clock` | `rosgraph_msgs/Clock`   | Gazebo (sim only)                      |

### Outputs

| Output                       | Type                     | Consumer          |
| ---------------------------- | ------------------------ | ----------------- |
| `/map`                       | `nav_msgs/OccupancyGrid` | Nav2, RViz2       |
| TF `map → odom`              | TF                       | Nav2 localization |
| `maps/*.pgm` / `maps/*.yaml` | Files                    | Nav2 map_server   |

---

### Map quality

The HC-SR04 produces lower-quality maps than a LiDAR (10°/step, 1–2 Hz, ±15° cone). **Mitigation**: reduce angular step to 5°, move slowly, and use environments with clear geometric features.

---

## Verification

```bash
ros2 topic hz /scan
ros2 topic hz /map
ros2 run tf2_tools view_frames     # TF tree → frames.pdf
ros2 service list | grep slam_toolbox
ros2 node list
```

---
