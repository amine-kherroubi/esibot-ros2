# EsiBot Navigation

Nav2 bringup and configuration for EsiBot (ROS 2 Jazzy).

## Build

```bash
cd ~/robot_ws
colcon build --symlink-install --packages-select esibot_navigation
source ~/robot_ws/install/setup.bash
```

## Run (default)

Uses the standard LaserScan topic `scan`.

```bash
ros2 launch esibot_navigation nav2.launch.py
```

## Run (Gazebo sim)

Gazebo publishes `/ultrasound_raw`, so override the scan topic:

```bash
ros2 launch esibot_navigation nav2.launch.py scan_topic:=ultrasound_raw
```

## Override map or params

```bash
ros2 launch esibot_navigation nav2.launch.py \
  map:=/absolute/path/to/your_map.yaml \
  params_file:=/absolute/path/to/nav2_params.yaml
```
