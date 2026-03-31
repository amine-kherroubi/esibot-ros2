# esibot_navigation

ROS 2 Jazzy navigation configuration for EsiBot. It launches Nav2 with EsiBot-specific settings (map, parameters, and launch wiring). This package does not implement custom nodes or algorithms; it only configures the Nav2 stack for the robot or simulation.

## Build

```bash
cd ~/robot_ws
colcon build --symlink-install --packages-select esibot_navigation
source ~/robot_ws/install/setup.bash
```

## Run

Real robot (default settings assume LaserScan topic `scan`):

```bash
ros2 launch esibot_navigation nav2.launch.py
```

Simulation (Gazebo publishes `ultrasound_raw`):

```bash
ros2 launch esibot_navigation nav2.launch.py scan_topic:=ultrasound_raw
```

Override map or params:

```bash
ros2 launch esibot_navigation nav2.launch.py \
  map:=/absolute/path/to/your_map.yaml \
  params_file:=/absolute/path/to/nav2_params.yaml
```

## Launch Arguments

- `map`: full path to the map YAML
- `params_file`: full path to `nav2_params.yaml`
- `scan_topic`: LaserScan topic used by AMCL and costmaps
- `use_sim_time`: `True` for simulation (`/clock`), `False` for real robot
- `use_rviz`: launch RViz
- `rviz_config`: RViz config path (defaults to Nav2 default view)

## Simulation vs. real robot

| Setting        | Simulation       | Real robot | Rationale                                                                                                          |
| -------------- | ---------------- | ---------- | ------------------------------------------------------------------------------------------------------------------ |
| `use_sim_time` | `True`           | `False`    | Gazebo publishes `/clock`. If `use_sim_time` is `True` on a real robot, nodes wait on `/clock` and appear stalled. |
| `scan_topic`   | `ultrasound_raw` | `scan`     | The simulated sensor publishes a different LaserScan topic than the real robot.                                    |

## Caveats and Rationale

1. `use_sim_time` defaults to `true` in `launch/nav2.launch.py`. On real hardware, set `use_sim_time:=false` or Nav2 will not advance time and the stack will appear frozen.
2. `scan_topic` must match the actual LaserScan source. AMCL, the local costmap, the global costmap, and the collision monitor all consume this topic. If it is wrong, localization will not converge and the costmaps will be empty.
3. Frame IDs assume `map -> odom -> base_footprint`. If your robot uses `base_link` or another base frame, update `amcl.base_frame_id`, `bt_navigator.robot_base_frame`, `local_costmap.robot_base_frame`, and `global_costmap.robot_base_frame` in `config/nav2_params.yaml`. A mismatch causes TF errors and Nav2 will fail to localize or plan.
4. `collision_monitor.footprint_topic` is relative (`local_costmap/published_footprint`) to stay namespace-safe. If you intentionally use absolute topics, adjust it to match your graph.
5. The local costmap defines a `static_layer` block but does not list it in `local_costmap.plugins`, so that block is ignored. Add `static_layer` to the plugin list if you want static map data in the local costmap.
6. The map YAML references the image with a relative path. If you move the YAML or the image, update `maps/esibot_map.yaml` or the map server will fail to load the map.

## Package Layout

- `launch/nav2.launch.py`: wraps `nav2_bringup` and injects map, params, and scan topic
- `config/nav2_params.yaml`: Nav2 configuration
- `maps/esibot_map.yaml` and `maps/esibot_map.pgm`: static map
