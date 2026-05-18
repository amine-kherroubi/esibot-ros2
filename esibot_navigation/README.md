# esibot_navigation

`esibot_navigation` is the Nav2 configuration package for this repository.
It provides launch wiring, Nav2 parameters, and map assets.

## Scope in this repository

In-repo files:

- `launch/nav2.launch.py`
- `config/nav2_params.yaml`
- `maps/esibot_map.yaml`
- `maps/esibot_map.pgm`

External runtime dependency (not vendored here):

- `nav2_bringup` package share, including `launch/bringup_launch.py`

`launch/nav2.launch.py` resolves that external path with `get_package_share_directory("nav2_bringup")` and includes `launch/bringup_launch.py` from the installed package.

## Declared package dependencies

From `package.xml`:

- `ament_index_python`
- `launch`
- `launch_ros`
- `nav2_common`
- `nav2_bringup`
- `opennav_docking`
- `rviz2`

This package is built with `ament_cmake` and installs only `launch`, `config`, and `maps` directories.

## Launch behavior (`launch/nav2.launch.py`)

The launch file:

1. Sets default paths for map and parameter files from this package share.
2. Declares launch arguments:
   - `map`
   - `params_file`
   - `scan_topic`
   - `use_sim_time`
   - `autostart`
   - `use_composition`
   - `use_respawn`
   - `use_rviz`
   - `rviz_config`
3. Rewrites selected Nav2 parameter keys with `RewrittenYaml` so `scan_topic` can be overridden at launch time.
4. Includes upstream Nav2 bringup with `slam:=False` and `use_localization:=True`.
5. Optionally launches RViz.

## Runtime interfaces expected by this configuration

- TF frames used in `config/nav2_params.yaml` include `map`, `odom`, and `base_footprint`.
- Default scan topic in this package is `scan`.
- `scan_topic` can be overridden (for example to `ultrasound_raw`).
- `use_sim_time` defaults to `false` in `launch/nav2.launch.py`.

Related producers in this repository:

- `esibot_sensors/esibot_sensors/radar_node.py` publishes `/scan`.
- `esibot_gazebo/launch/sim.launch.py` bridges `/ultrasound_raw` and `/clock` from Gazebo.

## Build

```bash
cd ~/robot_ws
colcon build --symlink-install --packages-select esibot_navigation
source ~/robot_ws/install/setup.bash
```

## Run examples

Hardware-oriented run:

```bash
ros2 launch esibot_navigation nav2.launch.py use_sim_time:=false scan_topic:=scan
```

Gazebo-oriented run:

```bash
ros2 launch esibot_navigation nav2.launch.py use_sim_time:=true scan_topic:=ultrasound_raw
```

Override map and params:

```bash
ros2 launch esibot_navigation nav2.launch.py \
  map:=/absolute/path/to/your_map.yaml \
  params_file:=/absolute/path/to/nav2_params.yaml
```

## Verification commands (target machine)

```bash
# External dependency present
ros2 pkg prefix nav2_bringup

# bringup_launch.py present in installed package share
python3 -c "import os,glob; from ament_index_python.packages import get_package_share_directory as g; d=g('nav2_bringup'); print(d); print(glob.glob(os.path.join(d,'launch','bringup_launch.py')))"

# This package declares nav2_bringup dependency
ros2 pkg xml esibot_navigation | grep nav2_bringup
```
