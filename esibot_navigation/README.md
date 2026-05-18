# esibot_navigation

`esibot_navigation` is the **Nav2 integration package** for EsiBot on ROS 2 Jazzy.
It does not implement custom planners/controllers; instead, it wires together upstream Nav2 components, supplies EsiBot-specific parameters, and chooses the map and sensor topics Nav2 should use.

---

## 1) What this package is, from first principles

At a first-principles level, autonomous navigation has four core jobs:

1. **Know where the robot is** in a fixed world frame (`map`) — localization.
2. **Know what space is free vs occupied** right now — costmaps built from map + live range sensing.
3. **Choose a route** to a goal — global planning.
4. **Generate safe velocity commands** to follow that route while avoiding obstacles — local control and safety monitoring.

This package contributes to those jobs by:

- launching Nav2 bringup (`nav2_bringup/bringup_launch.py`),
- feeding it the static map (`maps/esibot_map.yaml` + `.pgm`),
- supplying EsiBot tuning (`config/nav2_params.yaml`),
- and rewriting scan-topic parameters at launch so the same config can run in sim and hardware.

In short: this package is the **integration boundary** between EsiBot’s robot graph and Nav2’s generic navigation stack.

---

## 2) Dependency map with other packages (and what each one contributes)

`esibot_navigation` depends on both ROS/Nav2 packages and other EsiBot packages running in the graph.

### Direct package-level dependencies

From `package.xml`, this package directly depends on:

- `nav2_bringup` / `nav2_common`: launch and configuration infrastructure for Nav2
- `launch`, `launch_ros`, `ament_index_python`: launch-time plumbing
- `rviz2`: optional visualization
- `opennav_docking`: docking-related behavior/plugins expected by the Nav2 config

### Runtime dependencies on the rest of this codebase

Even if not declared as `exec_depend`, navigation correctness relies on data produced elsewhere:

- **`esibot_description`** (or equivalent TF publisher): must provide a coherent TF tree such as `map -> odom -> base_footprint` (or consistently configured alternate base frame).
- **`esibot_sensors`**: provides range data used as Nav2 `scan_topic` input (`scan` on hardware, `ultrasound_raw` in sim according to this repo’s conventions).
- **`esibot_slam`**: typically used earlier to build/maintain the map that `esibot_navigation` later consumes for localization-only navigation.
- **`esibot_bringup`**: usually starts robot drivers/control nodes that execute `/cmd_vel` outputs produced by Nav2.
- **`esibot_gazebo`**: in simulation, provides world, robot clock (`/clock`), and simulated sensor topics.

### Data-flow view (how packages connect)

1. Sensors package publishes LaserScan (`scan` or `ultrasound_raw`).
2. Nav2 localization (`amcl`) uses scan + map to estimate robot pose in `map` frame.
3. Costmap layers fuse map + scan into obstacle representations.
4. Planner/controller servers compute and command motion.
5. Bringup/driver package consumes velocity commands and actuates the robot.
6. TF providers keep transforms consistent so all of the above use the same spatial geometry.

If any of these links is broken (missing TF, wrong scan topic, stale map, no driver), navigation appears “up” but is functionally unusable.

---

## 3) How `nav2.launch.py` works exactly

`launch/nav2.launch.py` is a thin orchestrator. In order:

1. Resolves default file paths:
   - map: `maps/esibot_map.yaml`
   - params: `config/nav2_params.yaml`
   - RViz config: Nav2 default RViz file from `nav2_bringup`
2. Declares launch arguments (`map`, `params_file`, `scan_topic`, `use_sim_time`, `autostart`, `use_rviz`, etc.).
3. Builds a `RewrittenYaml` wrapper to patch scan-topic keys at launch-time:
   - `amcl`
   - local costmap voxel layer
   - global costmap obstacle layer
   - collision monitor
4. Includes upstream `nav2_bringup/bringup_launch.py` with:
   - `slam:=False` (localization on a prebuilt map)
   - `use_localization:=True`
   - map/params/use_sim_time/autostart forwarded
5. Optionally launches RViz, and auto-disables it by default when no display is detected.

This design gives one stable Nav2 config file while still allowing environment-specific sensor topic differences.

---

## 4) Operational contract (what must be true for Nav2 to work)

For successful operation, all of the following should hold:

- TF contract is valid (`map`, `odom`, robot base frame agree with Nav2 params).
- `scan_topic` exists, publishes valid `sensor_msgs/LaserScan`, and matches launch arg override.
- Static map YAML/image are readable and aligned with the physical/sim environment.
- `use_sim_time` matches environment:
  - `True` in Gazebo or any `/clock`-driven simulation
  - `False` on hardware unless an external clock is explicitly provided
- A downstream motion interface consumes Nav2 velocity commands.

---

## 5) Build and run

## Build

```bash
cd ~/robot_ws
colcon build --symlink-install --packages-select esibot_navigation
source ~/robot_ws/install/setup.bash
```

## Run

Real robot:

```bash
ros2 launch esibot_navigation nav2.launch.py use_sim_time:=false scan_topic:=scan
```

Simulation:

```bash
ros2 launch esibot_navigation nav2.launch.py use_sim_time:=true scan_topic:=ultrasound_raw
```

Override map/params:

```bash
ros2 launch esibot_navigation nav2.launch.py \
  map:=/absolute/path/to/your_map.yaml \
  params_file:=/absolute/path/to/nav2_params.yaml
```

---

## 6) Launch arguments

- `map`: full path to map YAML
- `params_file`: full path to Nav2 params YAML
- `scan_topic`: LaserScan topic for AMCL + costmaps + collision monitor
- `use_sim_time`: use `/clock` time source
- `autostart`: auto-activate Nav2 lifecycle nodes
- `use_rviz`: launch RViz
- `rviz_config`: RViz config path

---

## 7) Known caveats in current configuration

1. If `scan_topic` is incorrect, AMCL will fail to converge and obstacle layers will be empty.
2. Frame mismatches (e.g., `base_link` vs `base_footprint`) create TF lookup failures and block planning/control.
3. Map YAML uses a relative image path; moving map assets without updating YAML will break map server loading.
4. Headless systems default `use_rviz:=false`; explicitly set `use_rviz:=true` only when a display is available.

---

## 8) Package layout

- `launch/nav2.launch.py`: Nav2 bringup wrapper + parameter rewriting
- `config/nav2_params.yaml`: EsiBot Nav2 tuning
- `maps/esibot_map.yaml`, `maps/esibot_map.pgm`: localization map assets
