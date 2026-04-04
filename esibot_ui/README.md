# esibot_ui — EsiBot Web Dashboard

Real-time web interface for the EsiBot robot, accessible from any browser on the local network.

**Features:**
- Real-time SLAM map (pan, zoom, re-center)
- Camera feed (raw or annotated by vision)
- Nav2 navigation goal (click on map)
- AMCL initial pose definition (click on map)
- SLAM map saving
- Keyboard teleoperation (WASD)
- Battery level, radar servo angle, LIDAR scan display

---

## Interface Panels

### Connection
Manages the WebSocket connection between the browser and the robot.

| Element | Description |
|---------|-------------|
| URL field | Rosbridge address — e.g. `ws://192.168.1.34:9090` |
| Connect/Disconnect button | Opens/closes the WebSocket connection |
| Latency | Round-trip time measured in ms |

**Data source:** no ROS2 topic — direct connection via `ROSLIB.Ros` to the rosbridge WebSocket.

---

### Map
Displays the SLAM map built by slam_toolbox, the robot position, its traveled path, LIDAR scan, and goals/poses.

| Element | ROS2 Topic | Description |
|---------|-----------|-------------|
| Map (gray/white/black background) | `/map` (`nav_msgs/OccupancyGrid`) | Received once at startup then on each SLAM update. Drawn on an offscreen canvas. |
| Robot position (circle) | `/odom` (`nav_msgs/Odometry`) | Continuously updated. Position `x, y` + heading `yaw` extracted from quaternion. |
| Traveled path (blue line) | `/odom` | Accumulated locally in the browser (max 500 points, 5 cm step). |
| LIDAR scan (red rays) | `/scan` (`sensor_msgs/LaserScan`) | Optional (`SCAN_OVERLAY=true` in config.js). Projected on the map based on robot pose. |
| Navigation goal (flag) | Computed on click | Displayed after clicking on the map in Goal mode. Canvas coordinates → world coordinates. |
| Initial pose (green cross) | Computed on click | Displayed after clicking in Initial Pose mode. |

**Map controls:**
- **Scroll wheel** — zoom (0.2× to 20×)
- **Click + drag** — pan the view
- **⊙ Re-center** — centers the robot on screen
- **⊕ Initial pose** — click on map → publishes to `/initialpose` (AMCL)
- **⚑ Send goal** — click on map → sends Nav2 goal via `nav_goal_proxy`
- **💾 Save Map** — publishes to `/save_map` → `map_saver_node` saves the map

---

### Camera
Displays the video feed from the robot's ESP32 camera.

| Mode | ROS2 Topic | Description |
|------|-----------|-------------|
| Annotated | `/camera/image_annotated` (`sensor_msgs/Image`) | Image with `esibot_vision` detections overlaid (boxes, labels). |
| Raw | `/camera/compressed` (`sensor_msgs/CompressedImage`) | Raw JPEG image, no processing. |

**Data source:** images received via rosbridge, converted to base64 and displayed in an `<img>` tag. Automatic throttle based on network bandwidth.

---

### Teleop
Allows manual keyboard control of the robot.

| Key | Action |
|-----|--------|
| `W` / `↑` | Move forward |
| `S` / `↓` | Move backward |
| `A` / `←` | Turn left |
| `D` / `→` | Turn right |

**Data source:** no input topic. Publishes to `/cmd_vel` (`geometry_msgs/Twist`) at 10 Hz while a key is held. Auto-stop 200 ms after key release.

Configurable speeds in `config.js`: `CMD_VEL.LINEAR_SPEED` (default 0.4 m/s), `CMD_VEL.ANGULAR_SPEED` (default 1.5 rad/s).

---

### Battery
Displays the robot battery level as a graphical bar.

| Element | Description |
|---------|-------------|
| Battery icon | Green > 50%, orange > 20%, red ≤ 20% |
| Percentage | Displayed as numeric value |
| Estimated remaining time | Calculated from `BATTERY_CAPACITY_MINUTES` in config.js |

**Data source:** `/battery_state` (`sensor_msgs/BatteryState`). Uses `percentage` field (value 0.0–1.0).

---

### Servo Gauge
Displays the current angle of the servo motor that orients the ultrasonic radar.

| Element | Description |
|---------|-------------|
| Semi-circular gauge | 240° arc representing 0°–180° |
| Numeric value | Angle in degrees displayed at center |

**Data source:** `/esibot/servo_angle` (`std_msgs/Float32`). Value in degrees between 0 and 180.

---

## Architecture

```
Robot (Docker container)
├── bringup          →  motor driver, odometry, /cmd_vel, /odom, /tf
├── slam             →  SLAM map (/map, TF map→odom)
├── sensors          →  radar (/scan, /esibot/servo_angle)
├── camera           →  image feed (/camera/compressed, /camera/image_annotated)
├── vision           →  object detection (annotates /camera/image_annotated)
├── nav2             →  autonomous navigation (/navigate_to_pose)
│
├── dashboard_node   →  http://robot-ip:8080   (serves static web files)
├── web_bridge       →  ws://robot-ip:9090     (rosbridge: ROS2 ↔ browser WebSocket)
├── nav_goal_proxy   →  receives /nav_goal (topic) → sends NavigateToPose action
└── map_saver_node   →  receives /save_map (topic) → runs map_saver_cli

Browser (PC / phone)
└── opens http://robot-ip:8080 → automatically connects to ws://robot-ip:9090
```

### Why a nav_goal_proxy?

The web dashboard cannot directly call a ROS2 action (protocol too complex for rosbridge in this context). The proxy bridges the gap:

```
Dashboard JS  →  publishes PoseStamped on /nav_goal  →  nav_goal_proxy  →  NavigateToPose action  →  bt_navigator
                                                                        ↓
Dashboard JS  ←  subscribes /nav_goal_status (String) ←────────────────
                 (sending | navigating | reached | error)
```

### Why the cmd_vel relay?

Nav2 publishes on `/cmd_vel_nav` → `velocity_smoother` smooths and publishes on `/cmd_vel_smoothed`.
But the robot driver listens on `/cmd_vel`. A relay bridges the gap:

```
velocity_smoother (/cmd_vel_smoothed)  →  relay  →  /cmd_vel  →  esibot_driver
```

This relay must be launched manually (see Launch section).

---

## Installation (once)

### 1. Build the React dashboard

The React source code is located in `dashboard/` at the workspace root.

```bash
cd ~/esibot_ws/src/dashboard
npm install
npm run build
cp -r dist/ ../esibot_ui/web/
```

### 2. Build ROS2 packages

```bash
cd ~/esibot_ws
colcon build --packages-select web_bridge esibot_ui
source install/setup.bash
```

> **Important:** after any modification to `nav_goal_proxy.py`, `map_saver_node.py` or `dashboard_node.py`, run `colcon build --packages-select esibot_ui` without `--symlink-install` to ensure package metadata is correctly installed.

---

## Full Launch (order matters)

### Step 1 — Launch robot packages

**Real hardware:**

```bash
# Terminal 1 — driver + odometry
ros2 launch esibot_bringup bringup.launch.py

# Terminal 2 — radar sensor
ros2 launch esibot_sensors radar.launch.py

# Terminal 3 — SLAM
ros2 launch esibot_slam slam.launch.py mode:=hw

# Terminal 4 — camera
ros2 launch esibot_camera camera.launch.py esp32_ip:=192.168.1.80

# Terminal 5 — vision (optional — CPU intensive)
ros2 launch esibot_vision vision.launch.py
```

**Simulation (no hardware):**

```bash
ros2 launch esibot_bringup bringup.launch.py sim_mode:=true
ros2 launch esibot_sensors radar.launch.py   sim_mode:=true
ros2 launch esibot_slam    slam.launch.py    mode:=hw
ros2 launch esibot_camera  camera.launch.py  sim_mode:=true
ros2 launch esibot_vision  vision.launch.py
```

### Step 2 — Launch Nav2 (autonomous navigation)

**Active SLAM mode (map built in real-time):**

```bash
ros2 launch esibot_navigation nav2.launch.py use_rviz:=false slam_mode:=true
```

**Pre-built map mode (AMCL + localization):**

```bash
ros2 launch esibot_navigation nav2.launch.py use_rviz:=false
```

> Wait for `Managed nodes are active` message before continuing.

### Step 3 — Launch cmd_vel relay

Without this relay, the robot will not move physically during Nav2 navigation.

```bash
ros2 run topic_tools relay /cmd_vel_smoothed /cmd_vel
```

### Step 4 — Launch dashboard

```bash
ros2 launch esibot_ui dashboard.launch.py
```

Automatically launches:
- `dashboard_node` (HTTP server port 8080)
- `rosbridge_websocket` (WebSocket port 9090)
- `nav_goal_proxy` (NavigateToPose action proxy)
- `map_saver_node` (map saving)

### Step 5 — Open in browser

```
http://<robot-ip>:8080
```

---

## Known Issues and Solutions

### Nav2 fails to start — "Failed to change state for node: controller_server"

**Cause:** multiple Nav2 instances running simultaneously (DDS ghost participants after kill).

**Solution:**
```bash
# 1. Kill all nav2 processes
kill -9 $(ps aux | grep -E 'controller_server|bt_navigator|lifecycle_manager|planner_server|smoother_server|behavior_server|velocity_smoother|collision_monitor|waypoint_follower|opennav_docking' | grep -v grep | awk '{print $2}') 2>/dev/null

# 2. Wait 30 seconds (mandatory DDS cleanup)
sleep 30

# 3. Stop ROS2 daemon
ros2 daemon stop

# 4. Relaunch Nav2
ros2 launch esibot_navigation nav2.launch.py use_rviz:=false slam_mode:=true
```

### Robot does not move physically during navigation

**Cause:** the `/cmd_vel_smoothed → /cmd_vel` relay is not running.

**Solution:** launch the relay (Step 3 above).

**Verification:**
```bash
ros2 topic hz /cmd_vel   # should show a frequency during navigation
```

### Dashboard stays on "Sending…" after sending a goal

**Cause A:** Nav2 is not launched or not yet active.
**Solution:** wait for `Managed nodes are active` in Nav2 logs.

**Cause B:** DDS conflict — `nav_goal_proxy` running as duplicate instance.
**Solution:** kill all nav_goal_proxy processes and relaunch only the dashboard.

### TF error "Lookup would require extrapolation into the future"

**Cause:** SLAM toolbox or MPPI overloaded on CPU, ~50ms gap in TF.

**Solution:** reduce CPU load — stop `esibot_vision` if not needed (consumes >500% CPU).
```bash
kill -9 $(pgrep -f vision_node)
```

MPPI parameters in `esibot_navigation/config/nav2_params.yaml` are already optimized:
```yaml
controller_frequency: 10.0
model_dt:             0.1
batch_size:           500
time_steps:           30
```

---

## Modifying the Dashboard (React)

React source code is in `dashboard/` at the workspace root (not in this package).

After modifying a file in `dashboard/src/`:

```bash
# 1. Remove old build
rm -rf ~/esibot_ws/src/esibot_ui/web/

# 2. Rebuild React
cd ~/esibot_ws/src/dashboard
npm run build
cp -r dist/ ../esibot_ui/web/

# 3. Rebuild ROS2 package
cd ~/esibot_ws
colcon build --packages-select esibot_ui
source install/setup.bash

# 4. Restart
ros2 launch esibot_ui dashboard.launch.py
```

---

## Configuration

File: `dashboard/src/config.js`

| Parameter | Default | Description |
|-----------|---------|-------------|
| `ROSBRIDGE_URL` | `ws://localhost:9090` | Robot IP address |
| `ROBOT_NAME` | `EsiBot` | Name displayed in the interface |
| `CMD_VEL.LINEAR_SPEED` | `0.4` | Teleop linear speed (m/s) |
| `CMD_VEL.ANGULAR_SPEED` | `1.5` | Teleop angular speed (rad/s) |
| `BATTERY_CAPACITY_MINUTES` | `45` | Estimated battery life (min) |
| `SCAN_OVERLAY` | `false` | Display LIDAR scan on map |

---

## ROS2 Topics

| Topic | Direction | Type | Description |
|-------|-----------|------|-------------|
| `/map` | ← robot | `nav_msgs/OccupancyGrid` | SLAM map |
| `/odom` | ← robot | `nav_msgs/Odometry` | Robot position and speed |
| `/scan` | ← robot | `sensor_msgs/LaserScan` | Radar LIDAR scan |
| `/camera/compressed` | ← robot | `sensor_msgs/CompressedImage` | Raw camera feed |
| `/camera/image_annotated` | ← robot | `sensor_msgs/Image` | Camera with vision detections |
| `/battery_state` | ← robot | `sensor_msgs/BatteryState` | Battery level |
| `/esibot/servo_angle` | ← robot | `std_msgs/Float32` | Radar servo angle |
| `/nav_goal_status` | ← robot | `std_msgs/String` | Navigation status (sending/navigating/reached/error) |
| `/save_map_status` | ← robot | `std_msgs/String` | Map save status |
| `/cmd_vel` | → robot | `geometry_msgs/Twist` | Teleop velocity commands |
| `/nav_goal` | → robot | `geometry_msgs/PoseStamped` | Navigation goal (via nav_goal_proxy) |
| `/initialpose` | → robot | `geometry_msgs/PoseWithCovarianceStamped` | AMCL initial pose |
| `/save_map` | → robot | `std_msgs/Empty` | Triggers map saving |

---

## Nodes launched by dashboard.launch.py

| Node | Package | Description |
|------|---------|-------------|
| `dashboard_node` | `esibot_ui` | Python HTTP server (port 8080), serves static web files |
| `rosbridge_websocket` | `rosbridge_server` | WebSocket bridge ROS2 ↔ browser (port 9090) |
| `nav_goal_proxy` | `esibot_ui` | Receives `/nav_goal`, calls `/navigate_to_pose` action, publishes status on `/nav_goal_status` |
| `map_saver_node` | `esibot_ui` | Receives `/save_map`, runs `map_saver_cli`, publishes status on `/save_map_status` |

---

## Quick Diagnostic Commands

```bash
# Check all topics are arriving
ros2 topic list | grep -E 'map|odom|scan|cmd_vel|camera|battery'

# Check scan frequency
ros2 topic hz /scan

# Check robot receives velocity commands
ros2 topic hz /cmd_vel

# Check Nav2 cmd_vel chain
ros2 topic hz /cmd_vel_nav       # controller_server → smoother
ros2 topic hz /cmd_vel_smoothed  # smoother → relay
ros2 topic hz /cmd_vel           # relay → driver

# Test a Nav2 goal from terminal
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 1.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}"

# Check Nav2 lifecycle state
ros2 lifecycle list

# View available TFs
ros2 run tf2_tools view_frames

# Check active nodes
ros2 node list
```
