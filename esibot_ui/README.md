# esibot_ui — EsiBot Web Dashboard

Real-time web interface for the EsiBot robot, accessible from any browser on the local network.

---

## How It Works

```
Browser (PC / phone)
│  opens http://<robot-ip>:8080
│  connects to ws://<robot-ip>:9090 (rosbridge WebSocket)
│
├── Subscribes to ROS2 topics  →  displays map, camera, battery, position
└── Publishes to ROS2 topics   →  teleop commands, navigation goals
```

### Position: How the robot appears on the map

The dashboard composes two transforms to show the robot's true position on the map:

```
/tf (map→odom)       +       /odom (odom→base_footprint)
from slam_toolbox            from esibot_driver (wheel encoders)
"correction for drift"       "how far the wheels rolled"
        │                            │
        └──────── compose ───────────┘
                     │
                     ▼
        Robot position in MAP frame (accurate)
        x = tx + cos(θ)·ox − sin(θ)·oy
        y = ty + sin(θ)·ox + cos(θ)·oy
```

Why not just `/odom`? Because wheel odometry **drifts** — the wheels slip, and the error accumulates. SLAM corrects this drift via the `map→odom` transform on `/tf`. Without this correction, the robot would gradually appear at the wrong place on the map.

### Navigation: From click to motion

```
Click on map  →  JS converts pixel to world coords (meters)
      │
      ▼
Publishes PoseStamped on /nav_goal (via rosbridge WebSocket)
      │
      ▼
nav_goal_proxy  →  sends NavigateToPose action to Nav2
      │
      ▼
Nav2 plans path (NavfnPlanner) → follows it (MPPI controller)
      │
      ▼
/cmd_vel_nav → velocity_smoother → /cmd_vel_smoothed → relay → /cmd_vel → esibot_driver → motors
```

### Two navigation modes

| | SLAM mode (`slam_mode:=true`) | AMCL mode (`slam_mode:=false`) |
|---|---|---|
| Map source | slam_toolbox builds it live | map_server loads from file |
| Localization | slam_toolbox (map→odom TF) | AMCL particle filter |
| Initial pose needed? | No | **Yes** (click "Init Pose" first) |
| Save Map useful? | **Yes** (save for later use) | No (already saved) |
| CPU usage | Higher | Lower |

---

## ROS2 Topics

### Subscribed (robot → dashboard)

| Topic | Type | Source | Displays |
|-------|------|--------|----------|
| `/map` | `OccupancyGrid` | slam_toolbox | SLAM map (white=free, black=wall, gray=unknown) |
| `/tf` | `TFMessage` | slam_toolbox / AMCL | `map→odom` transform (drift correction) |
| `/odom` | `Odometry` | esibot_driver | Robot position (odom frame) + velocity |
| `/scan` | `LaserScan` | radar_node | Laser rays overlay on map |
| `/camera/compressed` | `CompressedImage` | esibot_camera | Raw video feed |
| `/camera/image_annotated` | `Image` | esibot_vision | Video with detection boxes |
| `/battery_state` | `BatteryState` | esibot_driver | Battery level (%) |
| `/esibot/servo_angle` | `Float32` | radar_node | Radar servo angle (0-180°) |
| `/nav_goal_status` | `String` | nav_goal_proxy | Navigation status (sending/navigating/reached/error) |
| `/save_map_status` | `String` | map_saver_node | Map save status (saving/saved/error) |

### Published (dashboard → robot)

| Topic | Type | Triggered by | Effect |
|-------|------|-------------|--------|
| `/cmd_vel` | `Twist` | Teleop keys (WASD) | Robot moves at 10 Hz while key held |
| `/nav_goal` | `PoseStamped` | Click map in Goal mode | nav_goal_proxy → Nav2 NavigateToPose |
| `/initialpose` | `PoseWithCovarianceStamped` | Click map in Pose mode | AMCL reinitializes particle filter |
| `/save_map` | `Empty` | Save Map button | map_saver_node saves .pgm + .yaml |

---

## Nodes launched by `dashboard.launch.py`

| Node | Package | Role |
|------|---------|------|
| `dashboard_node` | esibot_ui | HTTP server (port 8080), serves the React build |
| `rosbridge_websocket` | rosbridge_server | WebSocket bridge ROS2 ↔ browser (port 9090) |
| `nav_goal_proxy` | esibot_ui | `/nav_goal` topic → `NavigateToPose` action → publishes status on `/nav_goal_status` |
| `map_saver_node` | esibot_ui | `/save_map` topic → runs `map_saver_cli` → publishes status on `/save_map_status` |

### Why nav_goal_proxy?

The browser cannot call a ROS2 action directly (too complex for rosbridge). The proxy bridges the gap:

```
Dashboard  →  /nav_goal (PoseStamped)  →  nav_goal_proxy  →  NavigateToPose action  →  Nav2
Dashboard  ←  /nav_goal_status (String) ←─────────────────┘
               (sending → navigating → reached / error)
```

### Why the cmd_vel relay?

Nav2 publishes on `/cmd_vel_nav`. The velocity_smoother smooths and outputs `/cmd_vel_smoothed`. But the driver listens on `/cmd_vel`. A relay node bridges the gap:

```
Nav2 → /cmd_vel_nav → velocity_smoother → /cmd_vel_smoothed → relay → /cmd_vel → driver
```

This relay must be launched separately: `ros2 run topic_tools relay /cmd_vel_smoothed /cmd_vel`

---

## Interface Panels

| Panel | Description | Data source |
|-------|-------------|-------------|
| **Connection** | WebSocket URL, connect/disconnect, latency | Direct `ROSLIB.Ros` connection |
| **Map** | SLAM map + robot position + path + scan overlay + goal/pose click | `/map`, `/tf`, `/odom`, `/scan` |
| **Camera** | Raw or annotated video feed | `/camera/compressed` or `/camera/image_annotated` |
| **Teleop** | WASD / arrow keys + on-screen buttons | Publishes `/cmd_vel` at 10 Hz |
| **Battery** | Level bar (green/orange/red) + percentage | `/battery_state` |
| **Servo Gauge** | Semi-circular gauge showing radar servo angle | `/esibot/servo_angle` |

Map controls: scroll = zoom, drag = pan, ⊙ = recenter, ⊕ = initial pose, ⚑ = send goal, 💾 = save map.

---

## Build & Deploy

```bash
# 1. Build the React dashboard
cd ~/esibot_ws/src/dashboard
npm install && npm run build

# 2. Copy to ROS2 package
rm -rf ../esibot_ui/web/
cp -r dist/ ../esibot_ui/web/

# 3. Rebuild ROS2 package
cd ~/esibot_ws
colcon build --packages-select esibot_ui
source install/setup.bash

# 4. Launch
ros2 launch esibot_ui dashboard.launch.py
```

Access: `http://<robot-ip>:8080`

---

## Configuration

File: `dashboard/src/config.js`

| Parameter | Default | Description |
|-----------|---------|-------------|
| `ROSBRIDGE_URL` | `ws://localhost:9090` | WebSocket endpoint |
| `CMD_VEL.LINEAR_SPEED` | `0.4` | Teleop linear speed (m/s) |
| `CMD_VEL.ANGULAR_SPEED` | `1.5` | Teleop angular speed (rad/s) |
| `SCAN_OVERLAY` | `true` | Show laser scan rays on map |

---

## Quick Diagnostic

```bash
ros2 topic list | grep -E 'map|odom|scan|cmd_vel|camera|battery'
ros2 topic hz /scan         # ~0.33 Hz (HC-SR04 sweep)
ros2 topic hz /cmd_vel      # should show frequency during nav/teleop
ros2 node list              # all active nodes
```
