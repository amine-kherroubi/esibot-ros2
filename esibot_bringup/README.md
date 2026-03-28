# esibot_bringup

ROS 2 Jazzy bringup package for the EsiBot base. This package runs `esibot_driver`, which bridges the ESP32 motor/encoder controller to ROS 2 by subscribing to velocity commands and publishing odometry, TF, and battery state.

---

## 1. Package Overview

```
esibot_bringup/
├── config/
│   └── driver_params.yaml       ← all tunable parameters
├── launch/
│   └── bringup.launch.py        ← main launch file
└── esibot_bringup/
    └── esibot_driver.py         ← driver node
```

**What the driver does:**

- Reads encoder ticks and battery voltage from the ESP32 over UART (`ENC:` protocol)
- Integrates differential-drive kinematics into an odometry estimate (Runge-Kutta 2nd order)
- Publishes `/odom`, `/tf` (`odom → base_footprint`), and `/battery_state`
- Forwards velocity commands from `/cmd_vel` to the ESP32 (`CMD:` protocol)
- Safety: stops the robot automatically if no `/cmd_vel` arrives within `cmd_vel_timeout` seconds
- Sim mode: works without any hardware by integrating `/cmd_vel` directly into odometry

---

## 2. Build

```bash
cd ~/robot_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-select esibot_bringup
source ~/robot_ws/install/setup.bash
```

---

## 3. Quick Start

### Real hardware (ESP32 connected via USB)

```bash
ros2 launch esibot_bringup bringup.launch.py
```

### Custom serial port

```bash
ros2 launch esibot_bringup bringup.launch.py serial_port:=/dev/ttyUSB1
```

### Simulation mode (no ESP32 needed)

```bash
ros2 launch esibot_bringup bringup.launch.py sim_mode:=true
```

Then send velocity commands from Foxglove (see [Section 8]).

### Gazebo integration (sim time + sim mode)

```bash
ros2 launch esibot_bringup bringup.launch.py sim_mode:=true use_sim_time:=true
```

> **Note:** `teleop_twist_keyboard` requires an interactive TTY and cannot be launched

---

## 4. Launch Arguments

| Argument | Default | Description |
|---|---|---|
| `params_file` | `config/driver_params.yaml` | Full path to the parameters file |
| `serial_port` | `/dev/ttyUSB0` | UART device connected to the ESP32 |
| `baud_rate` | `115200` | UART baud rate — must match ESP32 firmware |
| `use_sim_time` | `false` | Use `/clock` topic (set `true` when running Gazebo) |
| `cmd_vel_topic` | `cmd_vel` | Velocity command topic for driver and teleop |
| `sim_mode` | `false` | Skip serial, integrate cmd_vel into odometry directly |
| `use_teleop` | `false` | Launch `teleop_twist_keyboard` (requires real TTY — see note above) |

---

## 5. Parameters

All parameters live in `config/driver_params.yaml` and can be overridden from the launch file or CLI.

### Serial

| Parameter | Default | Description |
|---|---|---|
| `serial_port` | `/dev/ttyUSB0` | UART device connected to the ESP32 |
| `baud_rate` | `115200` | UART baud rate |
| `serial_timeout` | `0.03` | Serial read timeout in seconds. **Must be less than the publish period** (`1 / publish_rate = 0.05 s`). If set higher, the timer callback will stall and the effective publish rate drops below 20 Hz. |

### Robot Geometry

> Measure your physical robot and update these. Wrong values = wrong odometry.

| Parameter | Default | Description |
|---|---|---|
| `wheel_base` | `0.138` | Distance between wheel centres (metres) — error here scales all rotational odometry |
| `wheel_radius` | `0.033` | Wheel radius (metres) — error here scales all linear odometry |
| `encoder_ticks_per_rev` | `330` | Encoder ticks per full wheel revolution — verify by rotating one wheel exactly one turn and checking `/odom` advances by `2π × 0.033 ≈ 0.207 m` |

### Frames

> Must match the URDF defined in `esibot_description`.

| Parameter | Default | Description |
|---|---|---|
| `odom_frame` | `odom` | Odometry frame ID |
| `base_frame` | `base_footprint` | Robot root frame (child of `odom` in TF) |

### Topics

| Parameter | Default | Description |
|---|---|---|
| `odom_topic` | `odom` | Odometry output topic |
| `cmd_vel_topic` | `cmd_vel` | Velocity command input topic |
| `battery_topic` | `battery_state` | Battery voltage output topic |

### Publishing

| Parameter | Default | Description |
|---|---|---|
| `publish_rate` | `20.0` | Timer rate in Hz for odometry, TF, and battery |
| `publish_tf` | `true` | Whether to broadcast `odom → base_frame` TF |

### Safety & Robustness

| Parameter | Default | Description |
|---|---|---|
| `cmd_vel_timeout` | `0.5` | Seconds without a `cmd_vel` before the robot is stopped. Set to `0.0` to disable. |
| `reconnect_on_error` | `true` | Attempt to reconnect the serial port after a failure |
| `reconnect_interval` | `2.0` | Minimum seconds between reconnect attempts |

### Velocity Limits

These are **hardcoded in `esibot_driver.py`** (not in the YAML) — adjust them to match your motor specs:

```python
MAX_LINEAR_VEL  = 0.3   # m/s  — ±0.3 m/s max forward/backward
MAX_ANGULAR_VEL = 2.0   # rad/s — ±2.0 rad/s max rotation
```

All incoming `cmd_vel` commands are clamped to these limits before being forwarded to the ESP32.

---

## 6. Topics & TF

### Published

| Topic | Type | Description |
|---|---|---|
| `/odom` | `nav_msgs/Odometry` | Pose and velocity estimate from encoder integration |
| `/tf` | `tf2_msgs/TFMessage` | `odom → base_footprint` dynamic transform |
| `/battery_state` | `sensor_msgs/BatteryState` | Battery voltage from ESP32 |

### Subscribed

| Topic | Type | Description |
|---|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` | Velocity commands. QoS: **RELIABLE, KEEP_LAST, depth=1** — only the most recent command is ever queued |

---

## 8. Visualization (Foxglove)

### Sending cmd_vel from the Publish panel

1. Add a **Publish** panel
2. Set topic: `/cmd_vel`
3. Set schema: `geometry_msgs/Twist`
4. Use this template:

```json
{
  "linear":  {"x": 0.2, "y": 0.0, "z": 0.0},
  "angular": {"x": 0.0, "y": 0.0, "z": 0.0}
}
```

| Field | Effect |
|---|---|
| `linear.x > 0` | Move forward |
| `linear.x < 0` | Move backward |
| `angular.z > 0` | Rotate left (counter-clockwise) |
| `angular.z < 0` | Rotate right (clockwise) |

Or use the **Joystick** panel — set its publish topic to `/cmd_vel` and drag to drive continuously.

---
### ESP32 → ROS 2 (encoder data)

```
ENC:<left_ticks>,<right_ticks>[,<voltage>]\n
```

| Field | Type | Description |
|---|---|---|
| `left_ticks` | `int` | Cumulative left encoder tick count |
| `right_ticks` | `int` | Cumulative right encoder tick count |
| `voltage` | `float` (optional) | Battery voltage in volts |

**Example:** `ENC:1234,1236,7.8`

### ROS 2 → ESP32 (motor command)

```
CMD:<v_right>,<v_left>\n
```

| Field | Type | Description |
|---|---|---|
| `v_right` | `float` | Right wheel velocity in m/s |
| `v_left` | `float` | Left wheel velocity in m/s |

**Example:** `CMD:0.300,-0.280`
---
