# esibot_sensors

ROS 2 package - Pseudo-LiDAR (HC-SR04 + SG90 servo)
ROS 2 Jazzy | Ubuntu 24.04

---

## What this package does

Sweeps an SG90 servo from -90 deg to +90 deg (centered on the robot's forward axis),
measures distance at each step with an HC-SR04 ultrasonic sensor, and publishes
the result as a `sensor_msgs/LaserScan` on `/scan` - the same format slam_toolbox
and Nav2 expect from a real LiDAR.

It also publishes the current servo angle on `/joint_states` at each step so the
TF tree (`servo_link`) stays accurate during the sweep.

Runs in **simulation mode** automatically when `RPi.GPIO` is not available (PC / WSL2).

---

## Package structure

```
esibot_sensors/
├── esibot_sensors/
│   ├── __init__.py
│   └── radar_node.py       ← main node
├── launch/
│   └── radar.launch.py
├── resource/
│   └── esibot_sensors
├── package.xml
├── setup.py
├── setup.cfg
└── README.md
```

---

## Prerequisites

- ROS 2 Jazzy
- Python 3

On Raspberry Pi (real hardware only):
```bash
pip install RPi.GPIO
```

---

## Build

```bash
cd ~/robot_ws
colcon build --symlink-install --packages-select esibot_sensors
source install/setup.bash
```

> After editing `radar_node.py`, rebuild before running — the node
> executes from the build directory, not the source directory.

---

## Launch

### Default launch
```bash
ros2 launch esibot_sensors radar.launch.py
```

### Override GPIO pins if your wiring differs
```bash
ros2 launch esibot_sensors radar.launch.py \
    servo_pin:=17 trig_pin:=27 echo_pin:=22
```

### Change sweep period (seconds)
```bash
ros2 launch esibot_sensors radar.launch.py sweep_period:=4.0
```

> `sweep_period` must be longer than the worst-case sweep duration.
> 19 steps x 110 ms (real hardware) ~ 2.1 s. Default 3.0 s gives margin.

---

## Launch parameters

| Parameter      | Default | Description                            |
| -------------- | ------- | -------------------------------------- |
| `servo_pin`    | 17      | BCM GPIO pin for SG90 signal           |
| `trig_pin`     | 27      | BCM GPIO pin for HC-SR04 TRIG          |
| `echo_pin`     | 22      | BCM GPIO pin for HC-SR04 ECHO          |
| `sweep_period` | 3.0     | Seconds between sweeps                 |
| `sim_mode`     | false   | Force simulation mode on real hardware |

---

## Wiring (Raspberry Pi)

| Component    | Pin                           |
| ------------ | ----------------------------- |
| Servo signal | GPIO 17                       |
| HC-SR04 TRIG | GPIO 27                       |
| HC-SR04 ECHO | GPIO 22 (via voltage divider) |
| VCC          | 5V                            |
| GND          | GND                           |

---

## Topics published

| Topic           | Type                     | Rate      | Description          |
| --------------- | ------------------------ | --------- | -------------------- |
| `/scan`         | `sensor_msgs/LaserScan`  | ~0.33 Hz  | Full sweep result    |
| `/joint_states` | `sensor_msgs/JointState` | 19x/sweep | Servo angle per step |

### /scan field values

| Field              | Value                           |
| ------------------ | ------------------------------- |
| `frame_id`         | `laser_link`                    |
| `angle_min`        | -pi/2 (-90 deg, faces right)    |
| `angle_max`        | +pi/2 (+90 deg, faces left)     |
| `angle_increment`  | 0.1745 rad (10°)                |
| `range_min`        | 0.02 m                          |
| `range_max`        | 4.0 m                           |
| Out-of-range value | `range_max + 1.0` (per REP-117) |

---

## Required nodes to run alongside

This node only publishes `/scan` and `/joint_states`.
It requires `robot_state_publisher` to be running (started by the display launch)
so that `/joint_states` is consumed and `/tf` is updated.

```bash
# Terminal 1
ros2 launch esibot_description display.launch.py

# Terminal 2
ros2 launch esibot_sensors radar.launch.py
```

---

## Verify it is working

```bash
# Check topics are live
ros2 topic list

# Check scan data
ros2 topic echo /scan

# Check servo angle is sweeping
ros2 topic echo /joint_states

# Check TF chain is complete
ros2 run tf2_tools view_frames
```

Expected `/scan` output:
- `frame_id: laser_link`
- 19 values in `ranges`
- all out-of-range readings = 5.0 (range_max + 1.0), never 0.0

Expected `/joint_states` output:
- `name: [servo_joint]`
- `position` stepping from -1.5707 to +1.5707 across 19 messages per sweep

Expected TF tree (from `view_frames`):
```
base_footprint -> base_link -> upper_plate -> servo_base -> servo_link -> laser_link
```
