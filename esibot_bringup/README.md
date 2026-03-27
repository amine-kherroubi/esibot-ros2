# esibot_bringup

ROS 2 Jazzy bringup for the EsiBot base. This package runs `esibot_driver`, which bridges the ESP32 motor/encoder controller to ROS 2 by subscribing to velocity commands and publishing odometry, TF, and battery state. An optional keyboard teleop node is included for quick testing.

## Build

```bash
cd ~/robot_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-select esibot_bringup
source ~/robot_ws/install/setup.bash
```

## Run

Default (no teleop):

```bash
ros2 launch esibot_bringup bringup.launch.py
```

With hardware attached:

```bash
ros2 launch esibot_bringup bringup.launch.py serial_port:=/dev/ttyUSB0
```

Enable keyboard teleop:

```bash
ros2 launch esibot_bringup bringup.launch.py use_teleop:=true
```

Run the node directly:

```bash
ros2 run esibot_bringup esibot_driver
```

If the serial port is not available, the driver runs in open-loop mode and integrates the last `cmd_vel` command to publish odometry. This is expected for development without hardware.

## Launch Arguments

- `params_file`: full path to the driver parameters file
- `serial_port`: UART device connected to the ESP32
- `baud_rate`: UART baud rate (must match firmware)
- `cmd_vel_topic`: velocity command topic for the driver and teleop
- `use_sim_time`: use `/clock` if `true`
- `use_teleop`: launch `teleop_twist_keyboard` in a new terminal
- `teleop_prefix`: terminal prefix for teleop (default `xterm -e`)

## Parameters

Parameters live in `config/driver_params.yaml` and can be overridden from the launch file or CLI.

| Parameter | Default | Description |
| --- | --- | --- |
| `serial_port` | `/dev/ttyUSB0` | UART device connected to the ESP32 |
| `baud_rate` | `115200` | UART baud rate |
| `serial_timeout` | `0.1` | Read timeout (seconds) |
| `wheel_base` | `0.16` | Distance between wheels (meters) |
| `wheel_radius` | `0.033` | Wheel radius (meters) |
| `encoder_ticks_per_rev` | `330` | Encoder ticks per wheel revolution |
| `odom_frame` | `odom` | Odometry frame ID |
| `base_frame` | `base_footprint` | Base frame ID (must match URDF) |
| `odom_topic` | `odom` | Odometry topic |
| `cmd_vel_topic` | `cmd_vel` | Velocity command topic |
| `battery_topic` | `battery_state` | Battery state topic |
| `publish_rate` | `20.0` | Publish/update rate (Hz) |
| `publish_tf` | `true` | Broadcast `odom -> base_frame` TF |
| `cmd_vel_timeout` | `0.5` | Stop the robot if no `cmd_vel` arrives within this many seconds (`0.0` disables) |
| `reconnect_on_error` | `true` | Attempt to reconnect the serial port after errors |
| `reconnect_interval` | `2.0` | Minimum seconds between reconnect attempts |

## Topics

| Topic | Type | Description |
| --- | --- | --- |
| `odom` | `nav_msgs/Odometry` | Odometry estimate |
| `tf` | `tf2_msgs/TFMessage` | `odom -> base_frame` transform |
| `battery_state` | `sensor_msgs/BatteryState` | Battery voltage |
| `cmd_vel` | `geometry_msgs/Twist` | Velocity command input |

## Nav2 Compatibility

This package is aligned with the Nav2 configuration in `esibot_navigation`:

1. Frames assume `map -> odom -> base_footprint`.
2. `odom` and `cmd_vel` are the default topics used by Nav2.
3. If you change `base_frame`, update `base_frame_id` and `robot_base_frame` in `config/nav2_params.yaml`.

## Serial Protocol

| Direction | Format | Example |
| --- | --- | --- |
| ESP32 -> host | `ENC:<left_ticks>,<right_ticks>,<voltage>` | `ENC:1234,1236,7.8` |
| host -> ESP32 | `CMD:<v_right>,<v_left>` | `CMD:0.300,-0.280` |

The ESP32 firmware must follow this format exactly.
