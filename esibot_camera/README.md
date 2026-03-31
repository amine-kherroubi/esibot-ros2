# esibot_camera

ROS 2 package - ESP32-CAM video stream to ROS 2
ROS 2 Jazzy | Ubuntu 24.04

---

## Description

This package captures the MJPEG video stream from an **ESP32-CAM** over Wi-Fi (HTTP),
applies OpenCV processing (resize), and publishes images to ROS 2 topics.

Two operating modes are supported:

- **Hardware mode** - real connection to the ESP32-CAM on the local network.
- **Simulation mode** - generates synthetic frames without physical hardware (`sim_mode: true`).

```
ESP32-CAM (Wi-Fi)
      |  HTTP MJPEG  http://192.168.1.80/stream
      v
esibot_camera_node  (OpenCV resize)
      |
      |--> /camera/image_raw    (sensor_msgs/Image)
      |--> /camera/camera_info  (sensor_msgs/CameraInfo)
      |--> /camera/compressed   (sensor_msgs/CompressedImage)
      `--> /camera/status       (std_msgs/String)
```

---

## Installation

```bash
cd ~/robot_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-select esibot_camera
source ~/.bashrc
```

---

## Structure

```
esibot_camera/
├── esibot_camera/
│   └── camera_stream_node.py      <- main MJPEG -> ROS 2 node
├── launch/
│   └── camera.launch.py           <- camera node + compressed republish
└── config/
    └── camera_params.yaml         <- default parameters
```

---

## Configuration - `config/camera_params.yaml`

```yaml
esibot_camera_node:
  ros__parameters:
    esp32_ip:        "192.168.1.80"
    esp32_port:      80
    stream_path:     "/stream"
    frame_width:     320
    frame_height:    240
    publish_rate:    10.0
    show_fps:        true
    reconnect_delay: 3.0
    camera_frame:    "camera_link"
    sim_mode:        false
```

| Parameter         | Default        | Description                                  |
| ----------------- | -------------- | -------------------------------------------- |
| `esp32_ip`        | `192.168.1.80` | ESP32-CAM IP address                         |
| `esp32_port`      | `80`           | ESP32-CAM HTTP port                          |
| `stream_path`     | `/stream`      | MJPEG stream URL path                        |
| `frame_width`     | `320`          | Output image width (px)                      |
| `frame_height`    | `240`          | Output image height (px)                     |
| `publish_rate`    | `10.0`         | Publish rate (Hz) - max ~15 Hz over Wi-Fi    |
| `show_fps`        | `true`         | Draw an FPS overlay on the image             |
| `reconnect_delay` | `3.0`          | Delay before reconnect attempts (seconds)    |
| `camera_frame`    | `camera_link`  | TF frame_id - must match the URDF link       |
| `sim_mode`        | `false`        | `true` -> synthetic frames without ESP32-CAM |

---

## Launch

### Hardware mode - ESP32-CAM on Wi-Fi

```bash
ros2 launch esibot_camera camera.launch.py esp32_ip:=192.168.1.80
```

### Simulation mode - no hardware

```bash
ros2 launch esibot_camera camera.launch.py sim_mode:=true
```

---

## Published topics

| Topic                 | Type                          | Description                                                    |
| --------------------- | ----------------------------- | -------------------------------------------------------------- |
| `/camera/image_raw`   | `sensor_msgs/Image`           | Raw BGR image 320x240 at 10 Hz                                 |
| `/camera/camera_info` | `sensor_msgs/CameraInfo`      | Camera intrinsics (placeholder)                                |
| `/camera/compressed`  | `sensor_msgs/CompressedImage` | JPEG image (via republish)                                     |
| `/camera/status`      | `std_msgs/String`             | 1 Hz heartbeat: `CONNECTED`, `DISCONNECTED`, `SIM_OK frames=N` |

---

## Visualization

```bash
ros2 run rqt_image_view rqt_image_view /camera/image_raw
```

Verify active topics:

```bash
ros2 topic list | grep camera
ros2 topic echo /camera/status
ros2 topic hz /camera/image_raw
```

---
