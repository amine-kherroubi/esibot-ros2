# esibot_camera

Package ROS2 — **Tâche ** : Stream vidéo ESP32-CAM → ROS2
| ROS2 Jazzy | Ubuntu 24.04

---

## Prérequis

Ce package capture le flux vidéo MJPEG d'une **ESP32-CAM** via WiFi (HTTP),
applique un traitement **OpenCV** (resize),
et publie les images sur le topic ROS2 `/camera/image_raw`.

It supports two operation modes:

Hardware Mode: Connects to a real ESP32-CAM on the local network.
Simulation Mode: Connects to a local mock server for testing without physical hardware.

```
ESP32-CAM (WiFi)
      │  HTTP MJPEG  http://192.168.1.80/stream
      ▼
esibot_camera node  (OpenCV traitement)
      │
      ├──► /camera/image_raw    (sensor_msgs/Image)
      ├──► /camera/camera_info  (sensor_msgs/CameraInfo)
      └──► /camera/status       (std_msgs/String)
```

---

## Installation

```bash
cd ~/esibot_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-select esibot_camera
source install/setup.bash
```

---

##  Configuration

Les paramètres du package sont dans `config/camera_params.yaml` :

```yaml
esibot_camera_node:
  ros__parameters:
    esp32_ip:        '192.168.1.80'   # ← IP de l'ESP32-CAM
    esp32_port:      80
    stream_path:     '/stream'
    frame_width:     320
    frame_height:    240
    publish_rate:    10.0
    show_fps:        false
    reconnect_delay: 3.0
    camera_frame:    'camera_optical_frame'
```

> Modifier `esp32_ip` selon l'adresse IP de ESP32-CAM ou du PC qui diffuse flux vidéo sur le réseau.
> Tous les autres paramètres ont des valeurs par défaut prêtes à l'emploi.

| Paramètre         | Défaut         | Description                    |
| ----------------- | -------------- | ------------------------------ |
| `esp32_ip`        | `192.168.1.80` | Adresse IP de l'ESP32-CAM      |
| `esp32_port`      | `80`           | Port HTTP de l'ESP32           |
| `stream_path`     | `/stream`      | Chemin URL du flux MJPEG       |
| `frame_width`     | `320`          | Largeur image après traitement |
| `frame_height`    | `240`          | Hauteur image après traitement |
| `publish_rate`    | `10.0`         | Fréquence publication (Hz)     |
| `show_fps`        | `true`         | Afficher FPS sur l'image       |
| `reconnect_delay` | `3.0`          | Délai reconnexion (secondes)   |

---

##  Lancement
# Option 1: Real Hardware
Use this when you have the ESP32-CAM connected to WiFi

Command:
```bash
ros2 launch esibot_camera esibot_camera.launch.py esp32_ip:=192.168.1.80
```

Démarre :
- `camera_stream_node` — stream MJPEG ESP32-CAM → `/camera/image_raw`
- `camera_compressed_republisher` — `/camera/image_raw` → `/camera/compressed`

---

##  other Visualisation options

```bash
ros2 run rqt_image_view rqt_image_view /camera/image_raw
```

---

## Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/camera/image_raw` | `sensor_msgs/Image` | Image brute 320×240 |
| `/camera/compressed` | `sensor_msgs/CompressedImage` | Image JPEG compressée |
| `/camera/camera_info` | `sensor_msgs/CameraInfo` | Calibration caméra |
| `/camera/status` | `std_msgs/String` | CONNECTED / DISCONNECTED |

---

## Configuration — `config/camera_params.yaml`

| Paramètre | Défaut | Description |
|-----------|--------|-------------|
| `esp32_ip` | `192.168.1.80` | IP de l'ESP32-CAM |
| `publish_rate` | `30.0` | Fréquence publication (Hz) |
| `frame_width` | `320` | Largeur image |
| `frame_height` | `240` | Hauteur image |
| `show_fps` | `true` | Afficher FPS sur l'image raw |
| `reconnect_delay` | `3.0` | Délai reconnexion (secondes) |

---

## Structure

```
esibot_camera/
├── esibot_camera/
│   └── camera_stream_node.py   ← stream MJPEG ESP32-CAM
├── launch/
│   ├── esibot_camera.launch.py ← caméra + compressed (recommandé)
│   └── camera.launch.py        ← caméra seule
└── config/
    └── camera_params.yaml
```
