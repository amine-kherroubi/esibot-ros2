# esibot_camera

Package ROS2 — Stream vidéo ESP32-CAM → ROS2
| ROS2 Jazzy | Ubuntu 24.04

---

## Description

Ce package capture le flux vidéo MJPEG d'une **ESP32-CAM** via WiFi (HTTP),
applique un traitement **OpenCV** (resize), et publie les images sur les topics ROS2.

Deux modes de fonctionnement sont supportés :

- **Hardware mode** — connexion réelle à l'ESP32-CAM sur le réseau local.
- **Simulation mode** — génère des frames synthétiques sans matériel physique (`sim_mode: true`).

```
ESP32-CAM (WiFi)
      │  HTTP MJPEG  http://192.168.1.80/stream
      ▼
esibot_camera_node  (OpenCV resize)
      │
      ├──► /camera/image_raw    (sensor_msgs/Image)
      ├──► /camera/camera_info  (sensor_msgs/CameraInfo)
      ├──► /camera/compressed   (sensor_msgs/CompressedImage)
      └──► /camera/status       (std_msgs/String)
```

---

## Installation

```bash
cd ~/esibot_ws
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
│   └── camera_stream_node.py      ← nœud principal MJPEG → ROS2
├── launch/
│   └── esibot_camera.launch.py    ← caméra + republication compressée
└── config/
    └── camera_params.yaml         ← paramètres par défaut
```

---

## Configuration — `config/camera_params.yaml`

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
    camera_frame:    "camera_optical_frame"
    sim_mode:        false
```


| Paramètre         | Défaut              | Description                                          |
|-------------------|---------------------|------------------------------------------------------|
| `esp32_ip`        | `192.168.1.80`      | Adresse IP de l'ESP32-CAM                            |
| `esp32_port`      | `80`                | Port HTTP de l'ESP32                                 |
| `stream_path`     | `/stream`           | Chemin URL du flux MJPEG                             |
| `frame_width`     | `320`               | Largeur image après traitement (px)                  |
| `frame_height`    | `240`               | Hauteur image après traitement (px)                  |
| `publish_rate`    | `10.0`              | Fréquence de publication (Hz) — max 15 Hz sur WiFi   |
| `show_fps`        | `true`              | Overlay FPS sur l'image (désactivé par défaut)       |
| `reconnect_delay` | `3.0`               | Délai avant tentative de reconnexion (secondes)      |
| `camera_frame`    | `camera_optical_frame` | frame_id TF — doit correspondre au lien URDF      |
| `sim_mode`        | `false`             | `true` → frames synthétiques sans ESP32-CAM          |

---

## Lancement

### Mode matériel — ESP32-CAM connectée au WiFi

```bash
ros2 launch esibot_camera esibot_camera.launch.py esp32_ip:=192.168.1.80
```

### Mode simulation — sans matériel

```bash
ros2 launch esibot_camera camera.launch.py sim_mode:=true
```

---

## Topics publiés

| Topic                 | Type                          | Description                              |
|-----------------------|-------------------------------|------------------------------------------|
| `/camera/image_raw`   | `sensor_msgs/Image`           | Image BGR brute 320×240 à 10 Hz          |
| `/camera/camera_info` | `sensor_msgs/CameraInfo`      | Intrinsèques caméra (placeholder)        |
| `/camera/compressed`  | `sensor_msgs/CompressedImage` | Image JPEG compressée (via republish)    |
| `/camera/status`      | `std_msgs/String`             | Heartbeat 1 Hz : `CONNECTED`, `DISCONNECTED`, `SIM_OK frames=N` |

---

## Visualisation

```bash
ros2 run rqt_image_view rqt_image_view /camera/image_raw
```

Vérifier les topics actifs :

```bash
ros2 topic list | grep camera
ros2 topic echo /camera/status
ros2 topic hz /camera/image_raw
```

---
