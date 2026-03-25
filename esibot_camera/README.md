#  esibot_camera

Package ROS2 — **Tâche ** : Stream vidéo ESP32-CAM → ROS2
| ROS2 Jazzy | Ubuntu 24.04

---

##  Description

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

##  Prérequis

- Ubuntu 24.04
- ROS2 Jazzy
- Python 3.12+

---

##  Installation

### 1. Installer les dépendances

```bash
cd ~/robot_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

### 2. Compiler

```bash
cd ~/robot_ws
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
ros2 launch esibot_camera camera.launch.py esp32_ip:=<YOUR_ESP32_IP>
```
Example:
```bash
ros2 launch esibot_camera camera.launch.py esp32_ip:=192.168.1.45
```

# Option 2: Simulation / Mock Server (No Hardware) :
start the visualization bridge:
 ```bash
 ros2 launch esibot_description display.launch.py
 ```

Run the Fake Server :
```bash
python3 fake_esp32_server.py
```

Run the ROS 2 Node  :
Open a second terminal and point the camera node to your localhost IP (127.0.0.1) and the server port (8080):
```bash
ros2 launch esibot_camera camera.launch.py esp32_ip:=127.0.0.1 esp32_port:=8080
```

---

##  other Visualisation options

**Option 1 — Navigateur** : ouvrir directement `http://192.168.1.80/stream`

**Option 2 — rqt** :
```bash
ros2 run rqt_image_view rqt_image_view
```
Sélectionner le topic `/camera/image_raw`


---

##  Structure du package

```
esibot_camera/
├── CMakeLists.txt
├── package.xml
├── README.md
├── esibot_camera/
│   ├── __init__.py
│   └── camera_stream_node.py    ← nœud principal
├── launch/
│   └── camera.launch.py
└── config/
    └── camera_params.yaml       ← configuration par défaut
```

---

##  Topics publiés

| Topic                 | Type                         | Description              |
| --------------------- | ---------------------------- | ------------------------ |
| `/camera/image_raw`   | `sensor_msgs/msg/Image`      | Image BGR 320×240        |
| `/camera/camera_info` | `sensor_msgs/msg/CameraInfo` | Calibration caméra       |
| `/camera/status`      | `std_msgs/msg/String`        | CONNECTED / DISCONNECTED |
