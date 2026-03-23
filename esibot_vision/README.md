# esibot_vision

Package ROS2 Jazzy | Ubuntu 24.04

**Tâche 4.1** — Vision temps réel : détection ligne au sol + détection obstacles, annotation image, publication topics `/vision/*`

---

## Prérequis

- Ubuntu 24.04 + ROS2 Jazzy
- Package `esibot_camera` lancé (fournit `/camera/image_raw`)

---

## Installation

```bash
cd ~/esibot_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-select esibot_vision
source install/setup.bash
```

---

## Lancement

**Terminal 1 — caméra (requis) :**
```bash
ros2 launch esibot_camera esibot_camera.launch.py esp32_ip:=192.168.1.80
```

**Terminal 2 — vision :**
```bash
ros2 launch esibot_vision vision.launch.py
```

---

## Voir le flux annoté

```bash
ros2 run rqt_image_view rqt_image_view /camera/image_annotated
```

---

## Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/camera/image_raw` | `sensor_msgs/Image` | Souscrit — source image |
| `/camera/image_annotated` | `sensor_msgs/Image` | Image avec ROI + contours |
| `/vision/line_position` | `std_msgs/Float32` | Position ligne : -1.0 (gauche) … +1.0 (droite) |
| `/vision/obstacle_detected` | `std_msgs/Bool` | True si obstacle détecté |
| `/vision/detections` | `std_msgs/String` | JSON complet des détections |

---

## Configuration — `config/vision_params.yaml`

| Paramètre | Défaut | Description |
|-----------|--------|-------------|
| `line_roi_ratio` | `0.55` | Zone ligne = 55% bas de l'image |
| `line_threshold` | `60` | Seuil binarisation |
| `line_color` | `dark` | `dark` = ligne noire sur fond clair |
| `obstacle_roi_ratio` | `0.40` | Zone obstacle = 40% haut de l'image |
| `obstacle_min_area` | `1500` | Aire minimale obstacle (px²) |
| `publish_annotated` | `true` | Publier image annotée |

---

## Structure

```
esibot_vision/
├── esibot_vision/
│   └── vision_node.py    ← détection ligne + obstacles
├── launch/
│   └── vision.launch.py
└── config/
    └── vision_params.yaml
```
