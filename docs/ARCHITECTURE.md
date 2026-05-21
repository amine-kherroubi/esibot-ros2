# EsiBot — Documentation Technique Complète

> Projet robotique ROS 2 — Robot différentiel autonome basé Raspberry Pi 4

---

## Table des matières

1. [Vue d'ensemble du système](#1-vue-densemble-du-système)
2. [Architecture matérielle](#2-architecture-matérielle)
3. [Architecture logicielle](#3-architecture-logicielle)
4. [Packages ROS 2](#4-packages-ros-2)
   - [esibot_bringup](#41-esibot_bringup)
   - [esibot_sensors](#42-esibot_sensors)
   - [esibot_camera](#43-esibot_camera)
   - [esibot_description](#44-esibot_description)
   - [esibot_slam](#45-esibot_slam)
   - [esibot_navigation](#46-esibot_navigation)
   - [esibot_vision](#47-esibot_vision)
   - [esibot_ui](#48-esibot_ui)
   - [esibot_logging](#49-esibot_logging)
   - [esibot_gazebo](#410-esibot_gazebo)
   - [web_bridge](#411-web_bridge)
5. [Dashboard React](#5-dashboard-react)
6. [Communication réseau](#6-communication-réseau)
7. [Lancement du projet](#7-lancement-du-projet)
8. [Calibration et tuning](#8-calibration-et-tuning)
9. [Outils et scripts](#9-outils-et-scripts)
10. [Carte des topics ROS 2](#10-carte-des-topics-ros-2)

---

## 1. Vue d'ensemble du système

EsiBot est un robot mobile différentiel autonome conçu pour le SLAM, la navigation autonome et la détection visuelle (panneaux, voies, obstacles). Il tourne sous **ROS 2 Jazzy** sur un **Raspberry Pi 4** et communique via WiFi avec un dashboard web React.

```
┌──────────────────────────────────────────────────────────────────┐
│                   Navigateur Web (Dashboard React)               │
│                   http://<robot>:8080                            │
│        (connexion ROS via ws://<robot>:9090 rosbridge)           │
└─────────────────────────┬────────────────────────────────────────┘
                          │ WebSocket
             ┌────────────▼────────────────┐
             │   rosbridge_suite (port 9090)│
             │   rosapi (introspection)     │
             └────────────┬────────────────┘
                          │ Topics / Actions / Services ROS 2
┌─────────────────────────▼───────────────────────────────────────┐
│                  Raspberry Pi 4 — ROS 2 Jazzy                   │
│                                                                  │
│  esibot_driver ──────── /odom, /tf, /battery_state              │
│  radar_node ─────────── /scan, /joint_states                    │
│  camera_stream_node ─── /camera/image_raw                       │
│  vision_node ─────────── /esibot/lane_error, /esibot/signs      │
│  slam_toolbox ─────────── /map                                  │
│  nav2 ──────────────────── /navigate_to_pose                    │
│  dashboard_node ────────── HTTP :8080                           │
│                                                                  │
│  GPIO → L298N (moteurs), SG90 (servo), HC-SR04, encodeurs       │
│  UART /dev/ttyS0 → ESP32 (télémétrie batterie)                  │
│  WiFi → ESP32-CAM MJPEG (192.168.1.80:80)                       │
└──────────────────────────────────────────────────────────────────┘
```

---

## 2. Architecture matérielle

### Composants physiques

| Composant | Rôle | Interface |
|-----------|------|-----------|
| Raspberry Pi 4 (8 GB) | Cerveau principal, ROS 2 | — |
| L298N | Pont-H double pour 2 moteurs DC | GPIO5/6/13/26 (IN1-IN4), GPIO18/19 (PWM ENA/ENB) |
| Encodeurs (disque 20 trous) | Odométrie, 40 ticks/tour (both edges) | GPIO24 (gauche), GPIO17 (droite) |
| SG90 / MG996R | Rotation du capteur HC-SR04 | GPIO12 (DMA PWM pigpio 50 Hz) |
| HC-SR04 | Mesure ultrasonique, distance 0.20–4.00 m | TRIG=GPIO23, ECHO=GPIO25 (diviseur tension 2.78 V) |
| ESP32 | Télémétrie batterie (BAT:\<tension\>) | UART /dev/ttyS0, 115200 baud |
| ESP32-CAM | Flux MJPEG | WiFi HTTP 192.168.1.80:80/stream |

### Câblage servo → HC-SR04

Le servo SG90 est monté sur `upper_plate` (+0.08 m Z) et porte le HC-SR04. La rotation du servo couvre ±90° (−π à +π rad en ROS). Le signal servo est en DMA PWM pigpio à 50 Hz, largeur d'impulsion 1000–2000 µs.

**Convention d'angle :**
- Servo hardware : +180° = droite, 0° = avant, −180° = gauche
- ROS (`/scan`) : −π = gauche, 0 = avant, +π = droite
- Mapping : `ros_angle = −servo_angle`

### Pont-H L298N — logique de contrôle moteur

```
v > 0.05  →  forward  (IN_fwd=HIGH, IN_rev=LOW)
v < −0.05 →  backward (IN_fwd=LOW,  IN_rev=HIGH)
|v| ≤ 0.05 → stop    (IN_fwd=LOW,  IN_rev=LOW)
```

Vitesse PWM plafonnée à 55% (`MAX_PWM_DUTY`) par réglage empirique.

---

## 3. Architecture logicielle

### Distribution ROS 2

- **Distrib :** ROS 2 Jazzy
- **OS :** Ubuntu 24.04 LTS
- **Build :** colcon + rosdep
- **DDS :** CycloneDDS (défaut Jazzy)
- **Simulation :** Gazebo Harmonic (`gz sim`)

### Arbre TF

```
base_footprint
└── base_link               (fixed, +wheel_radius en Z)
    ├── left_wheel          (continuous)
    ├── right_wheel         (continuous)
    ├── caster_wheel        (fixed, avant)
    └── upper_plate         (fixed, +0.08 m Z)
        ├── servo_base      (fixed, corps SG90)
        │   └── servo_link  (revolute ±π, publié par /joint_states)
        │       └── laser_link  (frame_id de /scan)
        └── camera_link     (fixed, ESP32-CAM)
            └── camera_optical_frame  (convention Z-forward)
```

### Modes de lancement

| Mode | Description |
|------|-------------|
| `slam` | Cartographie SLAM en temps réel avec slam_toolbox |
| `nav` | Navigation autonome avec carte existante (Nav2 + AMCL) |
| `vision` | Pipeline détection voie/panneaux/obstacles (YOLOv8) |
| `sim_mode=true` | Simulation Gazebo, pas de GPIO ni serial |

---

## 4. Packages ROS 2

### 4.1 esibot_bringup

**Rôle :** Driver bas niveau — contrôle moteurs, lecture encodeurs, publication odométrie.

**Mainteneur :** Sarah Hasnaoui | `ament_python`

#### `esibot_bringup/esibot_driver.py`

**Classe :** `EsibotDriver(Node)`

**Paramètres physiques (constantes) :**

| Constante | Valeur | Description |
|-----------|--------|-------------|
| `WHEEL_BASE` | 0.16 m | Écartement roues (centre à centre) |
| `WHEEL_RADIUS` | 0.033 m | Rayon roue |
| `TICKS_PER_REV` | 40 | Ticks encodeur par tour (20 trous × 2 fronts) |
| `MAX_LINEAR_VEL` | 0.3 m/s | Saturation vitesse linéaire |
| `MAX_ANGULAR_VEL` | 2.0 rad/s | Saturation vitesse angulaire |
| `MAX_PWM_DUTY` | 55% | Plafond PWM moteur |

**GPIO :**
- Encodeurs : GPIO24 (gauche), GPIO17 (droite) — interruptions sur front montant/descendant
- L298N IN1–IN4 : GPIO5, GPIO6, GPIO13, GPIO26
- L298N ENA/ENB (PWM) : GPIO18, GPIO19 (pigpio hardware PWM)
- Servo : GPIO12 (50 Hz DMA PWM)

**Méthodes clés :**

- `_setup_gpio()` — Configure interruptions encodeur (pigpio callback)
- `_setup_motor_gpio()` — Configure 4 sorties direction + 2 sorties PWM
- `_cmd_vel_callback(msg)` — Reçoit `/cmd_vel`, calcule v_left/v_right par cinématique différentielle, appelle `_set_motor()`
- `_set_motor(v_left, v_right)` — Convertit vitesses en commandes GPIO (direction + PWM duty)
- `_integrate_encoders(dt)` — Odométrie fermée par Runge-Kutta 2ème ordre
- `_integrate_open_loop(dt)` — Fallback odométrie par intégration cmd_vel si encodeurs absents
- `_read_serial()` — Thread lecture UART ESP32, parse `BAT:<tension>`, publie `/battery_state`
- `_publish_odometry()` — Publie `nav_msgs/Odometry` sur `/odom`
- `_publish_tf()` — Publie `odom → base_footprint` dans `/tf`

**Topics publiés :**
- `/odom` — `nav_msgs/Odometry`, 20 Hz
- `/tf` — transform `odom → base_footprint`
- `/battery_state` — `sensor_msgs/BatteryState`

**Topics souscrits :**
- `/cmd_vel` — `geometry_msgs/Twist`

**Dégradation gracieuse :** Si pigpio/serial indisponible (ex. développement PC), le nœud publie une odométrie estimée depuis cmd_vel sans crash.

#### `launch/bringup.launch.py`

Arguments :
- `sim_mode` (défaut: false) — passe en odométrie open-loop, skip serial
- `use_sim_time` (défaut: false)
- `serial_port` (défaut: `/dev/ttyS0`)
- `baud_rate` (défaut: 115200)
- `use_teleop` — lance `teleop_twist_keyboard` en option

#### `config/driver_params.yaml`

```yaml
serial_port: /dev/ttyS0
baud_rate: 115200
odom_frame: odom
base_frame: base_footprint
publish_rate: 20.0
```

---

### 4.2 esibot_sensors

**Rôle :** Radar ultrasonique rotatif — pilotage servo SG90 + mesure HC-SR04 → publication `/scan`.

**Mainteneur :** Amira Bouderbala | `ament_python`

#### `esibot_sensors/radar_node.py`

**Classe :** `RadarNode(Node)` (aussi `EsibotSensors`)

**Paramètres ROS :**

| Paramètre | Défaut | Description |
|-----------|--------|-------------|
| `servo_pin` | 12 | GPIO pigpio DMA PWM |
| `trig_pin` | 23 | HC-SR04 TRIG |
| `echo_pin` | 25 | HC-SR04 ECHO |
| `sweep_steps` | 37 | Nombre de faisceaux par balayage |
| `angle_increment` | 5° | Résolution angulaire (0.0873 rad) |
| `servo_coeff` | 6.0 µs/° | Coefficient de calibration servo |
| `settle_ms` | 20 ms | Temps de stabilisation servo après mouvement |
| `median_reads` | 1 | Nombre de pings médianisés par faisceau |
| `sim_mode` | false | Mode simulation (GPIO désactivé) |

**Limites de portée :**
- `RANGE_MIN` = 0.20 m (filtre réflexions châssis 7–17 cm)
- `RANGE_MAX` = 4.00 m

**Hiérarchie d'initialisation matérielle :**
1. Tentative pigpio (DMA PWM, jitter < 5 µs) → préféré
2. Fallback RPi.GPIO (busy-wait, ~500 µs jitter)
3. Mode simulation si aucun GPIO disponible

**Méthodes clés :**

- `_init_hw()` — Initialise pigpio ou RPi.GPIO, configure servo PWM 50 Hz
- `_move_servo(angle_deg)` — Rampe 5 étapes vers angle cible (réduit les vibrations)
- `_set_servo_angle(angle_deg)` — Calcule largeur impulsion : `1500 + angle × (500/90)` µs (1000 µs = −90°, 2000 µs = +90°)
- `_ping_pigpio()` — Mesure temps echo via callback pigpio (son : 343 m/s)
- `_ping_gpio()` — Fallback RPi.GPIO busy-wait
- `_measure(angle_deg)` — Déplace servo + stabilisation + médiane de `median_reads` pings
- `_sweep_loop()` — Boucle infinie : balayage droite↔gauche alternés, publie `/scan` à chaque fin de balayage
- `_publish(ranges, direction)` — Construit et publie `sensor_msgs/LaserScan`

**Fréquence de publication `/scan` :** ~0.33 Hz (1 balayage toutes ~3 s avec 37 faisceaux × 50 ms)

**Topics publiés :**
- `/scan` — `sensor_msgs/LaserScan`
- `/joint_states` — `sensor_msgs/JointState` (position servo, 10 Hz, pour rviz)

#### `launch/radar.launch.py`

Arguments : `servo_pin`, `trig_pin`, `echo_pin`, `sweep_period` (défaut 8.0 s), `sim_mode`

---

### 4.3 esibot_camera

**Rôle :** Acquisition flux MJPEG depuis ESP32-CAM → publication topics ROS Image.

**Mainteneur :** Idriss Yacine Ziadi | `ament_python`

#### `esibot_camera/camera_stream_node.py`

**Classe :** `CameraStreamNode(Node)`

**Paramètres :**

| Paramètre | Défaut | Description |
|-----------|--------|-------------|
| `esp32_ip` | 192.168.1.80 | IP de l'ESP32-CAM |
| `esp32_port` | 80 | Port HTTP |
| `stream_path` | /stream | URL path MJPEG |
| `frame_width` | 320 | Largeur de redimensionnement |
| `frame_height` | 240 | Hauteur |
| `publish_rate` | 10.0 Hz | Fréquence publication |
| `reconnect_delay` | 3.0 s | Attente avant reconnexion |
| `sim_mode` | false | Frame synthétique (marquages voie + stop sign) |

**Pipeline :**
1. Thread `capture_loop` se connecte en HTTP à l'ESP32-CAM
2. Parse les boundaries MJPEG (`0xFF 0xD8` début / `0xFF 0xD9` fin de frame)
3. Décode JPEG → OpenCV BGR via `cv2.imdecode`
4. Redimensionne à 320×240
5. Publie Image `bgr8` + CameraInfo + CompressedImage JPEG 85%

**Mode simulation :** Génère une scène synthétique avec marquages de voie défilants, panneau stop et obstacle mobile.

**Topics publiés :**
- `/camera/image_raw` — `sensor_msgs/Image` (bgr8)
- `/camera/camera_info` — `sensor_msgs/CameraInfo` (fx=fy=160.0, cx=160, cy=120)
- `/camera/image_annotated/compressed` — `sensor_msgs/CompressedImage` (jpeg)
- `/camera/status` — `std_msgs/String`

#### `esibot_camera/mjpeg_proxy.py`

**Classe :** `MjpegProxyNode(Node)`

Re-sert le flux MJPEG ESP32-CAM localement sur **port 8888** via HTTP simple. Permet d'afficher le flux dans le dashboard sans passer par rosbridge (latence directe < 100 ms vs ~300 ms via WebSocket).

#### `launch/camera.launch.py`

Noeuds : `camera_stream_node` + republish image_transport (raw → compressed).

Arguments : `esp32_ip`, `esp32_port`, `sim_mode`

#### `config/camera_params.yaml`

Paramètres : ip ESP32, dimensions frame, publish_rate, reconnect_delay.

---

### 4.4 esibot_description

**Rôle :** Modèle URDF du robot — cinématique, visualisation, TF statique.

**Mainteneur :** Rania Zitouni | `ament_cmake`

#### `urdf/esibot.urdf.xacro`

Fichier principal Xacro générant l'URDF complet.

**Dimensions châssis :** 0.138 m × 0.14 m × 0.012 m

**Propriétés roues :**
- Rayon : 0.033 m
- Largeur : 0.018 m
- Écartement : 0.138 m

**Joints :**

| Joint | Type | Axe | Limite |
|-------|------|-----|--------|
| `base_joint` (base_footprint→base_link) | fixed | — | — |
| `left_wheel_joint` | continuous | Y | ∞ |
| `right_wheel_joint` | continuous | Y | ∞ |
| `caster_wheel_joint` | fixed | — | — |
| `upper_plate_joint` | fixed | — | — |
| `servo_joint` | revolute | Y | ±π rad |
| `camera_joint` | fixed | — | — |

**Matériaux :** `chassis_grey`, `wheel_black`, `servo_white` (définis dans `materials.xacro`)

#### `launch/display.launch.py`

Lance : `robot_state_publisher` + (optionnel) `joint_state_publisher_gui` + (optionnel) `rviz2` + (optionnel) `foxglove_bridge`

#### `launch/full.launch.py`

**Orchestrateur principal du stack robot complet.**

Arguments :
- `sim_mode` (true/false)
- `mode` : `slam` | `nav` | `vision`
- `use_foxglove` (true/false)

**Séquence de démarrage (délais pour éviter race conditions) :**

```
t+0s  → robot_state_publisher (TF statique)
t+0s  → foxglove_bridge (ws:8765)
t+2s  → esibot_driver (odométrie, moteurs)
t+3s  → radar_node (mode slam/nav) OU camera_node (mode vision)
t+4s  → vision_node (mode vision uniquement)
t+5s  → slam_toolbox (mode slam) OU nav2 (mode nav)
```

---

### 4.5 esibot_slam

**Rôle :** Cartographie SLAM en ligne avec slam_toolbox.

**Mainteneur :** Rayan Boukakiou | `ament_cmake`

#### `launch/slam.launch.py`

Arguments :
- `mode` : `sim` | `hw` (détermine le fichier YAML et `use_sim_time`)
- `autostart` : true
- `use_lifecycle_manager` : false (true si Nav2 gère le cycle de vie)
- `use_rviz` : false
- `teleop` : false

**Nœuds :**
1. (sim uniquement) `relay_node` : `/ultrasound_raw` → `/scan` (relay Gazebo gpu_lidar)
2. `async_slam_toolbox_node` — algorithme `online_async` (LifecycleNode)

#### `config/slam_params_hw.yaml`

**Algorithme :** `online_async` — SLAM en ligne asynchrone

Paramètres clés :

| Paramètre | Valeur | Raison |
|-----------|--------|--------|
| `mode` | mapping | Construction de carte |
| `scan_topic` | /scan | Sortie radar_node |
| `resolution` | 0.05 m/cell | Précision carte |
| `max_laser_range` | 4.0 m | Portée HC-SR04 |
| `map_update_interval` | 2.0 s | Fréquence mise à jour carte |
| `minimum_time_interval` | 2.5 s | Période balayage HC-SR04 (~3 s) |
| `minimum_travel_distance` | 0.0 m | Mise à jour même sans déplacement |
| `minimum_travel_heading` | 0.0 rad | Idem |
| `throttle_scans` | 1 | Traiter chaque scan |
| `transform_publish_period` | 0.05 s | TF à 20 Hz |

**Solver :** CeresSolver avec `SPARSE_NORMAL_CHOLESKY`

#### `config/slam_params_sim.yaml`

Idem hardware avec `use_sim_time: true` et adaptations Gazebo.

#### `launch/save_map.launch.py`

Lance `nav2_map_server/map_saver_cli` — exporte la carte courante en `.pgm` + `.yaml`.

#### `maps/`

- `esibot_map.pgm` — Grille d'occupation pré-construite
- `esibot_map.yaml` — Métadonnées (origine, résolution, threshold occupé/libre)

---

### 4.6 esibot_navigation

**Rôle :** Navigation autonome avec carte connue — localisation AMCL + planification Nav2.

**Mainteneur :** Mohamed El Amine Kherroubi | `ament_cmake`

#### `launch/nav2.launch.py`

Arguments :
- `map` (défaut: `esibot_slam/maps/esibot_map.yaml`)
- `params_file` (`nav2_params.yaml`)
- `scan_topic` : `/scan`
- `use_sim_time` : false
- `autostart` : true

**Séquence de démarrage :**
1. Immédiat : `map_server` (charge carte .pgm)
2. Immédiat : AMCL (localisation Monte Carlo par particules)
3. +15 s : Stack Nav2 complet (planner, controller, behavior trees)

Le délai de 15 s laisse AMCL se stabiliser avec les premières scans avant que le planificateur ne démarre.

#### `config/nav2_params.yaml`

Paramètres pour :
- **AMCL** : filtre particulaire — `min_particles`, `max_particles`, `update_min_d`, `update_min_a`
- **BT Navigator** : exécution d'arbres de comportement
- **Global planner** : NavFn ou Smac Planner
- **Local controller** : DWB (Dynamic Window Approach) — `max_vel_x`, `min_vel_x`, `max_rot_vel`

---

### 4.7 esibot_vision

**Rôle :** Détection visuelle multi-tâche — voie (OpenCV), panneaux (YOLOv8n GTSRB), obstacles (YOLOv8n COCO).

**Mainteneur :** Idriss Yacine Ziadi | `ament_cmake`

#### `esibot_vision/vision_node.py`

**Classe :** `VisionNode(Node)`

**Pipeline (15 Hz) :**
1. Reçoit `/camera/image_raw`
2. Passe par `LaneDetector` → lane_error + lane_status
3. Passe par `SignDetector` (si modèle chargé) → panneaux confirmés
4. Passe par `ObstacleDetector` (si modèle chargé) → obstacles en voie
5. Annote le frame via `draw_hud()`
6. Publie tous les topics résultats

**Topics publiés :**
- `/camera/image_annotated` — frame annoté
- `/esibot/lane_error` — `std_msgs/Float32` (−1.0 gauche → +1.0 droite)
- `/esibot/lane_status` — `std_msgs/String` : `IN_LANE|LANE_LEFT|LANE_RIGHT|NO_LANE`
- `/esibot/signs` — `std_msgs/String` (JSON list `[{label, conf}]`)
- `/esibot/obstacles` — `std_msgs/String` (JSON list `[{label, proximity}]`)
- `/esibot/obstacle_in_lane` — `std_msgs/Bool`

#### `esibot_vision/lane_detector.py`

**Classe :** `LaneDetector`

Détecte les marquages de voie noirs sur fond blanc (bandes de ruban adhésif).

**Pipeline :**
1. Niveaux de gris → seuillage (`lane_threshold=60`)
2. Morphologie : OPEN puis CLOSE (supprime bruit)
3. Extraction contours
4. Filtres de validation :
   - Aire minimale ≥ 200 px²
   - Rapport d'élongation ≥ 2.5
   - Hauteur ≥ 10% ROI
5. Sélection : contour avec centroïde le plus bas (= le plus proche du robot)
6. Lissage EMA (α=0.35) + contrainte de saut max 60 px/frame

**Sortie :** `lane_error` normalisé, `lane_status`, positions bords internes gauche/droite.

#### `esibot_vision/sign_detector.py`

**Classe :** `SignDetector`

Modèle : YOLOv8n fine-tuné sur sous-ensemble GTSRB (8 classes)

| ID | Label |
|----|-------|
| 0 | speed_30 |
| 1 | speed_50 |
| 2 | speed_70 |
| 3 | speed_80 |
| 4 | stop |
| 5 | dir_straight |
| 6 | dir_right |
| 7 | dir_left |

**Débruitage temporel :** 3 détections consécutives requises pour confirmer (évite le clignotement).

#### `esibot_vision/obstacle_detector.py`

**Classe :** `ObstacleDetector`

Modèle : YOLOv8n COCO (classes générales)

**Fonctionnement :**
- Filtre spatial : ne retient que les bounding boxes dans la région inter-voies
- Étiquettes de proximité : `VERY_CLOSE` | `CLOSE` | `DETECTED` (selon position verticale dans frame)
- Filtre temporel : 4 frames pour confirmer, 10 frames pour supprimer
- Exclut les classes non pertinentes (laptop, phone, dining table…)

#### `esibot_vision/config.py`

Constantes globales : `SIGN_CLASSES`, couleurs BGR par classe, `GTSRB_TO_LOCAL` (mapping IDs GTSRB → indices locaux), étiquettes de proximité.

#### `esibot_vision/utils.py`

- `FPSCounter` : calcul FPS glissant
- `draw_hud()` : annotation frame avec boîtes, labels, FPS, lane_error

#### `scripts/train_signs.py`

Fine-tuning YOLOv8n sur GTSRB. Génère `signs_best.pt`.

#### `scripts/prepare_gtsrb.py`

Téléchargement et préparation du dataset GTSRB au format YOLOv8.

#### `models/`

- `yolov8n.pt` — modèle COCO pré-entraîné (pour obstacles)
- `signs_best.pt` — modèle GTSRB fine-tuné (non versionné dans git, ~300 MB)

#### `config/vision_params.yaml`

```yaml
image_width: 320
image_height: 240
lane_threshold: 60
lane_min_area: 200
sign_conf: 0.60
obstacle_conf: 0.40
process_rate: 15.0
sign_model_path: ""      # Vide = désactivé
obstacle_model_path: ""  # Vide = désactivé
```

---

### 4.8 esibot_ui

**Rôle :** Serveur du dashboard React + proxy Nav2 + déclencheur sauvegarde carte.

**Mainteneur :** EsiBot Team | `ament_python`

#### `esibot_ui/dashboard_node.py`

**Classe :** `DashboardNode(Node)`

Sert les fichiers statiques React du répertoire `web/` via `http.server.SimpleHTTPRequestHandler` sur le port 8080. Démarre le serveur HTTP dans un thread daemon.

**Accès :** `http://<robot_ip>:8080`

#### `esibot_ui/nav_goal_proxy.py`

**Classe :** `NavGoalProxy(Node)`

Pont entre le dashboard (clic sur carte) et l'action Nav2 `/navigate_to_pose`.

**Flow :**
1. Dashboard publie `PoseStamped` sur `/nav_goal`
2. Proxy annule l'objectif courant si actif
3. Envoie nouveau `NavigateToPose.Goal`
4. Publie statut en temps réel sur `/nav_goal_status` : `sending|navigating|reached|error`

**Timeout :** 3 s pour la connexion au serveur d'action Nav2.

#### `esibot_ui/map_saver_node.py`

**Classe :** `MapSaverNode(Node)`

Écoute `/save_map` (Empty), déclenche `subprocess.run()` sur `esibot_slam save_map.launch.py`, publie statut `saving|saved|error` sur `/save_map_status`. Timeout : 20 s.

#### `esibot_ui/web/`

Build React compilé (servi par `dashboard_node`). Contient `index.html` + assets JS/CSS.

#### `launch/dashboard.launch.py`

Lance : `dashboard_node` + (optionnel) `nav_goal_proxy` + (optionnel) `map_saver_node`.

---

### 4.9 esibot_logging

**Rôle :** Utilitaires de logging coloré partagés entre packages.

**Mainteneur :** Mohamed El Amine Kherroubi | `ament_python`

#### `esibot_logging/logging_utils.py`

- `ColorFormatter(logging.Formatter)` : colorise le niveau de log dans le terminal
  - DEBUG → gris, INFO → vert, WARNING → jaune, ERROR → rouge, CRITICAL → blanc sur rouge
- `setup_logging()` : initialise le logger root avec handler coloré
- `get_logger(node=None)` : retourne le logger ROS 2 (si nœud fourni) ou logger Python

**Variables d'environnement :** `NO_COLOR`, `FORCE_COLOR`, `ESIBOT_LOG_LEVEL`

---

### 4.10 esibot_gazebo

**Rôle :** Environnement de simulation Gazebo Harmonic.

**Mainteneur :** Rania Zitouni | `ament_cmake`

#### `launch/sim.launch.py`

Lance Gazebo Harmonic (`gz sim`) + spawn de l'URDF dans le monde SDF.

#### `worlds/esibot_world.sdf`

Monde Gazebo : physique, éclairage, plan de sol.

**Capteurs simulés :**
- `gpu_lidar` : simule le radar HC-SR04 rotatif (publie sur `/ultrasound_raw`)
- Camera optionelle

---

### 4.11 web_bridge

**Rôle :** Bridge WebSocket ROS 2 ↔ JavaScript (rosbridge_suite).

**Mainteneur :** EsiBot Team | `ament_python`

#### `launch/web_bridge.launch.py`

Lance :
1. `rosbridge_websocket` — port **9090** (ws://\<robot\>:9090)
2. `rosapi` — introspection services (liste topics, types, etc.)

#### `config/rosbridge_params.yaml`

Paramètres rosbridge : filtrage messages, compression, authentification.

---

## 5. Dashboard React

**Tech stack :** React 18 + Vite + roslib.js

**Structure `dashboard/src/` :**

```
src/
├── main.jsx                  # Point d'entrée React
├── App.jsx                   # Layout 3 colonnes + providers
├── config.js                 # URLs, vitesses, config globale
├── context/
│   ├── RosbridgeContext.jsx  # Connexion WebSocket globale
│   └── ThemeContext.jsx      # Toggle dark/light
├── components/
│   ├── Header.jsx            # Titre + thème + latence
│   ├── VideoFeed.jsx         # Flux caméra ESP32-CAM
│   ├── Teleop.jsx            # Joystick clavier → /cmd_vel
│   ├── MapCanvas.jsx         # Visualisation 2D carte + interaction
│   ├── BatteryPanel.jsx      # Niveau batterie
│   ├── ServoGauge.jsx        # Angle servo radar
│   ├── ConnectionPanel.jsx   # Statut rosbridge
│   └── Toast.jsx             # Notifications toast
├── hooks/
│   ├── useRosbridge.js       # Accès instance ROSLIB.Ros
│   ├── useBattery.js         # /battery_state
│   ├── useCamera.js          # /camera/status
│   ├── useMap.js             # /map (OccupancyGrid)
│   ├── useOdom.js            # /odom (pose x,y,θ)
│   ├── useScan.js            # /scan (overlay)
│   └── useServo.js           # /joint_states (angle servo)
└── utils/
    └── mapUtils.js           # Fonctions rendu canvas 2D
```

### `config.js`

```javascript
ROSBRIDGE_URL: `ws://${hostname}:9090`
ROBOT_NAME: "EsiBot"
CMD_VEL: { LINEAR_SPEED: 0.4, ANGULAR_SPEED: 1.5 }
ESP32_STREAM_URL: 'http://10.225.87.99:8888/stream'
BATTERY_CAPACITY_MINUTES: 45
```

### `RosbridgeContext.jsx`

Gère la connexion globale ROSLIB.Ros :
- Auto-reconnexion après 5 s en cas de déconnexion
- Mesure latence périodique via service `/rosapi/topics`
- État exposé : `connected`, `connecting`, `latency`, `rosRef`

### `MapCanvas.jsx`

Composant central de visualisation et d'interaction :

**Données visualisées (canvas offscreen) :**
- Grille d'occupation `/map` (noir=occupé, blanc=libre, gris=inconnu)
- Pose robot `/odom` (triangle)
- Scan laser `/scan` (rayons semi-transparents)
- Historique trajectoire (fil d'Ariane)
- Objectif de navigation (marqueur)

**Interactions souris :**
- **Pan** : clic+glisser déplace la vue
- **Zoom** : molette / pinch trackpad
- **Goal** : clic simple en mode goal → publie `/nav_goal`
- **Init pose** : clic+glisser → direction → publie pose initiale AMCL

**`mapUtils.js` :**
- `worldToCanvas(x, y, meta, viewport)` — coordonnées monde → pixels
- `canvasToWorld(px, py, meta, viewport)` — pixels → coordonnées monde
- `drawRobot(ctx, pose, ...)` — triangle orienté
- `drawScan(ctx, scan, pose, ...)` — rayons laser
- `drawPath(ctx, poses, ...)` — trajectoire
- `drawGoal(ctx, goal, ...)` — marqueur objectif
- `drawGrid(ctx, meta, viewport)` — grille de référence
- `drawScaleBar(ctx, meta, viewport)` — barre d'échelle
- `drawMapInfo(ctx, stats)` — métadonnées carte

### `Teleop.jsx`

Boutons directionnels (↑↓←→) + support clavier (flèches + Espace = stop).

- Publie `/cmd_vel` à **10 Hz** pendant appui
- Arrêt automatique après **200 ms** sans input
- Vitesses depuis `config.js` : 0.4 m/s linéaire, 1.5 rad/s angulaire

---

## 6. Communication réseau

### Ports utilisés

| Port | Protocole | Service |
|------|-----------|---------|
| 8080 | HTTP | Dashboard React (`dashboard_node`) |
| 9090 | WebSocket | rosbridge (`web_bridge`) |
| 8888 | HTTP MJPEG | Proxy caméra ESP32-CAM (`mjpeg_proxy`) |
| 8765 | WebSocket | foxglove_bridge (optionnel) |
| 80 | HTTP | ESP32-CAM stream direct |

### Topics ROS 2 principaux

| Topic | Type | Publisher | Subscribers |
|-------|------|-----------|-------------|
| `/cmd_vel` | Twist | Teleop / Nav2 | esibot_driver |
| `/odom` | Odometry | esibot_driver | slam_toolbox, nav2, dashboard |
| `/tf` | TFMessage | esibot_driver, robot_state_publisher | slam_toolbox, nav2, rviz |
| `/scan` | LaserScan | radar_node | slam_toolbox, nav2, dashboard |
| `/joint_states` | JointState | radar_node | robot_state_publisher, dashboard |
| `/map` | OccupancyGrid | slam_toolbox | nav2, dashboard |
| `/battery_state` | BatteryState | esibot_driver | dashboard |
| `/camera/image_raw` | Image | camera_stream_node | vision_node |
| `/esibot/lane_error` | Float32 | vision_node | — |
| `/esibot/signs` | String | vision_node | — |
| `/nav_goal` | PoseStamped | dashboard | nav_goal_proxy |
| `/nav_goal_status` | String | nav_goal_proxy | dashboard |
| `/save_map` | Empty | dashboard | map_saver_node |

---

## 7. Lancement du projet

### Prérequis Raspberry Pi

```bash
# Démon GPIO (requis pour servo + encodeurs)
sudo pigpiod

# Source de l'environnement ROS 2
source /opt/ros/jazzy/setup.bash
source ~/robot_ws/install/setup.bash
```

### `launch_robot.sh` (depuis PC)

Script d'orchestration remote via SSH (sshpass + tmux) :

```bash
./launch_robot.sh [PI_IP] [mapping|navigation]
# Ex : ./launch_robot.sh 10.83.158.99 mapping
```

**Ce que fait le script :**
1. `ssh_run()` : exécute des commandes SSH avec sshpass (pas de prompt mot de passe)
2. Lance `pigpiod` si non actif
3. Ouvre session tmux `esibot` avec 2 fenêtres :
   - Window 0 : stack robot complet (`full.launch.py mode:=slam` ou `mode:=nav`)
   - Window 1 : dashboard + web_bridge
4. `wait_for_topic()` : poll `/odom` avant de continuer (attente que le stack soit prêt)

**Accès dashboard :** `http://<PI_IP>:8080`

### Lancement manuel sur Pi

```bash
# Mode cartographie
ros2 launch esibot_description full.launch.py sim_mode:=false mode:=slam

# Mode navigation (avec carte existante)
ros2 launch esibot_description full.launch.py sim_mode:=false mode:=nav

# Mode vision
ros2 launch esibot_description full.launch.py sim_mode:=false mode:=vision

# Simulation Gazebo
ros2 launch esibot_description full.launch.py sim_mode:=true mode:=slam
```

### Lancement individuel des composants

```bash
# Driver seul
ros2 launch esibot_bringup bringup.launch.py

# Radar seul
ros2 launch esibot_sensors radar.launch.py

# SLAM seul (suppose driver + radar actifs)
ros2 launch esibot_slam slam.launch.py mode:=hw

# Navigation seule (suppose driver + radar actifs + carte)
ros2 launch esibot_navigation nav2.launch.py

# Dashboard seul
ros2 launch esibot_ui dashboard.launch.py

# WebSocket bridge
ros2 launch web_bridge web_bridge.launch.py

# Sauvegarde carte
ros2 launch esibot_slam save_map.launch.py
```

### Build du workspace

```bash
cd ~/robot_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

### Build du dashboard

```bash
cd ~/robot_ws/src/dashboard
npm install
npm run build
# → génère dashboard/dist/ (copié dans esibot_ui/web/)
```

---

## 8. Calibration et tuning

### esibot_driver — paramètres mécaniques

| Paramètre | Valeur | Méthode de mesure |
|-----------|--------|-------------------|
| `WHEEL_BASE` | 0.16 m | Mesurer centre-à-centre des roues |
| `WHEEL_RADIUS` | 0.033 m | Diamètre externe ÷ 2 |
| `TICKS_PER_REV` | 40 | Nb trous disque × 2 (both edges) |
| `MAX_PWM_DUTY` | 55% | Empirique (évite dérapage à pleine vitesse) |

### radar_node — tuning servo

| Paramètre | Valeur | Effet |
|-----------|--------|-------|
| `servo_coeff` | 6.0 µs/° | Calibrer si le servo ne suit pas l'angle commandé |
| `settle_ms` | 20 ms | Augmenter si vibrations dans les mesures |
| `median_reads` | 1–3 | Augmenter pour réduire le bruit (ralentit le sweep) |
| `angle_increment` | 5° | 37 faisceaux, réduire pour plus de résolution |

### slam_toolbox — tuning carte

| Paramètre | Valeur | Effet si trop élevé | Effet si trop bas |
|-----------|--------|---------------------|-------------------|
| `minimum_time_interval` | 2.5 s | Scans manqués | Re-traitement inutile |
| `map_update_interval` | 2.0 s | Carte figée | CPU élevé |
| `resolution` | 0.05 m | Moins de détails | Plus de mémoire/CPU |

### vision — seuils

| Paramètre | Valeur | Ajuster si |
|-----------|--------|-----------|
| `lane_threshold` | 60 | Mauvaise détection voie (tapis coloré, éclairage) |
| `sign_conf` | 0.60 | Trop de faux positifs → monter; manque panneaux → descendre |
| `obstacle_conf` | 0.40 | Idem obstacles |

---

## 9. Outils et scripts

### `launch_robot.sh`

Déploiement complet depuis PC vers Pi. Cf. section 7.

### `tools/project_snapshot.sh`

Capture un snapshot texte du projet entier :

```bash
./tools/project_snapshot.sh [-I ignore_pattern] [-q]
# Output: .snapshots/snapshot_<branch>_<timestamp>.txt
```

Contenu : arbre de fichiers + contenu de tous les fichiers texte. Utile pour debugging ou revue de code.

**Patterns ignorés par défaut :** `.git`, `__pycache__`, `*.pyc`, `.vscode`, `.idea`, `*.db3`, `*.mcap`

### `tools/launch_rviz.sh`

Lance RViz2 avec config `esibot_slam/config/esibot_slam.rviz`.

### `docs/plans/`

Plans de conception techniques (markdown) :
- `2026-05-21-cyclonedds-local-discovery-fix.md` — Fix découverte DDS locale

### `docs/esibot_map.yaml` + `docs/esibot_map.pgm`

Carte SLAM pré-construite de l'environnement de test.

---

## 10. Carte des topics ROS 2

```
[teleop_twist_keyboard]──/cmd_vel──┐
[dashboard/Teleop.jsx]─────────────┤
                                   ▼
                          [esibot_driver]
                          /odom ──────────────────────────────────────────┐
                          /tf (odom→base_footprint) ──────────────────────┤
                          /battery_state ─────────────────────────────────┤
                                                                           │
[radar_node]──/scan ────────────────────────────────────────────┐         │
              /joint_states ─────────────────────────────────────┤         │
                                                                 │         │
[robot_state_publisher]──/tf_static (base_footprint→laser_link)─┤         │
                                                                 │         │
                          [slam_toolbox] ◄── /scan + /tf ────────┘         │
                          /map ──────────────────────────────────────────┐ │
                                                                         │ │
                          [nav2 stack] ◄── /map + /scan + /odom/tf ─────┘ │
                          /navigate_to_pose (action) ◄── [nav_goal_proxy] │
                          /cmd_vel ──────────────────────────────────────┐ │
                                                                         │ │
[camera_stream_node]──/camera/image_raw ──────┐                          │ │
                      /camera/image_annotated/compressed                  │ │
                                              │                          │ │
                          [vision_node] ◄─────┘                          │ │
                          /esibot/lane_error                              │ │
                          /esibot/signs                                   │ │
                          /esibot/obstacles                               │ │
                          /esibot/obstacle_in_lane                       │ │
                                                                         │ │
[rosbridge_websocket] ◄──────────── tous les topics ci-dessus ───────────┘─┘
         │
[Dashboard React]
  ├── MapCanvas    ← /map + /odom + /scan + /tf
  ├── Teleop       → /cmd_vel
  ├── VideoFeed    ← http://:8888/stream (MJPEG direct)
  ├── BatteryPanel ← /battery_state
  ├── ServoGauge   ← /joint_states
  └── ConnectionPanel ← latence rosbridge
```
