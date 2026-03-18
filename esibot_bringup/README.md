# esibot_bringup
Package ROS2 — **Tâche 3.2** : Contrôle moteur + Odométrie ESP32 → ROS2
| ROS2 Jazzy | Ubuntu 24.04

## Description

Ce package fait le pont entre l'**ESP32** (moteurs + encodeurs) et **ROS2**.  
Il reçoit les commandes clavier, les transmet à l'ESP32, lit les encodeurs en retour,  
et calcule la position du robot en temps réel.

```
Clavier (WASD)
      │  /cmd_vel  (geometry_msgs/Twist)
      ▼
esibot_driver node  (odométrie différentielle)
      │  UART série  /dev/ttyUSB0
      ▼
   ESP32-CAM  ──► moteurs DC gauche/droite
      │  encodeurs (ticks)
      └──────────────────────────────────────────┐
                                                 ▼
                                    /odom  (nav_msgs/Odometry)
                                    /tf    (odom → base_footprint)
                                    /battery_state
```

## Prérequis

- Ubuntu 24.04
- ROS2 Jazzy
- Python 3.12+
- pyserial (`pip install pyserial`)

## Installation

### 1. Installer les dépendances
```bash
cd ~/robot_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y
pip install pyserial
```

### 2. Compiler
```bash
cd ~/robot_ws
colcon build --symlink-install --packages-select esibot_bringup
source install/setup.bash
```

### 3. Fix WSL2 (si vous êtes sur Windows avec WSL2)

ROS2 a un problème de communication entre terminaux sous WSL2. Installer CycloneDDS pour le résoudre :
```bash
sudo apt install ros-jazzy-rmw-cyclonedds-cpp -y
echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc
source ~/.bashrc
```

## Configuration

Les paramètres du package sont dans `config/driver_params.yaml` :

```yaml
esibot_driver:
  ros__parameters:
    serial_port:  '/dev/ttyUSB0'   # port série vers l'ESP32
    baud_rate:    115200
    odom_frame:   'odom'
    base_frame:   'base_footprint'
    publish_rate: 20.0             # Hz
```

Modifier `serial_port` selon le port détecté sur le Raspberry Pi :
```bash
ls /dev/ttyUSB*    # connexion USB
ls /dev/ttyAMA*    # connexion GPIO UART
```

| Paramètre | Défaut | Description |
|---|---|---|
| serial_port | /dev/ttyUSB0 | Port UART vers l'ESP32 |
| baud_rate | 115200 | Vitesse série (doit correspondre au firmware ESP32) |
| odom_frame | odom | Frame parent de l'odométrie |
| base_frame | base_footprint | Frame du robot |
| publish_rate | 20.0 | Fréquence de publication (Hz) |

## Lancement

```bash
source ~/robot_ws/install/setup.bash

# Mode simulation (sans matériel)
ros2 launch esibot_bringup bringup.launch.py

# Mode réel (avec ESP32 connecté)
ros2 launch esibot_bringup bringup.launch.py serial_port:=/dev/ttyUSB0
```

Ou lancer le nœud directement :
```bash
ros2 run esibot_bringup esibot_driver
```

En mode simulation (sans ESP32), le nœud affiche :
```
[WARN] Serial not available (...) — running in simulation mode.
[INFO] esibot_driver started | port=/dev/ttyUSB0 | 20.0Hz
```
C'est le comportement attendu.

## Visualisation

**Option 1 — Foxglove Studio** (recommandé si RViz2 non disponible) :
```bash
ros2 run foxglove_bridge foxglove_bridge
```
Puis ouvrir https://app.foxglove.dev → New connection → WebSocket → `ws://localhost:8765`  
Ajouter panneau **Odometry** sur le topic `/odom`.

**Option 2 — RViz2** :
```bash
ros2 run rviz2 rviz2
```
Ajouter display type **Odometry** → topic `/odom`.

**Vérifier les topics en ligne de commande :**
```bash
ros2 topic list
ros2 topic echo /odom
```

## Topics

| Topic | Type | Rôle |
|---|---|---|
| `/odom` | nav_msgs/Odometry | Position calculée par odométrie |
| `/tf` | geometry_msgs/TransformStamped | Transform odom → base_footprint |
| `/battery_state` | sensor_msgs/BatteryState | Tension batterie |
| `/cmd_vel` | geometry_msgs/Twist | Commandes vitesse reçues (clavier / Nav2) |

## Format de communication ESP32 ↔ Raspberry Pi

Le nœud utilise ce protocole série simple :

| Direction | Format | Exemple |
|---|---|---|
| ESP32 → RPi (encodeurs) | `ENC:<left_ticks>,<right_ticks>,<voltage>\n` | `ENC:1234,1236,7.8` |
| RPi → ESP32 (moteurs) | `CMD:<v_right>,<v_left>\n` | `CMD:0.300,-0.280` |

> Le firmware ESP32 doit respecter exactement ce format.

## Structure du package

```
esibot_bringup/
├── package.xml
├── setup.py
├── setup.cfg               ← requis pour que ROS2 trouve l'exécutable
├── README.md
├── resource/
│   └── esibot_bringup      ← fichier marqueur requis par ROS2 (vide)
├── esibot_bringup/
│   ├── __init__.py
│   └── esibot_driver.py    ← nœud principal
├── launch/
│   └── bringup.launch.py
└── config/
    └── driver_params.yaml  ← configuration par défaut
```

## Ce qui change avec le vrai matériel

Tout le code ROS2 (odométrie, topics, TF) reste identique.  
Seules ces trois choses changent :

| Où | Quoi faire |
|---|---|
| `_connect_serial()` | Confirmer le bon port avec `ls /dev/ttyUSB*` sur le Pi |
| `_read_encoders()` | S'assurer que le firmware ESP32 envoie le format `ENC:...` |
| `_cmd_vel_callback()` | S'assurer que le firmware ESP32 lit le format `CMD:...` |

Le format de communication doit être convenu avec la personne qui écrit le firmware ESP32.
