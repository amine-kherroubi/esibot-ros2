# esibot_ui — Dashboard Web EsiBot

Interface web temps réel pour le robot EsiBot, accessible depuis n'importe quel navigateur sur le réseau local.

**Fonctionnalités :**
- Carte SLAM en temps réel (pan, zoom, recentrage)
- Flux caméra (brut ou annoté par la vision)
- Envoi de goal de navigation Nav2 (clic sur la carte)
- Définition de pose initiale AMCL (clic sur la carte)
- Sauvegarde de la carte SLAM
- Téléopération clavier (WASD)
- Affichage batterie, angle servo radar, scan LIDAR

---

## Panels de l'interface

### Connection
Gère la connexion WebSocket entre le navigateur et le robot.

| Élément | Description |
|---------|-------------|
| Champ URL | Adresse rosbridge — ex. `ws://192.168.1.34:9090` |
| Bouton Connect/Disconnect | Ouvre/ferme la connexion WebSocket |
| Latence | Temps aller-retour mesuré en ms |

**Source de données :** aucun topic ROS2 — connexion directe via `ROSLIB.Ros` au rosbridge WebSocket.

---

### Map
Affiche la carte SLAM construite par slam_toolbox, la position du robot, son chemin parcouru, le scan LIDAR et les goals/poses.

| Élément | Topic ROS2 | Description |
|---------|-----------|-------------|
| Carte (fond gris/blanc/noir) | `/map` (`nav_msgs/OccupancyGrid`) | Reçue une fois au démarrage puis à chaque mise à jour SLAM. Dessinée sur un canvas offscreen. |
| Position du robot (triangle) | `/odom` (`nav_msgs/Odometry`) | Mise à jour en continu. Position `x, y` + cap `yaw` extraits du quaternion. |
| Chemin parcouru (trait bleu) | `/odom` | Accumulé localement dans le navigateur (max 500 points, pas de 5 cm). |
| Scan LIDAR (rayons rouges) | `/scan` (`sensor_msgs/LaserScan`) | Optionnel (`SCAN_OVERLAY=true` dans config.js). Projeté sur la carte selon la pose du robot. |
| Goal de navigation (fanion) | Calculé au clic | Affiché après clic sur la carte en mode Goal. Coordonnées canvas → coordonnées monde. |
| Pose initiale (croix verte) | Calculé au clic | Affiché après clic en mode Pose Initiale. |

**Contrôles de la carte :**
- **Molette** — zoom (0.2× à 20×)
- **Clic + drag** — déplacer la vue (pan)
- **⊙ Recentrer** — recentre le robot à l'écran
- **⊕ Pose initiale** — clic sur la carte → publie sur `/initialpose` (AMCL)
- **⚑ Envoyer un goal** — clic sur la carte → envoie le goal Nav2 via `nav_goal_proxy`
- **💾 Save Map** — publie sur `/save_map` → `map_saver_node` sauvegarde la carte

---

### Camera
Affiche le flux vidéo de la caméra ESP32 du robot.

| Mode | Topic ROS2 | Description |
|------|-----------|-------------|
| Annotated | `/camera/image_annotated` (`sensor_msgs/Image`) | Image avec les détections de `esibot_vision` superposées (boîtes, labels). |
| Raw | `/camera/compressed` (`sensor_msgs/CompressedImage`) | Image JPEG brute, sans traitement. |

**Source de données :** images reçues via rosbridge, converties en base64 et affichées dans une balise `<img>`. Throttle automatique selon la bande passante réseau.

---

### Teleop
Permet de contrôler le robot manuellement au clavier.

| Touche | Action |
|--------|--------|
| `W` / `↑` | Avancer |
| `S` / `↓` | Reculer |
| `A` / `←` | Tourner à gauche |
| `D` / `→` | Tourner à droite |

**Source de données :** aucun topic en entrée. Publie sur `/cmd_vel` (`geometry_msgs/Twist`) à 10 Hz tant qu'une touche est maintenue. Arrêt automatique 200 ms après le relâchement de la touche.

Vitesses configurables dans `config.js` : `CMD_VEL.LINEAR_SPEED` (défaut 0.4 m/s), `CMD_VEL.ANGULAR_SPEED` (défaut 1.5 rad/s).

---

### Battery
Affiche le niveau de batterie du robot sous forme de barre graphique.

| Élément | Description |
|---------|-------------|
| Icône batterie | Couleur verte > 50%, orange > 20%, rouge ≤ 20% |
| Pourcentage | Affiché en valeur numérique |
| Temps restant estimé | Calculé depuis `BATTERY_CAPACITY_MINUTES` dans config.js |

**Source de données :** `/battery_state` (`sensor_msgs/BatteryState`). Champ `percentage` utilisé (valeur 0.0–1.0).

---

### Servo Gauge
Affiche l'angle actuel du servo-moteur qui oriente le radar ultrasonique.

| Élément | Description |
|---------|-------------|
| Jauge semi-circulaire | Arc de 240° représentant 0°–180° |
| Valeur numérique | Angle en degrés affiché au centre |

**Source de données :** `/esibot/servo_angle` (`std_msgs/Float32`). Valeur en degrés entre 0 et 180.

---

## Architecture

```
Robot (container Docker)
├── bringup          →  driver moteurs, odométrie, /cmd_vel, /odom, /tf
├── slam             →  carte SLAM (/map, TF map→odom)
├── sensors          →  radar (/scan, /esibot/servo_angle)
├── camera           →  flux image (/camera/compressed, /camera/image_annotated)
├── vision           →  détection objets (annote /camera/image_annotated)
├── nav2             →  navigation autonome (/navigate_to_pose)
│
├── dashboard_node   →  http://robot-ip:8080   (sert les fichiers web statiques)
├── web_bridge       →  ws://robot-ip:9090     (rosbridge : ROS2 ↔ navigateur WebSocket)
├── nav_goal_proxy   →  reçoit /nav_goal (topic) → envoie action NavigateToPose
└── map_saver_node   →  reçoit /save_map (topic) → lance map_saver_cli

Navigateur (PC / téléphone)
└── ouvre http://robot-ip:8080 → se connecte automatiquement à ws://robot-ip:9090
```

### Pourquoi un nav_goal_proxy ?

Le dashboard web ne peut pas appeler directement une action ROS2 (protocole trop complexe pour rosbridge dans ce contexte). Le proxy fait le pont :

```
Dashboard JS  →  publie PoseStamped sur /nav_goal  →  nav_goal_proxy  →  action NavigateToPose  →  bt_navigator
                                                                       ↓
Dashboard JS  ←  subscribe /nav_goal_status (String) ←─────────────────
                 (sending | navigating | reached | error)
```

### Pourquoi le relay cmd_vel ?

Nav2 publie sur `/cmd_vel_nav` → `velocity_smoother` lisse et publie sur `/cmd_vel_smoothed`.
Mais le driver du robot écoute `/cmd_vel`. Un relay fait le pont :

```
velocity_smoother (/cmd_vel_smoothed)  →  relay  →  /cmd_vel  →  esibot_driver
```

Ce relay doit être lancé manuellement (voir section Lancement).

---

## Installation (une seule fois)

### 1. Compiler le dashboard React

Le code source React se trouve dans `dashboard/` à la racine du workspace.

```bash
cd ~/esibot_ws/src/dashboard
npm install
npm run build
cp -r dist/ ../esibot_ui/web/
```

### 2. Compiler les packages ROS2

```bash
cd ~/esibot_ws
colcon build --packages-select web_bridge esibot_ui
source install/setup.bash
```

> **Important :** après chaque modification de `nav_goal_proxy.py`, `map_saver_node.py` ou `dashboard_node.py`, relancer `colcon build --packages-select esibot_ui` sans `--symlink-install` pour que les métadonnées du package soient correctement installées.

---

## Lancement complet (ordre à respecter)

### Étape 1 — Lancer les packages robot

**Hardware réel :**

```bash
# Terminal 1 — driver + odométrie
ros2 launch esibot_bringup bringup.launch.py

# Terminal 2 — capteur radar
ros2 launch esibot_sensors radar.launch.py

# Terminal 3 — SLAM
ros2 launch esibot_slam slam.launch.py mode:=hw

# Terminal 4 — caméra
ros2 launch esibot_camera camera.launch.py esp32_ip:=192.168.1.80

# Terminal 5 — vision (optionnel — lourd en CPU)
ros2 launch esibot_vision vision.launch.py
```

**Simulation (sans hardware) :**

```bash
ros2 launch esibot_bringup bringup.launch.py sim_mode:=true
ros2 launch esibot_sensors radar.launch.py   sim_mode:=true
ros2 launch esibot_slam    slam.launch.py    mode:=hw
ros2 launch esibot_camera  camera.launch.py  sim_mode:=true
ros2 launch esibot_vision  vision.launch.py
```

### Étape 2 — Lancer Nav2 (navigation autonome)

**Mode SLAM actif (carte construite en direct) :**

```bash
ros2 launch esibot_navigation nav2.launch.py use_rviz:=false slam_mode:=true
```

**Mode carte pré-construite (AMCL + localisation) :**

```bash
ros2 launch esibot_navigation nav2.launch.py use_rviz:=false
```

> Attendre le message `Managed nodes are active` avant de continuer.

### Étape 3 — Lancer le relay cmd_vel

Sans ce relay, le robot ne bouge pas physiquement lors d'une navigation Nav2.

```bash
ros2 run topic_tools relay /cmd_vel_smoothed /cmd_vel
```

### Étape 4 — Lancer le dashboard

```bash
ros2 launch esibot_ui dashboard.launch.py
```

Lance automatiquement :
- `dashboard_node` (serveur HTTP port 8080)
- `rosbridge_websocket` (WebSocket port 9090)
- `nav_goal_proxy` (proxy action NavigateToPose)
- `map_saver_node` (sauvegarde carte)

### Étape 5 — Ouvrir dans le navigateur

```
http://<ip-du-robot>:8080
```

---

## Problèmes connus et solutions

### Nav2 échoue à démarrer — "Failed to change state for node: controller_server"

**Cause :** plusieurs instances Nav2 tournent simultanément (participants DDS fantômes après un kill).

**Solution :**
```bash
# 1. Killer tous les processus nav2
kill -9 $(ps aux | grep -E 'controller_server|bt_navigator|lifecycle_manager|planner_server|smoother_server|behavior_server|velocity_smoother|collision_monitor|waypoint_follower|opennav_docking' | grep -v grep | awk '{print $2}') 2>/dev/null

# 2. Attendre 30 secondes (nettoyage DDS obligatoire)
sleep 30

# 3. Stopper le daemon ROS2
ros2 daemon stop

# 4. Relancer Nav2
ros2 launch esibot_navigation nav2.launch.py use_rviz:=false slam_mode:=true
```

### Le robot ne bouge pas physiquement lors d'une navigation

**Cause :** le relay `/cmd_vel_smoothed → /cmd_vel` n'est pas lancé.

**Solution :** lancer le relay (Étape 3 ci-dessus).

**Vérification :**
```bash
ros2 topic hz /cmd_vel   # doit afficher une fréquence pendant la navigation
```

### Le dashboard reste sur "Envoi…" après envoi d'un goal

**Cause A :** Nav2 n'est pas lancé ou pas encore actif.
**Solution :** attendre `Managed nodes are active` dans les logs Nav2.

**Cause B :** conflit DDS — `nav_goal_proxy` tourne en double instance.
**Solution :** killer tous les processus nav_goal_proxy et relancer uniquement le dashboard.

### Erreur TF "Lookup would require extrapolation into the future"

**Cause :** SLAM toolbox ou MPPI trop chargé en CPU, écart de ~50ms dans le TF.

**Solution :** réduire la charge CPU — stopper `esibot_vision` si inutile (consomme >500% CPU).
```bash
kill -9 $(pgrep -f vision_node)
```

Les paramètres MPPI dans `esibot_navigation/config/nav2_params.yaml` sont déjà optimisés :
```yaml
controller_frequency: 10.0
model_dt:             0.1
batch_size:           500
time_steps:           30
```

---

## Modifier le dashboard (React)

Le code source React est dans `dashboard/` à la racine du workspace (pas dans ce package).

Si tu modifies un fichier dans `dashboard/src/` :

```bash
# 1. Supprimer l'ancien build
rm -rf ~/esibot_ws/src/esibot_ui/web/

# 2. Recompiler React
cd ~/esibot_ws/src/dashboard
npm run build
cp -r dist/ ../esibot_ui/web/

# 3. Rebuilder le package ROS2
cd ~/esibot_ws
colcon build --packages-select esibot_ui
source install/setup.bash

# 4. Relancer
ros2 launch esibot_ui dashboard.launch.py
```

---

## Configuration

Fichier : `dashboard/src/config.js`

| Paramètre | Défaut | Description |
|-----------|--------|-------------|
| `ROSBRIDGE_URL` | `ws://localhost:9090` | IP du robot |
| `ROBOT_NAME` | `EsiBot` | Nom affiché dans l'interface |
| `CMD_VEL.LINEAR_SPEED` | `0.4` | Vitesse linéaire téléop (m/s) |
| `CMD_VEL.ANGULAR_SPEED` | `1.5` | Vitesse angulaire téléop (rad/s) |
| `BATTERY_CAPACITY_MINUTES` | `45` | Autonomie estimée (min) |
| `SCAN_OVERLAY` | `false` | Afficher le scan LIDAR sur la carte |

---

## Topics ROS2

| Topic | Direction | Type | Description |
|-------|-----------|------|-------------|
| `/map` | ← robot | `nav_msgs/OccupancyGrid` | Carte SLAM |
| `/odom` | ← robot | `nav_msgs/Odometry` | Position et vitesse robot |
| `/scan` | ← robot | `sensor_msgs/LaserScan` | Scan LIDAR radar |
| `/camera/compressed` | ← robot | `sensor_msgs/CompressedImage` | Flux caméra brut |
| `/camera/image_annotated` | ← robot | `sensor_msgs/Image` | Caméra avec détections vision |
| `/battery_state` | ← robot | `sensor_msgs/BatteryState` | Niveau batterie |
| `/esibot/servo_angle` | ← robot | `std_msgs/Float32` | Angle servo radar |
| `/nav_goal_status` | ← robot | `std_msgs/String` | État navigation (sending/navigating/reached/error) |
| `/save_map_status` | ← robot | `std_msgs/String` | État sauvegarde carte |
| `/cmd_vel` | → robot | `geometry_msgs/Twist` | Commandes vitesse téléop |
| `/nav_goal` | → robot | `geometry_msgs/PoseStamped` | Goal de navigation (via nav_goal_proxy) |
| `/initialpose` | → robot | `geometry_msgs/PoseWithCovarianceStamped` | Pose initiale AMCL |
| `/save_map` | → robot | `std_msgs/Empty` | Déclenche la sauvegarde de la carte |

---

## Noeuds lancés par dashboard.launch.py

| Noeud | Package | Description |
|-------|---------|-------------|
| `dashboard_node` | `esibot_ui` | Serveur HTTP Python (port 8080), sert les fichiers web statiques |
| `rosbridge_websocket` | `rosbridge_server` | Pont WebSocket ROS2 ↔ navigateur (port 9090) |
| `nav_goal_proxy` | `esibot_ui` | Reçoit `/nav_goal`, appelle l'action `/navigate_to_pose`, publie statut sur `/nav_goal_status` |
| `map_saver_node` | `esibot_ui` | Reçoit `/save_map`, exécute `map_saver_cli`, publie statut sur `/save_map_status` |

---

## Commandes de diagnostic rapide

```bash
# Vérifier que tous les topics arrivent
ros2 topic list | grep -E 'map|odom|scan|cmd_vel|camera|battery'

# Vérifier la fréquence du scan
ros2 topic hz /scan

# Vérifier que le robot reçoit les commandes vitesse
ros2 topic hz /cmd_vel

# Vérifier la chaîne cmd_vel Nav2
ros2 topic hz /cmd_vel_nav       # controller_server → smoother
ros2 topic hz /cmd_vel_smoothed  # smoother → relay
ros2 topic hz /cmd_vel           # relay → driver

# Tester un goal Nav2 depuis le terminal
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 1.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}"

# Vérifier l'état du lifecycle Nav2
ros2 lifecycle list

# Voir les TF disponibles
ros2 run tf2_tools view_frames

# Vérifier les noeuds actifs
ros2 node list
```
