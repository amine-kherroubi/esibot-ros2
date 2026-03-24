# esibot_sensors  
Package ROS2 — Tâche 3.3 : Pseudo-LiDAR (Servo + HC-SR04)  
ROS2 Jazzy | Ubuntu 24.04  

---

## Description  

Ce package implémente un **LiDAR virtuel (pseudo-LiDAR)** en utilisant :

- Un capteur ultrason HC-SR04  
- Un servo-moteur SG90  

Le servo effectue un balayage de **0° à 180°**, et le capteur mesure la distance à chaque angle.  
Les données sont converties en message ROS2 `LaserScan`, compatible avec SLAM et Nav2.


Servo (angle θ)
↓
HC-SR04 (distance)
↓
radar_node
↓
/scan (LaserScan)
↓
SLAM / Navigation


---

## Principe de fonctionnement  

1. Rotation du servo (0° → 180°)  
2. Mesure de distance à chaque angle  
3. Stockage des données (angle, distance)  
4. Construction du message `LaserScan`  
5. Publication sur `/scan`  

---

## 📦 Prérequis  

- Ubuntu 24.04  
- ROS2 Jazzy  
- Python 3  

Option hardware (Raspberry Pi) :  

pip install RPi.GPIO


---

## Installation  

### 1. Installer les dépendances

cd ~/robot_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y

### 2. Compiler le workspace
cd ~/robot_ws
colcon build --symlink-install --packages-select esibot_sensors
source install/setup.bash

---

## Lancement  

### Lancer le nœud  

ros2 run esibot_sensors radar_node  

---

## Comportement  

Le code détecte automatiquement l’environnement :

- **Sans GPIO (PC)** → Mode simulation  
  → Génération de distances aléatoires (~1 m)  

- **Avec GPIO (Raspberry Pi)** → Mode réel  
  → Utilisation du servo + capteur HC-SR04  


---

## Câblage  

| Composant | Connexion |
|----------|----------|
| Servo signal | GPIO 17 |
| HC-SR04 TRIG | GPIO 27 |
| HC-SR04 ECHO | GPIO 22 |
| VCC | 5V |
| GND | GND |

Les GPIO du Raspberry Pi sont en 3.3V alors que le HC-SR04 envoie 5V sur ECHO  
→ utiliser un **diviseur de tension**  

---

## Topics  

| Topic | Type | Description |
|------|------|------------|
| /scan | sensor_msgs/LaserScan | Données du pseudo-LiDAR |

---

## Paramètres du scan  

| Paramètre        | Valeur  | Description |
|------------------|--------|-------------|
| angle_min        | 0 rad  | Angle de départ du balayage (correspond à 0°, début du scan) |
| angle_max        | π rad  | Angle final du balayage (180°, limite du servo) |
| angle_increment  | 10°    | Pas angulaire entre deux mesures successives du capteur |
| range_min        | 0.02 m | Distance minimale mesurable par le capteur (2 cm) |
| range_max        | 4.0 m  | Distance maximale mesurable par le capteur (4 m) |
| fréquence        | 1 Hz   | Nombre de scans complets effectués par seconde |

---

## Mode TEST vs RÉEL  

| Élément | Test | Réel |
|--------|------|------|
| Servo | simulé | réel |
| Distance | entrée clavier | capteur HC-SR04 |
| Topic /scan | oui | oui |

---

## Structure du package  

esibot_sensors/
├── package.xml
├── setup.py
├── setup.cfg
├── README.md
├── resource/
│ └── esibot_sensors
├── esibot_sensors/
│ ├── init.py
│ └── radar_node.py


---

## Différence avec un LiDAR réel  

| Critère | LiDAR | EsiBot |
|--------|------|--------|
| Technologie | Laser | Ultrasons |
| Rotation | 360° | 180° |
| Coût | élevé | faible |

---

## Tests  

Vérifier les topics :

ros2 topic list
ros2 topic echo /scan
 
La première commande permet de vérifier les topics actifs. Le topic /scan doit apparaître, ce qui confirme que le node du capteur publie correctement. Et la deuxième affiche en temps réel les données du capteur sur /scan, permettant de vérifier que les valeurs de distance sont bien envoyées et mises à jour.

### Visualisation :
en utilisant l'outil de visualisation des données ROS2 "RViz2"

ros2 run rviz2 rviz2

### Configuration dans RViz2 :
Ajouter un display **LaserScan** avec le topic `/scan`.
Cela permet d’afficher les mesures du capteur sous forme de scan (type radar) en temps réel.
---

## Intégration  

Ce nœud est utilisé par :

- slam_toolbox (cartographie)  
- Nav2 (navigation autonome)  

---
