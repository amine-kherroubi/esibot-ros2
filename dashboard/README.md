# dashboard — Code source React du Dashboard EsiBot

Interface web temps réel pour le robot EsiBot (carte SLAM, caméra, téléopération, navigation Nav2).

Ce dossier contient uniquement le **code source** React. Le build compilé est généré dans `esibot_ui/web/` et servi par `dashboard_node` (package ROS2 `esibot_ui`).

---

## Structure

```
dashboard/
├── src/
│   ├── components/     — Composants React (MapCanvas, Teleop, Camera…)
│   ├── hooks/          — Hooks ROS2 (useMap, useOdom, useScan…)
│   ├── context/        — Contexte rosbridge (connexion WebSocket)
│   ├── utils/          — Utilitaires (mapUtils: worldToCanvas, canvasToWorld…)
│   ├── styles/         — CSS global
│   └── config.js       — Configuration (URL rosbridge, vitesses, etc.)
├── package.json
└── vite.config.js
```

---

## Compiler et déployer

Après chaque modification du code source :

```bash
# 1. Supprimer l'ancien build
rm -rf ~/esibot_ws/src/esibot_ui/web/

# 2. Compiler
cd ~/esibot_ws/src/dashboard
npm run build

# 3. Copier le build vers le package ROS2
cp -r dist/ ../esibot_ui/web/

# 4. Rebuilder le package ROS2
cd ~/esibot_ws
colcon build --packages-select esibot_ui
source install/setup.bash

# 5. Relancer le dashboard
ros2 launch esibot_ui dashboard.launch.py
```

---

## Configuration

Modifier `src/config.js` :

| Paramètre | Défaut | Description |
|-----------|--------|-------------|
| `ROSBRIDGE_URL` | `ws://localhost:9090` | Adresse IP du robot |
| `ROBOT_NAME` | `EsiBot` | Nom affiché dans l'interface |
| `CMD_VEL.LINEAR_SPEED` | `0.4` | Vitesse linéaire téléop (m/s) |
| `CMD_VEL.ANGULAR_SPEED` | `1.5` | Vitesse angulaire téléop (rad/s) |
| `BATTERY_CAPACITY_MINUTES` | `45` | Autonomie estimée (min) |
| `SCAN_OVERLAY` | `false` | Afficher le scan LIDAR sur la carte |

---

## Notes

- `dist/` est généré par `npm run build` — **ne pas committer** (exclu via `.gitignore`)
- `node_modules/` — **ne pas committer** (exclu via `.gitignore`)
- Le build compilé `esibot_ui/web/` est également exclu du git
