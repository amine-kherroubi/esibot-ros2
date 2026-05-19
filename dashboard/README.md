# dashboard — EsiBot Dashboard React Source

Real-time web interface for the EsiBot robot (SLAM map, camera, teleoperation, Nav2 navigation).

This folder contains only the React **source code**. The compiled build is generated in `esibot_ui/web/` and served by `dashboard_node` (ROS2 package `esibot_ui`).

---

## Structure

```
dashboard/
├── src/
│   ├── components/     — React components (MapCanvas, Teleop, Camera…)
│   ├── hooks/          — ROS2 hooks (useMap, useOdom, useScan…)
│   ├── context/        — Rosbridge context (WebSocket connection)
│   ├── utils/          — Utilities (mapUtils: worldToCanvas, canvasToWorld…)
│   ├── styles/         — Global CSS
│   └── config.js       — Configuration (rosbridge URL, speeds, etc.)
├── package.json
└── vite.config.js
```

---

## Build and Deploy

After modifying the source code:

```bash
# 1. Remove old build
rm -rf ~/esibot_ws/src/esibot_ui/web/

# 2. Build
cd ~/esibot_ws/src/dashboard
npm run build

# 3. Copy build to ROS2 package
cp -r dist/ ../esibot_ui/web/

# 4. Rebuild ROS2 package
cd ~/esibot_ws
colcon build --packages-select esibot_ui
source install/setup.bash

# 5. Restart dashboard
ros2 launch esibot_ui dashboard.launch.py
```

---

## Configuration

Edit `src/config.js`:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `ROSBRIDGE_URL` | `ws://localhost:9090` | Robot IP address |
| `ROBOT_NAME` | `EsiBot` | Name displayed in the interface |
| `CMD_VEL.LINEAR_SPEED` | `0.4` | Teleop linear speed (m/s) |
| `CMD_VEL.ANGULAR_SPEED` | `1.5` | Teleop angular speed (rad/s) |
| `BATTERY_CAPACITY_MINUTES` | `45` | Estimated battery life (min) |
| `SCAN_OVERLAY` | `false` | Display LIDAR scan overlay on map |
