#!/bin/bash
# stop_robot.sh - Clean shutdown of all EsiBot ROS2 nodes
# Usage: ./stop_robot.sh
#   or:  ssh esibot@<pi-ip> 'bash -s' < stop_robot.sh

echo "=== EsiBot Clean Shutdown ==="

# 1. Send SIGTERM to all ROS-related processes
echo "[1/5] Sending SIGTERM..."
pkill -15 -f 'ros2' 2>/dev/null
pkill -15 -f 'rosapi' 2>/dev/null
pkill -15 -f 'rosbridge' 2>/dev/null
pkill -15 -f 'foxglove_bridge' 2>/dev/null
pkill -15 -f 'component_container' 2>/dev/null
pkill -15 -f 'slam_toolbox' 2>/dev/null
pkill -15 -f 'amcl' 2>/dev/null
pkill -15 -f 'map_server' 2>/dev/null
pkill -15 -f 'lifecycle_manager' 2>/dev/null
pkill -15 -f 'controller_server' 2>/dev/null
pkill -15 -f 'planner_server' 2>/dev/null
pkill -15 -f 'bt_navigator' 2>/dev/null
pkill -15 -f 'velocity_smoother' 2>/dev/null
pkill -15 -f 'collision_monitor' 2>/dev/null
pkill -15 -f 'behavior_server' 2>/dev/null
pkill -15 -f 'smoother_server' 2>/dev/null
pkill -15 -f 'waypoint_follower' 2>/dev/null
pkill -15 -f 'docking_server' 2>/dev/null
pkill -15 -f 'route_server' 2>/dev/null
pkill -15 -f 'vision_node' 2>/dev/null
pkill -15 -f 'camera_node' 2>/dev/null
pkill -15 -f 'dashboard_node' 2>/dev/null
pkill -15 -f 'map_saver_node' 2>/dev/null
pkill -15 -f 'nav_goal_proxy' 2>/dev/null
pkill -15 -f 'esibot_driver' 2>/dev/null
pkill -15 -f 'radar_node' 2>/dev/null
fuser -k 8080/tcp 9090/tcp 2>/dev/null

# 2. Wait for graceful shutdown
echo "[2/5] Waiting 3s for graceful shutdown..."
sleep 3

# 3. Force kill ALL python3 and ROS processes (except system ones)
echo "[3/5] Force killing all remaining ROS/python3 processes..."
pkill -9 -f 'ros2' 2>/dev/null
pkill -9 -f 'rosapi' 2>/dev/null
pkill -9 -f 'rosbridge' 2>/dev/null
pkill -9 -f 'foxglove_bridge' 2>/dev/null
for PID in $(ps aux | grep python3 | grep -v grep | grep -v networkd | grep -v unattended | awk '{print $2}'); do
    kill -9 "$PID" 2>/dev/null
done
sleep 1

# 4. Clean up DDS shared memory
echo "[4/5] Cleaning DDS shared memory..."
rm -f /dev/shm/fastrtps_* 2>/dev/null
rm -f /dev/shm/Fast-RTPS-* 2>/dev/null

# 5. Verify
STILL_RUNNING=$(ps aux | grep -E 'ros2|python3' | grep -v grep | grep -v networkd | grep -v unattended | grep -v stop_robot || true)
if [ -z "$STILL_RUNNING" ]; then
    echo "[5/5] Verified clean."
    echo ""
    echo "=== All EsiBot nodes stopped cleanly ==="
else
    echo "[5/5] WARNING: Some processes still running:"
    echo "$STILL_RUNNING"
fi
