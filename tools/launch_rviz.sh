#!/bin/bash
# Launch RViz2 configured for EsiBot — connects to Pi over LAN.
#
# Usage:
#   ./tools/launch_rviz.sh                    # Pi at default IP
#   PI_IP=192.168.1.42 ./tools/launch_rviz.sh # override IP

PI_IP="${PI_IP:-10.83.158.99}"
RVIZ_CFG="$(cd "$(dirname "$0")/.." && pwd)/esibot_bringup/config/esibot_rviz.rviz"

source /opt/ros/humble/setup.bash

export ROS_DOMAIN_ID=0
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
export ROS_STATIC_PEERS="$PI_IP"

echo "Connecting to Pi at $PI_IP (domain $ROS_DOMAIN_ID)"
echo "Waiting for topics..."

# Quick check — list topics and show what's reachable
timeout 4 ros2 topic list 2>/dev/null | grep -E "/scan|/map|/odom|/tf" \
  && echo "Pi topics visible — launching RViz2" \
  || echo "Warning: Pi topics not yet visible (Pi may still be booting)"

rviz2 -d "$RVIZ_CFG"
