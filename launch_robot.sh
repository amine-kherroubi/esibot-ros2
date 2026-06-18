#!/usr/bin/env bash
# Launch all EsiBot nodes on the Raspberry Pi via SSH.
# Usage: ./launch_robot.sh [PI_IP] [MODE]
#   PI_IP  defaults to 10.196.185.99
#   MODE   mapping (default) | navigation
set -euo pipefail

PI_IP="${1:-10.196.185.99}"
MODE="${2:-mapping}"
PI_USER="esibot"
PI_PASS="esibot"
WS="source /home/esibot/robot_ws/install/setup.bash"
SESSION="esibot"

if [[ "$MODE" != "mapping" && "$MODE" != "navigation" ]]; then
  echo "ERROR: MODE must be 'mapping' or 'navigation', got: $MODE"
  exit 1
fi

ssh_run() {
  sshpass -p "$PI_PASS" ssh -o ConnectTimeout=10 -o ServerAliveInterval=10 \
    "$PI_USER@$PI_IP" "bash -lc '$1'"
}

wait_for_topic() {
  local topic="$1"
  local label="$2"
  local timeout=20
  echo "  Waiting for $label ..."
  sshpass -p "$PI_PASS" ssh -o ConnectTimeout=30 "$PI_USER@$PI_IP" bash << ENDSSH
source /home/esibot/robot_ws/install/setup.bash > /dev/null 2>&1
deadline=\$((SECONDS + $timeout))
while ! ros2 topic list 2>/dev/null | grep -qF "$topic"; do
  if [ \$SECONDS -ge \$deadline ]; then
    echo "  TIMEOUT waiting for $topic after ${timeout}s"
    exit 1
  fi
  sleep 1
done
echo "  $topic ready"
ENDSSH
}

echo "==> Mode    : $MODE"
echo "==> Target  : $PI_USER@$PI_IP"
echo ""

echo "==> Connecting ..."
ssh_run "echo connected"

echo "==> Starting pigpiod ..."
sshpass -p "$PI_PASS" ssh -o ConnectTimeout=10 "$PI_USER@$PI_IP" bash << 'ENDSSH'
if pgrep pigpiod > /dev/null; then
  echo "  pigpiod already running"
else
  sudo pigpiod && sleep 1
  if pgrep pigpiod > /dev/null; then
    echo "  pigpiod started OK"
  else
    echo "  WARNING: pigpiod failed to start — servo/timing accuracy reduced"
  fi
fi
ENDSSH

echo "==> Killing stale ROS processes ..."
ssh_run "pkill -f esibot_driver 2>/dev/null; pkill -f ros2 2>/dev/null; sleep 1; echo cleanup done" || true

echo "==> Resetting tmux session '$SESSION' ..."
ssh_run "tmux kill-session -t $SESSION 2>/dev/null; true"
sleep 1

# ── Step 1: full robot stack (robot_state_publisher + driver + radar + slam/nav)
ROS_MODE="slam"
[[ "$MODE" == "navigation" ]] && ROS_MODE="nav"

echo "==> [1/2] Robot stack (full.launch.py mode:=$ROS_MODE) ..."
sshpass -p "$PI_PASS" ssh -o ConnectTimeout=10 "$PI_USER@$PI_IP" "
  tmux new-session -d -s $SESSION -n robot -x 220 -y 50
  tmux send-keys -t $SESSION:robot '$WS && ros2 launch esibot_description full.launch.py mode:=$ROS_MODE use_foxglove:=false' Enter
"
wait_for_topic "/odom" "/odom (driver ready, ~7s)"
wait_for_topic "/tf"   "/tf   (TF tree ready)"

# ── Step 2: dashboard (always last) ────────────────────────────────────────
echo "==> [2/2] Dashboard + web bridge ..."
sshpass -p "$PI_PASS" ssh -o ConnectTimeout=10 "$PI_USER@$PI_IP" "
  tmux new-window -t $SESSION -n dashboard
  tmux send-keys -t $SESSION:dashboard '$WS && ros2 launch esibot_ui dashboard.launch.py' Enter
"

echo ""
echo "==> All nodes launched in $MODE mode."
echo ""
echo "    Live logs  : ssh $PI_USER@$PI_IP  →  tmux attach -t $SESSION"
echo "    Dashboard  : http://$PI_IP:8080"
echo "    WS bridge  : ws://$PI_IP:9090"
echo ""
if [[ "$MODE" == "mapping" ]]; then
  echo "    tmux windows : 0:robot  1:dashboard"
  echo ""
  echo "    When mapping is done, save the map from the dashboard,"
  echo "    then relaunch in navigation mode:"
  echo "      ./launch_robot.sh $PI_IP navigation"
else
  echo "    tmux windows : 0:robot  1:dashboard"
fi
