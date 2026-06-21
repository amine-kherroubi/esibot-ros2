"""
EsiBot — full bringup launch file
===================================
Starts the complete EsiBot stack in one command.

Modes:
  mode:=slam    → build a new map (slam_toolbox active, Nav2 inactive)
  mode:=nav     → navigate on existing map (Nav2 active, slam_toolbox inactive)
  mode:=vision  → camera + vision node only (no SLAM, no Nav2)

Simulation:
  sim_mode:=true → no ESP32, no GPIO, odometry and scan are simulated

Usage:
  # Simulation + SLAM (build a map):
  ros2 launch esibot_description full.launch.py sim_mode:=true mode:=slam

  # Simulation + Nav2 (navigate on existing map):
  ros2 launch esibot_description full.launch.py sim_mode:=true mode:=nav

  # Camera + vision only (real ESP32-CAM required):
  ros2 launch esibot_description full.launch.py mode:=vision

  # Camera + vision with custom ESP32 IP:
  ros2 launch esibot_description full.launch.py mode:=vision esp32_ip:=10.55.37.10

  # Real hardware + SLAM:
  ros2 launch esibot_description full.launch.py mode:=slam

  # Real hardware + Nav2:
  ros2 launch esibot_description full.launch.py mode:=nav

Launch order (fixed):
  1. robot_state_publisher   → /robot_description, /tf_static           (all modes)
  2. foxglove_bridge         → WebSocket ws://localhost:8765             (all modes)
  2. dashboard        +1s    → HTTP :8080, rosbridge :9090               (all modes)
  3. esibot_driver    +2s    → /odom, /tf (odom→base_footprint)         (all modes)
  4. radar_node       +3s    → /scan, /joint_states                     (slam / nav)
  4. vision_node      +3s    → detections, annotated stream              (vision)
  5. slam_toolbox     +5s    → /map, /tf (map→odom)                     (slam)
  6. nav2             +35s   → navigation stack                          (nav)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    # ── Package paths ─────────────────────────────────────────────────────────
    desc_pkg = get_package_share_directory("esibot_description")
    bringup_pkg = get_package_share_directory("esibot_bringup")
    sensors_pkg = get_package_share_directory("esibot_sensors")
    slam_pkg = get_package_share_directory("esibot_slam")
    nav_pkg = get_package_share_directory("esibot_navigation")
    camera_pkg = get_package_share_directory("esibot_camera")
    vision_pkg = get_package_share_directory("esibot_vision")
    ui_pkg = get_package_share_directory("esibot_ui")

    default_urdf = os.path.join(desc_pkg, "urdf", "esibot.urdf.xacro")

    # ── Launch arguments ──────────────────────────────────────────────────────

    sim_mode_arg = DeclareLaunchArgument(
        "sim_mode",
        default_value="false",
        description=(
            "true  = simulation mode (no ESP32, no GPIO). " "false = real hardware."
        ),
    )

    mode_arg = DeclareLaunchArgument(
        "mode",
        default_value="slam",
        choices=["slam", "nav", "vision"],
        description=(
            "slam   = build a new map with slam_toolbox. "
            "nav    = navigate on an existing map with Nav2. "
            "vision = camera + vision node only."
        ),
    )

    use_foxglove_arg = DeclareLaunchArgument(
        "use_foxglove",
        default_value="false",
        description="Launch Foxglove WebSocket bridge on ws://localhost:8765",
    )

    esp32_ip_arg = DeclareLaunchArgument(
        "esp32_ip",
        default_value="10.55.37.10",
        description="ESP32-CAM IP address (vision mode only).",
    )

    sim_mode = LaunchConfiguration("sim_mode")
    mode = LaunchConfiguration("mode")
    use_foxglove = LaunchConfiguration("use_foxglove")
    esp32_ip = LaunchConfiguration("esp32_ip")

    is_slam = IfCondition(PythonExpression(["'", mode, "' == 'slam'"]))
    is_nav = IfCondition(PythonExpression(["'", mode, "' == 'nav'"]))
    is_vision = IfCondition(PythonExpression(["'", mode, "' == 'vision'"]))
    is_slam_or_nav = IfCondition(PythonExpression(["'", mode, "' in ['slam', 'nav']"]))

    # ── pigpiod — required for HC-SR04 µs-precision timing ──────────────────
    ensure_pigpiod = ExecuteProcess(
        cmd=["bash", "-c", "pgrep pigpiod || sudo pigpiod"],
        output="screen",
        condition=IfCondition(PythonExpression(["'", sim_mode, "' == 'false'"])),
    )

    # ── Robot description ─────────────────────────────────────────────────────
    robot_description = ParameterValue(
        Command([FindExecutable(name="xacro"), " ", default_urdf]),
        value_type=str,
    )

    # ── Startup log ───────────────────────────────────────────────────────────
    log_start = LogInfo(
        msg=[
            "\n",
            "=======================================================\n",
            "  EsiBot Full Bringup\n",
            "=======================================================\n",
            "  sim_mode : ",
            sim_mode,
            "\n",
            "  mode     : ",
            mode,
            "\n",
            "  foxglove : ws://localhost:8765\n",
            "=======================================================\n",
        ]
    )

    # ── Node 1: robot_state_publisher — all modes ─────────────────────────────
    robot_state_pub = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {"robot_description": robot_description},
            {"use_sim_time": False},
            {"publish_frequency": 50.0},
        ],
    )

    # ── Node 2: foxglove_bridge — all modes ───────────────────────────────────
    foxglove_bridge = Node(
        package="foxglove_bridge",
        executable="foxglove_bridge",
        name="foxglove_bridge",
        output="screen",
        parameters=[
            {"port": 8765},
            {"address": "0.0.0.0"},
            {"use_sim_time": False},
            {"send_buffer_limit": 10000000},
        ],
        condition=IfCondition(use_foxglove),
    )

    # ── Dashboard +1s — all modes ────────────────────────────────────────────
    # rosbridge (:9090) + HTTP dashboard (:8080) — starts before driver so the
    # UI is reachable as soon as the bringup is up.
    dashboard_launch = TimerAction(
        period=1.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(ui_pkg, "launch", "dashboard.launch.py")
                ),
            )
        ],
    )

    # ── Node 3: esibot_driver +2s — all modes ────────────────────────────────
    driver_launch = TimerAction(
        period=2.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(bringup_pkg, "launch", "bringup.launch.py")
                ),
                launch_arguments={"sim_mode": sim_mode}.items(),
            )
        ],
    )

    # ── Node 4a: radar_node +3s — slam and nav modes only ────────────────────
    radar_launch = TimerAction(
        period=3.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(sensors_pkg, "launch", "radar.launch.py")
                ),
                launch_arguments={"sim_mode": sim_mode}.items(),
            )
        ],
        condition=is_slam_or_nav,
    )

    # ── Node 5a: vision_node +3s — vision mode only ───────────────────────────
    # vision_node connects directly to ESP32-CAM MJPEG stream.
    # camera_stream_node is NOT launched — ESP32 only accepts one client.
    # sign_model_path left empty → obstacle-only (yolov8n.pt default).
    # lane_detection disabled — not needed for current use case.
    vision_launch = TimerAction(
        period=3.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(vision_pkg, "launch", "vision.launch.py")
                ),
                launch_arguments={
                    "esp32_ip":        esp32_ip,
                    "lane_detection":  "false",
                    "sign_model_path": "",
                }.items(),
            )
        ],
        condition=is_vision,
    )

    # ── Node 5b: slam_toolbox +5s — slam mode only ───────────────────────────
    # /odom (driver, +2s) and /scan (radar, +3s) must be publishing first.
    slam_launch = TimerAction(
        period=5.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(slam_pkg, "launch", "slam.launch.py")
                ),
                launch_arguments={"mode": "hw"}.items(),
            )
        ],
        condition=is_slam,
    )

    # ── Node 6: nav2 +35s — nav mode only ───────────────────────────────────
    # Pi 4 needs driver (+2s) and radar (+3s) fully running before nav2 starts.
    # AMCL must publish map→odom TF before global_costmap activates.
    # 35s gives localization time to converge on the Pi.
    nav_launch = TimerAction(
        period=35.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(nav_pkg, "launch", "nav2.launch.py")
                ),
                launch_arguments={"use_sim_time": "false"}.items(),
            )
        ],
        condition=is_nav,
    )

    return LaunchDescription(
        [
            sim_mode_arg,
            mode_arg,
            use_foxglove_arg,
            esp32_ip_arg,
            log_start,
            ensure_pigpiod,   # immediate — hw mode only
            robot_state_pub,  # immediate — all modes
            foxglove_bridge,  # immediate — all modes
            dashboard_launch, # +1s      — all modes
            driver_launch,    # +2s      — all modes
            radar_launch,     # +3s      — slam / nav only
            vision_launch,    # +3s      — vision only
            slam_launch,      # +5s      — slam only
            nav_launch,       # +35s     — nav only
        ]
    )
