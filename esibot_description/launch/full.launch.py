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
  ros2 launch esibot_bringup full_bringup.launch.py sim_mode:=true mode:=slam

  # Simulation + Nav2 (navigate on existing map):
  ros2 launch esibot_bringup full_bringup.launch.py sim_mode:=true mode:=nav

  # Camera + vision only (real ESP32-CAM required):
  ros2 launch esibot_bringup full_bringup.launch.py mode:=vision

  # Camera + vision in sim mode:
  ros2 launch esibot_bringup full_bringup.launch.py sim_mode:=true mode:=vision

  # Real hardware + SLAM:
  ros2 launch esibot_bringup full_bringup.launch.py mode:=slam

  # Real hardware + Nav2:
  ros2 launch esibot_bringup full_bringup.launch.py mode:=nav

Launch order (fixed):
  1. robot_state_publisher   → /robot_description, /tf_static           (all modes)
  2. foxglove_bridge         → WebSocket ws://localhost:8765             (all modes)
  3. esibot_driver    +2s    → /odom, /tf (odom→base_footprint)         (all modes)
  4. radar_node       +3s    → /scan, /joint_states                     (slam / nav)
  4. camera_node      +3s    → /camera/image_raw, /camera/camera_info   (vision)
  5. vision_node      +4s    → detections, annotated stream              (vision)
  5. slam_toolbox     +5s    → /map, /tf (map→odom)                     (slam)
  6. nav2             +5s    → navigation stack                          (nav)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    # ── Package paths ─────────────────────────────────────────────────────────
    desc_pkg    = get_package_share_directory("esibot_description")
    bringup_pkg = get_package_share_directory("esibot_bringup")
    sensors_pkg = get_package_share_directory("esibot_sensors")
    slam_pkg    = get_package_share_directory("esibot_slam")
    nav_pkg     = get_package_share_directory("esibot_navigation")
    camera_pkg  = get_package_share_directory("esibot_camera")
    vision_pkg  = get_package_share_directory("esibot_vision")

    default_urdf = os.path.join(desc_pkg, "urdf", "esibot.urdf.xacro")

    # ── Launch arguments ──────────────────────────────────────────────────────

    sim_mode_arg = DeclareLaunchArgument(
        "sim_mode",
        default_value="false",
        description=(
            "true  = simulation mode (no ESP32, no GPIO). "
            "false = real hardware."
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
        default_value="true",
        description="Launch Foxglove WebSocket bridge on ws://localhost:8765",
    )

    sim_mode     = LaunchConfiguration("sim_mode")
    mode         = LaunchConfiguration("mode")
    use_foxglove = LaunchConfiguration("use_foxglove")

    is_slam   = IfCondition(PythonExpression(["'", mode, "' == 'slam'"]))
    is_nav    = IfCondition(PythonExpression(["'", mode, "' == 'nav'"]))
    is_vision = IfCondition(PythonExpression(["'", mode, "' == 'vision'"]))
    is_slam_or_nav = IfCondition(
        PythonExpression(["'", mode, "' in ['slam', 'nav']"])
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
            "  sim_mode : ", sim_mode, "\n",
            "  mode     : ", mode, "\n",
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

    # ── Node 4b: camera_node +3s — vision mode only ───────────────────────────
    # camera_link is in /tf_static from robot_state_publisher (Node 1).
    # 3s delay is sufficient for that to be published.
    camera_launch = TimerAction(
        period=3.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(camera_pkg, "launch", "camera.launch.py")
                ),
                launch_arguments={"sim_mode": sim_mode}.items(),
            )
        ],
        condition=is_vision,
    )

    # ── Node 5a: vision_node +4s — vision mode only ───────────────────────────
    # 1s after camera_node to ensure /camera/image_raw is publishing
    # before vision_node tries to subscribe.
    vision_launch = TimerAction(
        period=4.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(vision_pkg, "launch", "vision.launch.py")
                ),
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

    # ── Node 6: nav2 +5s — nav mode only ────────────────────────────────────
    nav_launch = TimerAction(
        period=5.0,
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

    return LaunchDescription([
        sim_mode_arg,
        mode_arg,
        use_foxglove_arg,
        log_start,
        robot_state_pub,    # immediate — all modes
        foxglove_bridge,    # immediate — all modes
        driver_launch,      # +2s      — all modes
        radar_launch,       # +3s      — slam / nav only
        camera_launch,      # +3s      — vision only
        vision_launch,      # +4s      — vision only
        slam_launch,        # +5s      — slam only
        nav_launch,         # +5s      — nav only
    ])