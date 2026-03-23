"""
esibot_slam — launch/slam.launch.py
=====================================
Main SLAM launch file for the EsiBot robot (Task 3.5).

This file launches ONLY the SLAM-related nodes. It is designed to be
used either standalone (composed on top of a running sim/bringup) or
included from slam_sim.launch.py.

Available modes:
  sim (default) — Gazebo Harmonic simulation
    • use_sim_time = true  (reads /clock from Gazebo)
    • relay_node: forwards /ultrasound_raw → /scan
      (gz_bridge in sim.launch.py publishes the LaserScan on /ultrasound_raw;
       slam_toolbox and the rest of the stack expect /scan)

  hw — Real hardware (Raspberry Pi 4)
    • use_sim_time = false (wall clock)
    • static_transform_publisher: broadcasts a zero TF ultrasound_sensor → laser_link
      radar_node (esibot_sensors) uses frame_id='laser_link', but the EsiBot URDF
      defines the sensor link as 'ultrasound_sensor'. Without this fix, slam_toolbox
      raises a TF lookup error. The zero transform attaches 'laser_link' to the
      existing TF tree without touching the esibot_sensors team's code.

Nodes launched:
  1. relay_node            (topic_tools/relay)             — SIMULATION ONLY
     /ultrasound_raw → /scan

  2. static_tf_laser_link  (tf2_ros/static_transform_publisher) — HARDWARE ONLY
     ultrasound_sensor → laser_link  (zero transform / same position)

  3. async_slam_toolbox_node  (slam_toolbox)               — ALWAYS
     Reads /scan + /odom + /tf  →  publishes /map + TF map→odom

  4. rviz2                                                  — optional (use_rviz:=true)
     Loads config/esibot_slam.rviz

  5. teleop_twist_keyboard                                  — optional (teleop:=true)
     Opens in a separate xterm window for manual map building

Launch arguments:
  mode      sim | hw      Operating mode (default: sim)
  use_rviz  true | false  Launch RViz2 (default: false)
  teleop    true | false  Launch keyboard teleoperation (default: false)

Usage examples:
  # Simulation — Gazebo must already be running (sim.launch.py)
  ros2 launch esibot_slam slam.launch.py

  # Simulation with RViz2
  ros2 launch esibot_slam slam.launch.py use_rviz:=true

  # Simulation with RViz2 + keyboard teleoperation
  ros2 launch esibot_slam slam.launch.py use_rviz:=true teleop:=true

  # All-in-one (Gazebo + SLAM + RViz2 in one command)
  ros2 launch esibot_slam slam_sim.launch.py

  # Real hardware (esibot_bringup + esibot_sensors must already be running)
  ros2 launch esibot_slam slam.launch.py mode:=hw use_rviz:=true teleop:=true

Runtime prerequisites:
  sim : ros2 launch esibot_gazebo sim.launch.py must be running
  hw  : esibot_bringup (esibot_driver) + esibot_sensors (radar_node) must be running
        Verify: ros2 topic hz /scan   → should show ~1–2 Hz
                ros2 topic hz /odom   → should show ~10+ Hz
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():

    # ── Package directories ───────────────────────────────────────────────────
    slam_pkg = get_package_share_directory("esibot_slam")

    slam_params_sim = os.path.join(slam_pkg, "config", "slam_params_sim.yaml")
    slam_params_hw  = os.path.join(slam_pkg, "config", "slam_params_hw.yaml")
    rviz_config     = os.path.join(slam_pkg, "config", "esibot_slam.rviz")

    # ── Launch arguments ──────────────────────────────────────────────────────
    mode_arg = DeclareLaunchArgument(
        "mode",
        default_value="sim",
        choices=["sim", "hw"],
        description=(
            "Operating mode. "
            "'sim' = Gazebo Harmonic simulation (use_sim_time=true, relay /ultrasound_raw→/scan). "
            "'hw'  = Real Raspberry Pi 4 hardware (use_sim_time=false, /scan direct from radar_node)."
        ),
    )

    use_rviz_arg = DeclareLaunchArgument(
        "use_rviz",
        default_value="false",
        description="Launch RViz2 with the EsiBot SLAM configuration.",
    )

    teleop_arg = DeclareLaunchArgument(
        "teleop",
        default_value="false",
        description=(
            "Launch teleop_twist_keyboard in a separate xterm window "
            "for manual robot control during map building. "
            "Requires xterm: sudo apt install xterm"
        ),
    )

    mode     = LaunchConfiguration("mode")
    use_rviz = LaunchConfiguration("use_rviz")
    teleop   = LaunchConfiguration("teleop")

    # PythonExpression conditions — compatible with all ROS 2 versions (Galactic+).
    # Evaluates the string at launch time, not at import time.
    is_sim = IfCondition(PythonExpression(["'", mode, "' == 'sim'"]))
    is_hw  = IfCondition(PythonExpression(["'", mode, "' == 'hw'"]))

    # ── [1] relay_node — SIMULATION ONLY ────────────────────────────────────
    #
    # Why this node is needed:
    #   In esibot_gazebo/sim.launch.py, gz_bridge is configured with:
    #     "/ultrasound_raw@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan"
    #   This means Gazebo's gpu_lidar sensor data arrives on /ultrasound_raw in ROS 2.
    #
    #   All SLAM and navigation nodes (slam_toolbox, Nav2) expect the standard
    #   /scan topic. This relay node silently copies every message from
    #   /ultrasound_raw to /scan with no modification.
    #
    #   On hardware: radar_node (esibot_sensors) publishes /scan directly.
    #   → This node is NOT launched in hw mode.
    relay_node = Node(
        package="topic_tools",
        executable="relay",
        name="ultrasound_to_scan_relay",
        output="screen",
        arguments=["/ultrasound_raw", "/scan"],
        condition=is_sim,
    )

    # ── [2] static_tf_laser_link — HARDWARE ONLY ─────────────────────────────
    #
    # Why this node is needed:
    #   radar_node.py (esibot_sensors) publishes the LaserScan with:
    #     msg.header.frame_id = 'laser_link'
    #   But the EsiBot URDF (esibot_description) defines the sensor link as:
    #     <link name="ultrasound_sensor"> (child of servo_link)
    #
    #   slam_toolbox needs to find the transform chain:
    #     base_footprint → ... → laser_link
    #   Since 'laser_link' doesn't exist in the URDF, TF lookup fails.
    #
    # Fix: publish a static zero-transform
    #     parent = ultrasound_sensor  (exists in URDF TF tree)
    #     child  = laser_link         (used by radar_node)
    #     x=0, y=0, z=0, roll=0, pitch=0, yaw=0
    #   This "attaches" laser_link to the existing TF tree at the exact same
    #   position as ultrasound_sensor, without modifying any other team's code.
    #
    # Long-term fix: update radar_node.py to use frame_id='ultrasound_sensor'.
    #
    # In simulation: the Gazebo gpu_lidar sensor automatically uses the URDF
    # link name 'ultrasound_sensor' as its frame_id → no fix needed in sim mode.
    static_tf_laser_link = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_tf_ultrasound_to_laser_link",
        output="screen",
        # Arguments format: x y z yaw pitch roll parent_frame child_frame
        arguments=["0", "0", "0", "0", "0", "0",
                   "ultrasound_sensor", "laser_link"],
        condition=is_hw,
    )

    # ── [3a] slam_toolbox — SIMULATION MODE ──────────────────────────────────
    #
    # Node: async_slam_toolbox_node
    #   Asynchronous online SLAM — processes scans as they arrive without blocking.
    #   This is the correct mode for a slow, irregular sensor like the HC-SR04 radar.
    #
    # Input topics consumed:
    #   /scan   (sensor_msgs/LaserScan)  — relayed from /ultrasound_raw in sim
    #   /odom   (nav_msgs/Odometry)      — differential odometry from esibot_driver
    #   /tf     (TF tree)                — base_footprint→odom from robot_state_publisher
    #
    # Output produced:
    #   /map         (nav_msgs/OccupancyGrid)       — 2D occupancy map (live)
    #   /map_updates (map_msgs/OccupancyGridUpdate) — incremental map updates
    #   TF map→odom                                 — robot localization in the map
    #   /slam_toolbox/* services                    — save/load/serialize map
    #
    # Parameters: slam_params_sim.yaml (use_sim_time=true, faster update cycle)
    slam_toolbox_sim = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        parameters=[slam_params_sim],
        remappings=[
            ("/scan", "/scan"),
            ("/odom", "/odom"),
        ],
        condition=is_sim,
    )

    # ── [3b] slam_toolbox — HARDWARE MODE ────────────────────────────────────
    #
    # Same node, different parameter file.
    # Parameters: slam_params_hw.yaml
    #   use_sim_time: false        → wall clock
    #   minimum_time_interval: 0.5 → HC-SR04 radar is slow (1–2 Hz)
    #   map_update_interval: 5.0   → save Raspberry Pi 4 CPU resources
    #   transform_timeout: 1.0     → account for UART/WiFi latency
    slam_toolbox_hw = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        parameters=[slam_params_hw],
        remappings=[
            ("/scan", "/scan"),
            ("/odom", "/odom"),
        ],
        condition=is_hw,
    )

    # ── [4] RViz2 — OPTIONAL ─────────────────────────────────────────────────
    #
    # Loads config/esibot_slam.rviz which displays:
    #   • SLAM map (/map)              — white=free, black=obstacle, grey=unknown
    #   • LaserScan (/scan)            — red squares showing radar sweep points
    #   • Robot model (/robot_description) — EsiBot 3D URDF model
    #   • TF tree                      — all frames: map→odom→base_footprint→...
    #   • Odometry (/odom)             — current position and orientation
    #
    # Tip: In simulation, Foxglove (ws://localhost:8765) is often more convenient
    # because it works from a browser without display server issues on WSL2.
    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(use_rviz),
    )

    # ── [5] Keyboard Teleoperation — OPTIONAL ─────────────────────────────────
    #
    # Launches teleop_twist_keyboard in a separate xterm terminal.
    # Publishes geometry_msgs/Twist on /cmd_vel.
    # Controls: i=forward, , (comma)=backward, j=left, l=right, k=stop
    #
    # Requirement: sudo apt install xterm
    # Alternative (without xterm): open a new terminal and run:
    #   ros2 run teleop_twist_keyboard teleop_twist_keyboard
    teleop_node = Node(
        package="teleop_twist_keyboard",
        executable="teleop_twist_keyboard",
        name="teleop_twist_keyboard",
        output="screen",
        remappings=[("/cmd_vel", "/cmd_vel")],
        prefix="xterm -e",
        condition=IfCondition(teleop),
    )

    # ── Startup info logs ─────────────────────────────────────────────────────
    log_sim = LogInfo(
        msg=[
            "\n",
            "=======================================================\n",
            "  EsiBot SLAM — SIMULATION MODE (Gazebo Harmonic)\n",
            "=======================================================\n",
            "  Nodes launched:\n",
            "    • relay_node   : /ultrasound_raw → /scan\n",
            "    • slam_toolbox : async_slam_toolbox_node\n",
            "  Config: slam_params_sim.yaml  |  use_sim_time: true\n",
            "\n",
            "  Prerequisite: ros2 launch esibot_gazebo sim.launch.py\n",
            "  Or use all-in-one: ros2 launch esibot_slam slam_sim.launch.py\n",
            "=======================================================\n",
        ],
        condition=is_sim,
    )

    log_hw = LogInfo(
        msg=[
            "\n",
            "=======================================================\n",
            "  EsiBot SLAM — HARDWARE MODE (Raspberry Pi 4)\n",
            "=======================================================\n",
            "  Nodes launched:\n",
            "    • static_tf    : ultrasound_sensor → laser_link\n",
            "    • slam_toolbox : async_slam_toolbox_node\n",
            "  Config: slam_params_hw.yaml  |  use_sim_time: false\n",
            "\n",
            "  Prerequisites:\n",
            "    ros2 launch esibot_bringup bringup.launch.py\n",
            "  Verify topics:\n",
            "    ros2 topic hz /scan   # should show ~1–2 Hz\n",
            "    ros2 topic hz /odom   # should show ~10+ Hz\n",
            "=======================================================\n",
        ],
        condition=is_hw,
    )

    return LaunchDescription([
        # Declare arguments first
        mode_arg,
        use_rviz_arg,
        teleop_arg,

        # Startup info
        log_sim,
        log_hw,

        # Mode-conditional nodes
        relay_node,            # sim only: relay /ultrasound_raw → /scan
        static_tf_laser_link,  # hw only:  fix TF frame mismatch

        # SLAM — only one instance active depending on mode
        slam_toolbox_sim,
        slam_toolbox_hw,

        # Optional tools
        rviz2,
        teleop_node,
    ])
