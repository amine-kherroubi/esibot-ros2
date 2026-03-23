"""
esibot_slam — launch/slam_sim.launch.py
========================================
All-in-one launch file: Gazebo Harmonic + SLAM + optional RViz2.

This is the recommended single-command launch for simulation demos and
development. It starts everything in the correct order with timed delays
to ensure each component is ready before the next one starts.

Launch sequence:
  t = 0 s  : Gazebo Harmonic starts, world esibot_world.sdf is loaded
  t = 4 s  : EsiBot robot is spawned in Gazebo (handled by sim.launch.py)
  t = 6 s  : relay_node starts  (/ultrasound_raw → /scan)
  t = 7 s  : async_slam_toolbox_node starts
  t = 7 s  : RViz2 starts (if use_rviz:=true)
  t = 8 s  : teleop_twist_keyboard starts (if teleop:=true)

Included launch files:
  • esibot_gazebo/sim.launch.py
    Launches: gz sim + robot_state_publisher + spawn + gz_bridge + Foxglove
    The gz_bridge maps Gazebo topics to ROS 2:
      /ultrasound_raw  ← gz.msgs.LaserScan   (from gpu_lidar sensor)
      /odom            ← gz.msgs.Odometry    (from diff_drive plugin)
      /tf              ← gz.msgs.Pose_V      (robot transform)
      /clock           ← gz.msgs.Clock       (simulation time)
      /cmd_vel         → gz.msgs.Twist       (motor commands)

Launch arguments:
  use_rviz     true | false   Launch RViz2 (default: true)
  use_foxglove true | false   Launch Foxglove bridge (default: true)
  teleop       true | false   Launch keyboard teleoperation (default: false)

Usage:
  # Full simulation demo:
  ros2 launch esibot_slam slam_sim.launch.py

  # With keyboard teleoperation (opens an xterm window):
  ros2 launch esibot_slam slam_sim.launch.py teleop:=true

  # Without RViz2 (use Foxglove at ws://localhost:8765 instead):
  ros2 launch esibot_slam slam_sim.launch.py use_rviz:=false

  # Without Foxglove:
  ros2 launch esibot_slam slam_sim.launch.py use_foxglove:=false

After mapping — save the map (Task deliverable #4):
  ros2 launch esibot_slam save_map.launch.py map_name:=esibot_map
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
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # ── Package directories ───────────────────────────────────────────────────
    slam_pkg   = get_package_share_directory("esibot_slam")
    gazebo_pkg = get_package_share_directory("esibot_gazebo")

    slam_params_sim = os.path.join(slam_pkg, "config", "slam_params_sim.yaml")
    rviz_config     = os.path.join(slam_pkg, "config", "esibot_slam.rviz")

    # ── Launch arguments ──────────────────────────────────────────────────────
    use_rviz_arg = DeclareLaunchArgument(
        "use_rviz",
        default_value="true",
        description="Launch RViz2 with the EsiBot SLAM visualization config.",
    )

    use_foxglove_arg = DeclareLaunchArgument(
        "use_foxglove",
        default_value="true",
        description=(
            "Launch the Foxglove WebSocket bridge (ws://localhost:8765). "
            "Open https://app.foxglove.dev to visualize the map in your browser."
        ),
    )

    teleop_arg = DeclareLaunchArgument(
        "teleop",
        default_value="false",
        description=(
            "Launch teleop_twist_keyboard in a separate xterm window. "
            "Requires xterm: sudo apt install xterm"
        ),
    )

    use_rviz     = LaunchConfiguration("use_rviz")
    use_foxglove = LaunchConfiguration("use_foxglove")
    teleop       = LaunchConfiguration("teleop")

    # ── [1] Gazebo Harmonic — include sim.launch.py from esibot_gazebo ────────
    #
    # This launch file is maintained by the esibot_gazebo team.
    # It handles: gz sim, robot spawning, gz_bridge, and Foxglove bridge.
    #
    # We pass use_foxglove through so the user's argument controls it here too.
    # use_sim_time is always true for simulation.
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_pkg, "launch", "sim.launch.py")
        ),
        launch_arguments={
            "use_foxglove": use_foxglove,
            "use_sim_time": "true",
        }.items(),
    )

    # ── [2] relay_node — t=6s ─────────────────────────────────────────────────
    #
    # Wait 6 s to ensure Gazebo has fully loaded the world and the robot
    # has been spawned (spawn happens at t≈4 s inside sim.launch.py).
    # The gpu_lidar sensor only starts publishing after the robot is spawned.
    relay_node = TimerAction(
        period=6.0,
        actions=[
            Node(
                package="topic_tools",
                executable="relay",
                name="ultrasound_to_scan_relay",
                output="screen",
                arguments=["/ultrasound_raw", "/scan"],
            ),
        ],
    )

    # ── [3] slam_toolbox — t=7s ───────────────────────────────────────────────
    #
    # Start 1 s after the relay to ensure /scan is already being published
    # before slam_toolbox subscribes to it.
    slam_toolbox = TimerAction(
        period=7.0,
        actions=[
            Node(
                package="slam_toolbox",
                executable="async_slam_toolbox_node",
                name="slam_toolbox",
                output="screen",
                parameters=[slam_params_sim],
                remappings=[
                    ("/scan", "/scan"),
                    ("/odom", "/odom"),
                ],
            ),
        ],
    )

    # ── [4] RViz2 — t=7s, optional ───────────────────────────────────────────
    #
    # Starts alongside slam_toolbox so the user can watch the map build
    # from the very first scan. Displays map, LaserScan, robot model, TF.
    rviz2 = TimerAction(
        period=7.0,
        actions=[
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", rviz_config],
                parameters=[{"use_sim_time": True}],
                condition=IfCondition(use_rviz),
            ),
        ],
    )

    # ── [5] Keyboard teleoperation — t=8s, optional ───────────────────────────
    #
    # Starts 1 s after SLAM to ensure /cmd_vel is properly connected.
    # Opens in a dedicated xterm terminal so the main terminal stays clean.
    # Controls: i=forward, ,=backward, j=turn left, l=turn right, k=stop
    teleop_node = TimerAction(
        period=8.0,
        actions=[
            Node(
                package="teleop_twist_keyboard",
                executable="teleop_twist_keyboard",
                name="teleop_twist_keyboard",
                output="screen",
                prefix="xterm -e",
                condition=IfCondition(teleop),
            ),
        ],
    )

    log_info = LogInfo(
        msg=[
            "\n",
            "╔══════════════════════════════════════════════════════════╗\n",
            "║       EsiBot SLAM — FULL SIMULATION (Gazebo + SLAM)     ║\n",
            "╠══════════════════════════════════════════════════════════╣\n",
            "║  t= 0 s  Gazebo Harmonic + esibot_world.sdf             ║\n",
            "║  t= 4 s  EsiBot robot spawned in Gazebo                 ║\n",
            "║  t= 6 s  relay_node: /ultrasound_raw → /scan            ║\n",
            "║  t= 7 s  async_slam_toolbox_node                        ║\n",
            "║  t= 7 s  RViz2 (if use_rviz:=true)                     ║\n",
            "║  t= 8 s  teleop (if teleop:=true)                       ║\n",
            "╠══════════════════════════════════════════════════════════╣\n",
            "║  Foxglove : ws://localhost:8765                         ║\n",
            "║  Save map : ros2 launch esibot_slam save_map.launch.py  ║\n",
            "╚══════════════════════════════════════════════════════════╝\n",
        ]
    )

    return LaunchDescription([
        use_rviz_arg,
        use_foxglove_arg,
        teleop_arg,
        log_info,
        gazebo_launch,
        relay_node,
        slam_toolbox,
        rviz2,
        teleop_node,
    ])
