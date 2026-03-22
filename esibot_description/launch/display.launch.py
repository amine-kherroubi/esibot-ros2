"""
EsiBot Display Launch File
===========================
Purpose: visualize the robot URDF / TF tree WITHOUT launching Gazebo.
         Use this to:
           • check that the URDF parses correctly after edits
           • inspect the TF tree and all link positions in Foxglove
           • manually sweep the servo_joint slider to verify the turret rotates
           • verify the URDF before running the full simulation

What it launches:
  1. robot_state_publisher  — reads URDF, publishes /robot_description + static /tf
  2. joint_state_publisher_gui — slider window for every non-fixed joint
  3. foxglove_bridge (optional) — WebSocket bridge for browser visualization

What it does NOT launch:
  • Gazebo (no physics, no world)
  • SLAM or Nav2

Usage:
  # Default (Foxglove bridge ON — because RViz2 has WSL2 display issues)
  ros2 launch esibot_description display.launch.py

  # Disable Foxglove bridge (if you want to use RViz2 natively)
  ros2 launch esibot_description display.launch.py use_foxglove:=false

  # Pass a custom URDF/xacro file for quick testing
  ros2 launch esibot_description display.launch.py \
      urdf_file:=/path/to/your/test.urdf.xacro

Then open: https://app.foxglove.dev → Open Connection → ws://localhost:8765
  Add panels:
    • "3D"         → see the robot model and TF frames
    • "Topic List" → verify all expected topics are present
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # ── Default URDF path ─────────────────────────────────────────────────────
    default_urdf = os.path.join(
        get_package_share_directory("esibot_description"),
        "urdf",
        "esibot.urdf.xacro",
    )

    # ── Launch arguments ──────────────────────────────────────────────────────
    urdf_file_arg = DeclareLaunchArgument(
        "urdf_file",
        default_value=default_urdf,
        description="Absolute path to the URDF or xacro file to display",
    )

    use_foxglove_arg = DeclareLaunchArgument(
        "use_foxglove",
        default_value="true",
        description=(
            "Launch the Foxglove WebSocket bridge. "
            "Set to false if you want to use RViz2 instead."
        ),
    )

    use_rviz_arg = DeclareLaunchArgument(
        "use_rviz",
        default_value="false",
        description="Launch RViz2. Note: may not work well in WSL2.",
    )

    use_foxglove = LaunchConfiguration("use_foxglove")
    use_rviz     = LaunchConfiguration("use_rviz")
    urdf_file    = LaunchConfiguration("urdf_file")

    # ── Robot description: xacro → URDF string ────────────────────────────────
    # Command() runs at launch time, not at import time.
    # The result is a string containing the full parsed URDF XML.
    robot_description = ParameterValue(
        Command([FindExecutable(name="xacro"), " ", urdf_file]),
        value_type=str
    )

    # ── Nodes ─────────────────────────────────────────────────────────────────

    # 1. robot_state_publisher
    #    • Reads robot_description (URDF) and publishes:
    #        /robot_description  (latched string topic)
    #        /tf_static          (all fixed joints, once)
    #        /tf                 (moving joints, updated from /joint_states)
    #    • use_sim_time=false because there's no Gazebo running here
    robot_state_pub = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {"robot_description": robot_description},
            {"use_sim_time": False},
            # Publish TF at 50 Hz (default is 50, explicit is clearer)
            {"publish_frequency": 50.0},
        ],
    )

    # 2. joint_state_publisher_gui
    #    • Opens a slider window for every non-fixed joint in the URDF.
    #    • EsiBot's only non-fixed joints: left_wheel, right_wheel, servo_joint
    #    • Moving the servo_joint slider → turret rotates in Foxglove/RViz2
    #    • Publishes on /joint_states which robot_state_publisher subscribes to
    joint_state_pub_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        output="screen",
        parameters=[{"use_sim_time": False}],
    )

    # 3. Foxglove bridge
    #    • WebSocket server at ws://localhost:8765
    #    • accessible from Windows browser (WSL2 port forwarding is automatic)
    #    • Only launches when use_foxglove:=true
    foxglove_bridge = Node(
        package="foxglove_bridge",
        executable="foxglove_bridge",
        name="foxglove_bridge",
        output="screen",
        parameters=[
            {"port": 8765},
            # 0.0.0.0 makes it reachable from Windows host, not just WSL
            {"address": "0.0.0.0"},
            {"use_sim_time": False},
            # Publish /tf at full rate so the 3D panel stays smooth
            {"send_buffer_limit": 10000000},
        ],
        condition=IfCondition(use_foxglove),
    )

    # 4. RViz2 (optional, may not work in WSL2 without WSLg / VcXsrv)
    #    Loads a default config — you can replace with your own .rviz file
    rviz_config = os.path.join(
        get_package_share_directory("esibot_description"),
        "config",
        "esibot.rviz",
    )
    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        # Only load config if it exists; fall back to blank if not
        arguments=["-d", rviz_config] if os.path.exists(rviz_config) else [],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription([
        urdf_file_arg,
        use_foxglove_arg,
        use_rviz_arg,
        robot_state_pub,
        joint_state_pub_gui,
        foxglove_bridge,
        rviz2,
    ])
