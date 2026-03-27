"""
EsiBot Display Launch File
===========================
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


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
    # Both robot_state_publisher and joint_state_publisher receive this same
    # object — it is evaluated once and shared, avoiding the QoS mismatch
    # that occurs when joint_state_publisher tries to read /robot_description
    # from the topic.
    robot_description = ParameterValue(
        Command([FindExecutable(name="xacro"), " ", urdf_file]),
        value_type=str,
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
            {"publish_frequency": 50.0},
        ],
    )

    # 2. Foxglove bridge
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
            {"address": "0.0.0.0"},
            {"use_sim_time": False},
            {"send_buffer_limit": 10000000},
        ],
        condition=IfCondition(use_foxglove),
    )

    # 3. RViz2
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
        arguments=["-d", rviz_config] if os.path.exists(rviz_config) else [],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription([
        urdf_file_arg,
        use_foxglove_arg,
        use_rviz_arg,
        robot_state_pub,
        foxglove_bridge,
        rviz2,
    ])