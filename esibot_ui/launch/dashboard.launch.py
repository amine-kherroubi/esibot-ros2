"""
esibot_ui — launch/dashboard.launch.py
=======================================
Lance le serveur web EsiBot (dashboard_node) + web_bridge (rosbridge).

Usage :
  # Dashboard seul
  ros2 launch esibot_ui dashboard.launch.py

  # Dashboard + rosbridge ensemble
  ros2 launch esibot_ui dashboard.launch.py with_bridge:=true

  # Port personnalisé
  ros2 launch esibot_ui dashboard.launch.py http_port:=8080 bridge_port:=9090

Accès :
  Dashboard : http://<robot-ip>:8080
  Bridge    : ws://<robot-ip>:9090
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # ── Arguments ────────────────────────────────────────────────────────────
    http_port_arg = DeclareLaunchArgument(
        'http_port',
        default_value='8080',
        description='Port HTTP du dashboard web'
    )
    bridge_port_arg = DeclareLaunchArgument(
        'bridge_port',
        default_value='9090',
        description='Port WebSocket rosbridge'
    )
    with_bridge_arg = DeclareLaunchArgument(
        'with_bridge',
        default_value='true',
        description='Lancer rosbridge en même temps que le dashboard'
    )

    # ── Log ──────────────────────────────────────────────────────────────────
    log = LogInfo(msg=[
        '\n',
        '=======================================================\n',
        '  EsiBot UI — Dashboard + Web Bridge\n',
        '=======================================================\n',
        '  Dashboard : http://0.0.0.0:', LaunchConfiguration('http_port'), '\n',
        '  Bridge    : ws://0.0.0.0:',  LaunchConfiguration('bridge_port'), '\n',
        '=======================================================\n',
    ])

    # ── dashboard_node ────────────────────────────────────────────────────────
    dashboard_node = Node(
        package='esibot_ui',
        executable='dashboard_node',
        name='dashboard_node',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'port': LaunchConfiguration('http_port'),
        }]
    )

    # ── web_bridge (optionnel) ────────────────────────────────────────────────
    web_bridge_pkg = get_package_share_directory('web_bridge')
    web_bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(web_bridge_pkg, 'launch', 'web_bridge.launch.py')
        ),
        launch_arguments={
            'port': LaunchConfiguration('bridge_port')
        }.items(),
        condition=IfCondition(LaunchConfiguration('with_bridge'))
    )

    map_saver = Node(
        package="esibot_ui",
        executable="map_saver_node",
        name="map_saver_node",
        output="screen",
    )

    nav_goal_proxy = Node(
        package='esibot_ui',
        executable='nav_goal_proxy',
        name='nav_goal_proxy',
        output='screen',
    )

    return LaunchDescription([
        http_port_arg,
        bridge_port_arg,
        with_bridge_arg,
        log,
        dashboard_node,
        web_bridge,
        map_saver,
        nav_goal_proxy,
    ])
