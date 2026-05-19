"""
web_bridge — launch/web_bridge.launch.py
=========================================
Lance rosbridge_suite comme passerelle WebSocket entre ROS2 et le dashboard web.

Usage :
  ros2 launch web_bridge web_bridge.launch.py
  ros2 launch web_bridge web_bridge.launch.py port:=9090

Le dashboard se connecte sur : ws://<robot-ip>:9090
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share   = get_package_share_directory('web_bridge')
    params_file = os.path.join(pkg_share, 'config', 'rosbridge_params.yaml')

    port_arg = DeclareLaunchArgument(
        'port',
        default_value='9090',
        description='Port WebSocket du bridge (default: 9090)'
    )

    log = LogInfo(msg=[
        '\n',
        '=======================================================\n',
        '  EsiBot Web Bridge — rosbridge_suite\n',
        '=======================================================\n',
        '  WebSocket : ws://0.0.0.0:', LaunchConfiguration('port'), '\n',
        '  Dashboard : connecter sur ws://<robot-ip>:', LaunchConfiguration('port'), '\n',
        '=======================================================\n',
    ])

    rosbridge_node = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name='rosbridge_websocket',
        output='screen',
        parameters=[
            params_file,
            {'port': LaunchConfiguration('port')}
        ]
    )

    rosapi_node = Node(
        package='rosapi',
        executable='rosapi_node',
        name='rosapi',
        output='screen'
    )

    return LaunchDescription([
        port_arg,
        log,
        rosbridge_node,
        rosapi_node,
    ])
