"""
Joy teleop — PS4 DualShock 4
==============================
Lance joy_node + teleop_twist_joy indépendamment du dashboard.

Usage :
  ros2 launch esibot_bringup joy_teleop.launch.py

Prérequis sur le Pi :
  sudo apt install ros-jazzy-joy ros-jazzy-teleop-twist-joy

Connexion manette :
  USB  : brancher simplement
  BT   : maintenir Share + PS jusqu'au clignotement rapide, puis :
         sudo bluetoothctl
         scan on → connect <MAC> → trust <MAC>

Contrôle :
  Maintenir L1          → active l'envoi cmd_vel
  Maintenir L1 + R1     → turbo
  Stick gauche Y        → avancer / reculer
  Stick droit  X        → tourner gauche / droite
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    joy_params = os.path.join(
        get_package_share_directory('esibot_bringup'),
        'config', 'joy_params.yaml'
    )

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        parameters=[{
            'device_id': 0,
            'deadzone': 0.1,
            'autorepeat_rate': 20.0,
        }]
    )

    teleop_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy_node',
        output='screen',
        parameters=[joy_params],
        remappings=[('/cmd_vel', '/cmd_vel')],
    )

    return LaunchDescription([
        joy_node,
        teleop_node,
    ])
