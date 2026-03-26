"""
bringup.launch.py
=================
Launches the full esibot_bringup stack:
  - esibot_driver node  (odometry + motor control)
  - teleop_twist_keyboard  (keyboard control)

Usage:
    # Default (simulation, no serial)
    ros2 launch esibot_bringup bringup.launch.py

    # With real hardware
    ros2 launch esibot_bringup bringup.launch.py serial_port:=/dev/ttyUSB0
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # ── Launch arguments (can be overridden from command line) ────────────────
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyUSB0',
        description='Serial port connected to ESP32 (e.g. /dev/ttyUSB0)'
    )

    baud_rate_arg = DeclareLaunchArgument(
        'baud_rate',
        default_value='115200',
        description='UART baud rate matching ESP32 firmware setting'
    )

    # Path to the YAML config file inside this package
    config_file = PathJoinSubstitution([
        FindPackageShare('esibot_bringup'),
        'config',
        'driver_params.yaml'
    ])

    # ── esibot_driver node ────────────────────────────────────────────────────
    driver_node = Node(
        package='esibot_bringup',
        executable='esibot_driver',
        name='esibot_driver',
        output='screen',
        parameters=[
            config_file,
            {
                # Command-line arguments override the YAML file
                'serial_port': LaunchConfiguration('serial_port'),
                'baud_rate':   LaunchConfiguration('baud_rate'),
            }
        ]
    )

    # ── teleop_twist_keyboard ─────────────────────────────────────────────────
    # This lets you drive the robot with W/A/S/D keys in the terminal.
    # It publishes on /cmd_vel, which esibot_driver reads.
    teleop_node = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_twist_keyboard',
        output='screen',
        prefix='xterm -e',      # opens in a separate terminal window
        remappings=[('/cmd_vel', '/cmd_vel')]
    )

    return LaunchDescription([
        serial_port_arg,
        baud_rate_arg,
        driver_node,
        teleop_node,
    ])
