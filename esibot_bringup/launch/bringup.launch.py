"""Bring up the EsiBot driver (and optional teleop)."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = PathJoinSubstitution(
        [FindPackageShare("esibot_bringup"), "config", "driver_params.yaml"]
    )

    declare_params_file = DeclareLaunchArgument(
        "params_file",
        default_value=params_file,
        description="Full path to the driver parameters file",
    )

    declare_serial_port = DeclareLaunchArgument(
        "serial_port",
        default_value="/dev/ttyUSB0",
        description="Serial port connected to the ESP32 (e.g. /dev/ttyUSB0)",
    )

    declare_baud_rate = DeclareLaunchArgument(
        "baud_rate",
        default_value="115200",
        description="UART baud rate matching ESP32 firmware",
    )

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use /clock if true (simulation)",
    )

    declare_cmd_vel_topic = DeclareLaunchArgument(
        "cmd_vel_topic",
        default_value="cmd_vel",
        description="Velocity command topic (used by driver and teleop)",
    )

    declare_use_teleop = DeclareLaunchArgument(
        "use_teleop",
        default_value="false",
        description="Launch teleop_twist_keyboard in a new terminal",
    )

    declare_teleop_prefix = DeclareLaunchArgument(
        "teleop_prefix",
        default_value="xterm -e",
        description="Prefix to open teleop in a separate terminal",
    )

    driver_node = Node(
        package="esibot_bringup",
        executable="esibot_driver",
        name="esibot_driver",
        output="screen",
        parameters=[
            LaunchConfiguration("params_file"),
            {
                "serial_port": LaunchConfiguration("serial_port"),
                "baud_rate": LaunchConfiguration("baud_rate"),
                "cmd_vel_topic": LaunchConfiguration("cmd_vel_topic"),
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            },
        ],
    )

    teleop_node = Node(
        package="teleop_twist_keyboard",
        executable="teleop_twist_keyboard",
        name="teleop_twist_keyboard",
        output="screen",
        prefix=LaunchConfiguration("teleop_prefix"),
        remappings=[("cmd_vel", LaunchConfiguration("cmd_vel_topic"))],
        condition=IfCondition(LaunchConfiguration("use_teleop")),
    )

    return LaunchDescription(
        [
            declare_params_file,
            declare_serial_port,
            declare_baud_rate,
            declare_use_sim_time,
            declare_cmd_vel_topic,
            declare_use_teleop,
            declare_teleop_prefix,
            driver_node,
            teleop_node,
        ]
    )
