"""Bring up the EsiBot driver (and optional teleop).

=== SIM MODE — same pattern as radar_node ===

  sim_mode:=true  → driver runs without serial hardware.
                    _serial_conn stays None → _simulate_motion() is used.
                    /odom and TF are published from cmd_vel integration.
                    No ESP32 required.

  sim_mode:=false (default) → driver connects to ESP32 via UART.
                    Serial port and baud rate are used.

Usage:
  # Real hardware:
  ros2 launch esibot_bringup bringup.launch.py

  # Simulation (no ESP32):
  ros2 launch esibot_bringup bringup.launch.py sim_mode:=true

  # Simulation with teleop:
  ros2 launch esibot_bringup bringup.launch.py sim_mode:=true use_teleop:=true

  # Custom serial port:
  ros2 launch esibot_bringup bringup.launch.py serial_port:=/dev/ttyUSB1

  # Gazebo integration (sim time + sim mode):
  ros2 launch esibot_bringup bringup.launch.py sim_mode:=true use_sim_time:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = PathJoinSubstitution(
        [FindPackageShare("esibot_bringup"), "config", "driver_params.yaml"]
    )

    # ── Launch arguments ──────────────────────────────────────────────────────

    declare_params_file = DeclareLaunchArgument(
        "params_file",
        default_value=params_file,
        description="Full path to the driver parameters file",
    )

    declare_serial_port = DeclareLaunchArgument(
        "serial_port",
        default_value="/dev/ttyS0",
        description="Serial port connected to the ESP32 (e.g. /dev/ttyS0)",
    )

    declare_baud_rate = DeclareLaunchArgument(
        "baud_rate",
        default_value="115200",
        description="UART baud rate matching ESP32 firmware",
    )

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use /clock if true (Gazebo simulation)",
    )

    declare_cmd_vel_topic = DeclareLaunchArgument(
        "cmd_vel_topic",
        default_value="cmd_vel",
        description="Velocity command topic (used by driver and teleop)",
    )

    # ── sim_mode — mirrors radar_node pattern exactly ─────────────────────────
    # When true: driver skips serial connection, uses _simulate_motion().
    # /odom and TF are still published — pipeline works without ESP32.
    declare_sim_mode = DeclareLaunchArgument(
        "sim_mode",
        default_value="false",
        description=(
            "Simulation mode — same pattern as radar_node. "
            "true  = no serial hardware needed, odometry integrated from cmd_vel. "
            "false = connects to ESP32 via UART (default)."
        ),
    )

    declare_use_teleop = DeclareLaunchArgument(
        "use_teleop",
        default_value="false",
        description="Launch teleop_twist_keyboard in a new terminal",
    )

    # ── Startup log — mirrors radar_node / slam_toolbox log pattern ───────────
    log_hw = LogInfo(
        msg=[
            "\n",
            "=======================================================\n",
            "  EsiBot Bringup — HARDWARE MODE\n",
            "=======================================================\n",
            "  esibot_driver connecting to ESP32 via UART.\n",
            "  Serial port : ",
            LaunchConfiguration("serial_port"),
            "\n",
            "  Baud rate   : ",
            LaunchConfiguration("baud_rate"),
            "\n",
            "  Topics      : /odom, /tf, /battery_state\n",
            "=======================================================\n",
        ],
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration("sim_mode"), "' == 'false'"])
        ),
    )

    log_sim = LogInfo(
        msg=[
            "\n",
            "=======================================================\n",
            "  EsiBot Bringup — SIMULATION MODE\n",
            "=======================================================\n",
            "  No hardware detected. Odometry integrated from cmd_vel.\n",
            "  Topics      : /odom, /tf, /battery_state\n",
            "  Serial port : not used\n",
            "=======================================================\n",
        ],
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration("sim_mode"), "' == 'true'"])
        ),
    )

    # ── Driver node ───────────────────────────────────────────────────────────
    # sim_mode is forwarded as a ROS parameter to the node.
    # When sim_mode=true the driver's _connect_serial() is bypassed —
    # _serial_conn stays None → _simulate_motion() path is taken in _update().
    # This is identical in spirit to radar_node's HARDWARE_AVAILABLE gate.
    driver_node = Node(
        package="esibot_bringup",
        executable="esibot_driver",
        name="esibot_driver",
        output="screen",
        emulate_tty=True,
        parameters=[
            LaunchConfiguration("params_file"),
            {
                "serial_port": LaunchConfiguration("serial_port"),
                "baud_rate": LaunchConfiguration("baud_rate"),
                "cmd_vel_topic": LaunchConfiguration("cmd_vel_topic"),
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "sim_mode": LaunchConfiguration("sim_mode"),
            },
        ],
    )

    # ── Teleop — optional ─────────────────────────────────────────────────────
    teleop_node = Node(
        package="teleop_twist_keyboard",
        executable="teleop_twist_keyboard",
        name="teleop_twist_keyboard",
        output="screen",
        remappings=[("cmd_vel", LaunchConfiguration("cmd_vel_topic"))],
        condition=IfCondition(LaunchConfiguration("use_teleop")),
    )

    return LaunchDescription(
        [
            declare_params_file,
            declare_serial_port,
            declare_baud_rate,
            declare_use_sim_time,
            declare_sim_mode,
            declare_cmd_vel_topic,
            declare_use_teleop,
            log_hw,
            log_sim,
            driver_node,
            teleop_node,
        ]
    )
