#!/usr/bin/env python3
"""
EsiBot Sensors — radar_node launch file
========================================
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # ── Launch arguments ──────────────────────────────────────────────────────

    servo_pin_arg = DeclareLaunchArgument(
        "servo_pin",
        default_value="17",
        description="BCM GPIO pin number for the SG90 servo signal wire",
    )

    trig_pin_arg = DeclareLaunchArgument(
        "trig_pin",
        default_value="23",
        description="BCM GPIO pin number for HC-SR04 TRIG",
    )

    echo_pin_arg = DeclareLaunchArgument(
        "echo_pin",
        default_value="25",
        description="BCM GPIO pin number for HC-SR04 ECHO",
    )

    sweep_period_arg = DeclareLaunchArgument(
        "sweep_period",
        default_value="3.0",
        description=(
            "Seconds between sweep triggers. "
            "Must be longer than the worst-case sweep duration "
            "(19 steps x 0.11 s ~ 2.1 s). Default 3.0 s gives margin."
        ),
    )

    sim_mode_arg = DeclareLaunchArgument(
        "sim_mode",
        default_value="false",
        description=(
            "Set to true to force simulation/mock mode even when "
            "RPi.GPIO is available (useful for unit testing on a PC)."
        ),
    )

    # ── radar_node ────────────────────────────────────────────────────────────

    radar_node = Node(
        package="esibot_sensors",
        executable="radar_node",
        name="radar_node",
        output="screen",
        emulate_tty=True,  # color log output in the terminal
        parameters=[
            {
                "servo_pin": LaunchConfiguration("servo_pin"),
                "trig_pin": LaunchConfiguration("trig_pin"),
                "echo_pin": LaunchConfiguration("echo_pin"),
                "sweep_period": LaunchConfiguration("sweep_period"),
                "sim_mode": LaunchConfiguration("sim_mode"),
            }
        ],
        # /scan publishes at ~0.33 Hz (one scan per sweep_period).
        # QoS is left at the node default (reliable, depth 10).
    )

    return LaunchDescription(
        [
            servo_pin_arg,
            trig_pin_arg,
            echo_pin_arg,
            sweep_period_arg,
            sim_mode_arg,
            radar_node,
        ]
    )
