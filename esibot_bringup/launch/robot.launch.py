"""
Top-level EsiBot launcher
=========================

Composes package-level launch fragments into a single, canonical entrypoint
`ros2 launch esibot_bringup robot.launch.py`.

Public API (recommended):
  runtime:=sim|hardware       (default: sim)
  sim_backend:=mock|gazebo    (default: mock)
  mode:=base|slam|nav|vision  (default: base)
  dashboard:=true|false       (default: true)
  bridge:=true|false          (default: true)
  visualization:=none|rviz|foxglove|both (default: none)
  log_level:=debug|info|warn|error (default: info)

This launcher forwards the appropriate flags to the underlying package
launch files and ensures simulation vs hardware modes are isolated.
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
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description():
    # package paths
    bringup_pkg = get_package_share_directory("esibot_bringup")
    desc_pkg = get_package_share_directory("esibot_description")
    sensors_pkg = get_package_share_directory("esibot_sensors")
    camera_pkg = get_package_share_directory("esibot_camera")
    vision_pkg = get_package_share_directory("esibot_vision")
    slam_pkg = get_package_share_directory("esibot_slam")
    nav_pkg = get_package_share_directory("esibot_navigation")
    gazebo_pkg = get_package_share_directory("esibot_gazebo")
    ui_pkg = get_package_share_directory("esibot_ui")

    # launch arguments
    runtime_arg = DeclareLaunchArgument(
        "runtime",
        default_value="sim",
        choices=["sim", "hardware"],
        description="Overall runtime target: sim or hardware",
    )

    sim_backend_arg = DeclareLaunchArgument(
        "sim_backend",
        default_value="mock",
        choices=["mock", "gazebo"],
        description="Simulation backend when runtime:=sim",
    )

    mode_arg = DeclareLaunchArgument(
        "mode",
        default_value="base",
        choices=["base", "slam", "nav", "vision"],
        description="High-level runtime mode",
    )

    dashboard_arg = DeclareLaunchArgument(
        "dashboard",
        default_value="true",
        description="Enable the HTTP dashboard",
    )

    bridge_arg = DeclareLaunchArgument(
        "bridge",
        default_value="true",
        description="Enable the foxglove/rosbridge websocket",
    )

    visualization_arg = DeclareLaunchArgument(
        "visualization",
        default_value="none",
        choices=["none", "rviz", "foxglove", "both"],
        description="Visualization policy",
    )

    log_level_arg = DeclareLaunchArgument(
        "log_level",
        default_value="info",
        description="Global log verbosity",
    )

    runtime = LaunchConfiguration("runtime")
    sim_backend = LaunchConfiguration("sim_backend")
    mode = LaunchConfiguration("mode")
    dashboard = LaunchConfiguration("dashboard")
    bridge = LaunchConfiguration("bridge")
    visualization = LaunchConfiguration("visualization")
    log_level = LaunchConfiguration("log_level")

    # convenience conditions
    is_vision = IfCondition(PythonExpression(["'", mode, "' == 'vision'"]))
    is_not_vision = IfCondition(PythonExpression(["'", mode, "' != 'vision'"]))

    # startup banner
    log_banner = LogInfo(
        msg=[
            "\n",
            "=======================================================\n",
            "  EsiBot — Canonical Robot Launcher\n",
            "=======================================================\n",
            "  runtime      : ",
            runtime,
            "\n",
            "  sim_backend  : ",
            sim_backend,
            "\n",
            "  mode         : ",
            mode,
            "\n",
            "  visualization: ",
            visualization,
            "\n",
            "  dashboard    : ",
            dashboard,
            "\n",
            "  bridge       : ",
            bridge,
            "\n",
            "  log_level    : ",
            log_level,
            "\n",
            "=======================================================\n",
        ]
    )

    # Include bringup (driver) — forward sim_mode and use_sim_time
    bringup_include = TimerAction(
        period=2.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(bringup_pkg, "launch", "bringup.launch.py")
                ),
                launch_arguments={
                    "sim_mode": PythonExpression(["'", runtime, "' == 'sim'"]),
                    "use_sim_time": PythonExpression([
                        "'",
                        runtime,
                        "' == 'sim' and '",
                        sim_backend,
                        "' == 'gazebo'",
                    ]),
                }.items(),
            )
        ],
    )

    # Include radar for non-vision modes
    radar_include = TimerAction(
        period=3.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(sensors_pkg, "launch", "radar.launch.py")
                ),
                launch_arguments={
                    "sim_mode": PythonExpression(["'", runtime, "' == 'sim'"]),
                }.items(),
            )
        ],
        condition=is_not_vision,
    )

    # Camera + vision (vision mode only)
    camera_include = TimerAction(
        period=3.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(camera_pkg, "launch", "camera.launch.py")
                ),
                launch_arguments={
                    "sim_mode": PythonExpression(["'", runtime, "' == 'sim'"]),
                }.items(),
            )
        ],
        condition=is_vision,
    )

    vision_include = TimerAction(
        period=4.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(vision_pkg, "launch", "vision.launch.py")
                ),
                launch_arguments={"camera_image_topic": "/camera/compressed"}.items(),
            )
        ],
        condition=is_vision,
    )

    # SLAM — include the correct mode (sim vs hw)
    slam_include_sim = TimerAction(
        period=5.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(slam_pkg, "launch", "slam.launch.py")
                ),
                launch_arguments={"mode": "sim"}.items(),
            )
        ],
        condition=IfCondition(
            PythonExpression(["'", mode, "' == 'slam' and '", runtime, "' == 'sim'"])
        ),
    )

    slam_include_hw = TimerAction(
        period=5.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(slam_pkg, "launch", "slam.launch.py")
                ),
                launch_arguments={"mode": "hw"}.items(),
            )
        ],
        condition=IfCondition(
            PythonExpression(["'", mode, "' == 'slam' and '", runtime, "' == 'hardware'"])
        ),
    )

    # Nav2 — forward use_sim_time when using Gazebo
    nav_include = TimerAction(
        period=5.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(nav_pkg, "launch", "nav2.launch.py")
                ),
                launch_arguments={
                    "use_sim_time": PythonExpression([
                        "'",
                        runtime,
                        "' == 'sim' and '",
                        sim_backend,
                        "' == 'gazebo'",
                    ])
                }.items(),
            )
        ],
        condition=IfCondition(PythonExpression(["'", mode, "' == 'nav'"])),
    )

    # Gazebo sim (only when runtime==sim and sim_backend==gazebo)
    gazebo_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_pkg, "launch", "sim.launch.py")
        ),
        launch_arguments={
            "use_foxglove": PythonExpression([
                "'",
                bridge,
                "' == 'true' and '",
                visualization,
                "' in ['foxglove','both']",
            ])
        }.items(),
        condition=IfCondition(
            PythonExpression([
                "'",
                runtime,
                "' == 'sim' and '",
                sim_backend,
                "' == 'gazebo'",
            ])
        ),
    )

    # Display (RViz / Foxglove) — only if visualization requested
    display_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(desc_pkg, "launch", "display.launch.py")
        ),
        launch_arguments={
            "use_foxglove": PythonExpression([
                "'",
                bridge,
                "' == 'true' and '",
                visualization,
                "' in ['foxglove','both']",
            ]),
            "use_rviz": PythonExpression([
                "'",
                visualization,
                "' in ['rviz','both']",
            ]),
        }.items(),
        condition=IfCondition(
            PythonExpression(["'", visualization, "' != 'none'"])
        ),
    )

    # Dashboard (web UI)
    dashboard_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ui_pkg, "launch", "dashboard.launch.py")
        ),
        launch_arguments={"with_bridge": bridge}.items(),
        condition=IfCondition(dashboard),
    )

    return LaunchDescription(
        [
            # args
            runtime_arg,
            sim_backend_arg,
            mode_arg,
            dashboard_arg,
            bridge_arg,
            visualization_arg,
            log_level_arg,
            # startup banner
            log_banner,
            # includes
            bringup_include,
            radar_include,
            camera_include,
            vision_include,
            slam_include_sim,
            slam_include_hw,
            nav_include,
            gazebo_include,
            display_include,
            dashboard_include,
        ]
    )
