"""
EsiBot Nav2 bringup (localization + navigation) on a pre-built map.

Usage:
  ros2 launch esibot_navigation nav2.launch.py

Override map or params:
  ros2 launch esibot_navigation nav2.launch.py \
      map:=/absolute/path/to/your_map.yaml \
      params_file:=/absolute/path/to/nav2_params.yaml
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    pkg_share = get_package_share_directory("esibot_navigation")
    nav2_bringup_dir = get_package_share_directory("nav2_bringup")

    default_map = os.path.join(pkg_share, "maps", "esibot_map.yaml")
    default_params = os.path.join(pkg_share, "config", "nav2_params.yaml")
    default_rviz = os.path.join(nav2_bringup_dir, "rviz", "nav2_default_view.rviz")
    headless = not (
        os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY")
    )
    default_use_rviz = "false" if headless else "true"

    # Launch arguments
    map_arg = DeclareLaunchArgument(
        "map",
        default_value=default_map,
        description="Full path to the map YAML file",
    )
    params_arg = DeclareLaunchArgument(
        "params_file",
        default_value=default_params,
        description="Full path to Nav2 parameters YAML",
    )
    scan_topic_arg = DeclareLaunchArgument(
        "scan_topic",
        default_value="scan",
        description="LaserScan topic to use for localization and costmaps",
    )
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation (Gazebo) clock if true",
    )
    autostart_arg = DeclareLaunchArgument(
        "autostart",
        default_value="true",
        description="Autostart the Nav2 stack",
    )
    use_composition_arg = DeclareLaunchArgument(
        "use_composition",
        default_value="false",
        description="Use composable node container",
    )
    use_respawn_arg = DeclareLaunchArgument(
        "use_respawn",
        default_value="false",
        description="Respawn nodes if they crash",
    )
    use_rviz_arg = DeclareLaunchArgument(
        "use_rviz",
        default_value=default_use_rviz,
        description="Launch RViz2 with Nav2 config",
    )
    rviz_config_arg = DeclareLaunchArgument(
        "rviz_config",
        default_value=default_rviz,
        description="Full path to RViz2 config",
    )

    slam_mode_arg = DeclareLaunchArgument(
        "slam_mode",
        default_value="false",
        description="true = slam_toolbox running, skip AMCL+map_server",
    )

    # Rewrite YAML to allow scan topic overrides (keeps default 'scan' standard)
    configured_params = RewrittenYaml(
        source_file=LaunchConfiguration("params_file"),
        param_rewrites={
            "amcl.ros__parameters.scan_topic": LaunchConfiguration("scan_topic"),
            "local_costmap.local_costmap.ros__parameters.voxel_layer.scan.topic": LaunchConfiguration(
                "scan_topic"
            ),
            "global_costmap.global_costmap.ros__parameters.obstacle_layer.scan.topic": LaunchConfiguration(
                "scan_topic"
            ),
            "collision_monitor.ros__parameters.scan.topic": LaunchConfiguration(
                "scan_topic"
            ),
        },
        convert_types=True,
    )

    # Nav2 bringup (localization + navigation)
    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, "launch", "bringup_launch.py")
        ),
        launch_arguments={
            "slam": "False",
            "use_localization": PythonExpression(['"False" if "', LaunchConfiguration("slam_mode"), '" == "true" else "True"']),
            "map": LaunchConfiguration("map"),
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "params_file": configured_params,
            "autostart": LaunchConfiguration("autostart"),
            "use_composition": "False",
            "use_respawn": "False",
            "log_level": "info",
        }.items(),
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", LaunchConfiguration("rviz_config")],
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_rviz")),
    )

    actions = [
        map_arg,
        params_arg,
        scan_topic_arg,
        use_sim_time_arg,
        autostart_arg,
        use_composition_arg,
        use_respawn_arg,
        use_rviz_arg,
        rviz_config_arg,
        slam_mode_arg,
    ]

    if headless:
        actions.append(
            LogInfo(
                msg=(
                    "[nav2.launch.py] No DISPLAY/WAYLAND_DISPLAY detected — "
                    "defaulting use_rviz:=false."
                )
            )
        )

    actions.extend([bringup, rviz])

    return LaunchDescription(actions)
