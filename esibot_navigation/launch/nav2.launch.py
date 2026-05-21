"""
EsiBot Nav2 launch file: localization (AMCL) + navigation on a pre-built map.

Localization (map_server + AMCL) starts immediately.
Navigation stack starts after a 15-second delay so that AMCL has time to
activate and publish the map→odom TF before global_costmap tries to use it.

Usage:
  ros2 launch esibot_navigation nav2.launch.py

Override map or parameters:
  ros2 launch esibot_navigation nav2.launch.py \
      map:=/absolute/path/to/your_map.yaml \
      params_file:=/absolute/path/to/nav2_params.yaml
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    pkg_share = get_package_share_directory("esibot_navigation")
    slam_pkg_share = get_package_share_directory("esibot_slam")
    nav2_bringup_dir = get_package_share_directory("nav2_bringup")

    default_map = os.path.join(slam_pkg_share, "maps", "esibot_map.yaml")
    default_params = os.path.join(pkg_share, "config", "nav2_params.yaml")
    default_rviz = os.path.join(nav2_bringup_dir, "rviz", "nav2_default_view.rviz")

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
        default_value="/scan",
        description="LaserScan topic for localization and costmaps",
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
        default_value="False",
        description="Use composable node container",
    )
    use_respawn_arg = DeclareLaunchArgument(
        "use_respawn",
        default_value="False",
        description="Respawn nodes if they crash",
    )
    use_rviz_arg = DeclareLaunchArgument(
        "use_rviz",
        default_value="false",
        description="Launch RViz2 with Nav2 config",
    )
    rviz_config_arg = DeclareLaunchArgument(
        "rviz_config",
        default_value=default_rviz,
        description="Full path to RViz2 config",
    )

    # Rewrite scan_topic only in AMCL — do NOT rewrite "topic" globally because
    # that clobbers cmd_vel_in_topic / cmd_vel_out_topic / state_topic etc.
    configured_params = RewrittenYaml(
        source_file=LaunchConfiguration("params_file"),
        param_rewrites={
            "scan_topic": LaunchConfiguration("scan_topic"),
        },
        convert_types=True,
    )

    common_args = {
        "use_sim_time": LaunchConfiguration("use_sim_time"),
        "params_file": configured_params,
        "autostart": LaunchConfiguration("autostart"),
        "use_composition": LaunchConfiguration("use_composition"),
        "use_respawn": LaunchConfiguration("use_respawn"),
        "log_level": "info",
    }

    # Step 1 — localization: map_server + AMCL start immediately.
    # AMCL has set_initial_pose:true so it publishes map→odom TF right after activation.
    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, "launch", "localization_launch.py")
        ),
        launch_arguments={
            **common_args,
            "map": LaunchConfiguration("map"),
        }.items(),
    )

    # Step 2 — navigation stack: starts 15 s later, after AMCL has activated
    # and published map→odom, so global_costmap can find the transform.
    navigation = TimerAction(
        period=15.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(nav2_bringup_dir, "launch", "navigation_launch.py")
                ),
                launch_arguments=common_args.items(),
            )
        ],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", LaunchConfiguration("rviz_config")],
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_rviz")),
    )

    return LaunchDescription([
        map_arg,
        params_arg,
        scan_topic_arg,
        use_sim_time_arg,
        autostart_arg,
        use_composition_arg,
        use_respawn_arg,
        use_rviz_arg,
        rviz_config_arg,
        localization,
        navigation,
        rviz,
    ])
