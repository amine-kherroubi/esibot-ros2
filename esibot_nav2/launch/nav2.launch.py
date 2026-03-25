"""
EsiBot Nav2 bringup (localization + navigation) on a pre-built map.

Usage:
  ros2 launch esibot_nav2 nav2.launch.py

Override map or params:
  ros2 launch esibot_nav2 nav2.launch.py \
      map:=/absolute/path/to/your_map.yaml \
      params_file:=/absolute/path/to/nav2_params.yaml
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("esibot_nav2")
    nav2_bringup_dir = get_package_share_directory("nav2_bringup")

    default_map = os.path.join(pkg_share, "maps", "esibot_map.yaml")
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
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="True",
        description="Use simulation (Gazebo) clock if true",
    )
    autostart_arg = DeclareLaunchArgument(
        "autostart",
        default_value="True",
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
        default_value="True",
        description="Launch RViz2 with Nav2 config",
    )
    rviz_config_arg = DeclareLaunchArgument(
        "rviz_config",
        default_value=default_rviz,
        description="Full path to RViz2 config",
    )

    # Nav2 bringup (localization + navigation)
    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, "launch", "bringup_launch.py")
        ),
        launch_arguments={
            "slam": "False",
            "use_localization": "True",
            "map": LaunchConfiguration("map"),
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "params_file": LaunchConfiguration("params_file"),
            "autostart": LaunchConfiguration("autostart"),
            "use_composition": LaunchConfiguration("use_composition"),
            "use_respawn": LaunchConfiguration("use_respawn"),
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

    return LaunchDescription(
        [
            map_arg,
            params_arg,
            use_sim_time_arg,
            autostart_arg,
            use_composition_arg,
            use_respawn_arg,
            use_rviz_arg,
            rviz_config_arg,
            bringup,
            rviz,
        ]
    )
