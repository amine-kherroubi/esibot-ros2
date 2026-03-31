# Copyright 2026 EsiBot Project
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
r"""
Save the current SLAM map to disk.

This launch file calls nav2_map_server's map_saver_cli, which reads the /map
topic and writes two files that together represent the 2D occupancy map built
by slam_toolbox:

  <map_dir>/<map_name>.pgm   — PGM grayscale image of the map
                               White pixels  = free space (traversable)
                               Black pixels  = obstacles / walls
                               Grey pixels   = unknown (not yet explored)

  <map_dir>/<map_name>.yaml  — Metadata file read by Nav2 (Task 3.6)
                               Contains: resolution, origin, free/occupied thresholds

These two files are Task 3.5 deliverable #4:
  Saved SLAM map - .pgm + .yaml of a minimum 3 m x 3 m area

The saved map is also required by Task 3.6 (Nav2 navigation):
  ros2 launch esibot_bringup navigation.launch.py map:=<path_to_yaml>

Launch arguments:
  map_name   string   Base name for the output files (default: esibot_map)
  map_dir    path     Destination directory (default: share/esibot_slam/maps/)

Usage:
  # Save with default name:
  ros2 launch esibot_slam save_map.launch.py

  # Save with a custom name:
  ros2 launch esibot_slam save_map.launch.py map_name:=lab_room_final

  # Save to a custom directory:
  ros2 launch esibot_slam save_map.launch.py \
      map_name:=final_map \
      map_dir:=/home/ubuntu/esibot_maps

Prerequisites:
  - slam_toolbox must be running and actively publishing on /map
  - The robot must have explored a sufficient area
  Verify: ros2 topic hz /map   -> should show a non-zero rate

After saving, verify the files:
  ls -lh ~/robot_ws/src/esibot_slam/maps/
  # or wherever map_dir points
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():

    slam_pkg = get_package_share_directory('esibot_slam')

    # ── Launch arguments ──────────────────────────────────────────────────────
    map_name_arg = DeclareLaunchArgument(
        'map_name',
        default_value='esibot_map',
        description=(
            'Base name for the saved map files. '
            'Produces <map_name>.pgm (image) and <map_name>.yaml (metadata).'
        ),
    )

    map_dir_arg = DeclareLaunchArgument(
        'map_dir',
        default_value=os.path.join(slam_pkg, 'maps'),
        description=(
            'Destination directory for the map files. '
            'Default: share/esibot_slam/maps/ inside the ROS 2 workspace.'
        ),
    )

    map_name = LaunchConfiguration('map_name')
    map_dir = LaunchConfiguration('map_dir')

    # ── map_saver_cli ─────────────────────────────────────────────────────────
    #
    # nav2_map_server provides map_saver_cli.
    # It subscribes to /map (nav_msgs/OccupancyGrid) and saves it to disk.
    #
    # Arguments:
    #   -f <path>              : output file path without extension
    #   --ros-args             : begin ROS 2 parameter arguments
    #   -p save_map_timeout    : max milliseconds to wait for /map (5000 ms)
    #   -p free_thresh_default : cells below this probability are free (0.25)
    #   -p occupied_thresh_default : cells above this are obstacles (0.65)
    #
    # The probability thresholds match the slam_toolbox default values.
    # Changing them will affect how Nav2 interprets the map during navigation.
    save_map = ExecuteProcess(
        cmd=[
            'ros2',
            'run',
            'nav2_map_server',
            'map_saver_cli',
            '-f',
            PathJoinSubstitution([map_dir, map_name]),
            '--ros-args',
            '-p',
            'save_map_timeout:=5000.0',
            '-p',
            'free_thresh_default:=0.25',
            '-p',
            'occupied_thresh_default:=0.65',
        ],
        output='screen',
    )

    log_info = LogInfo(
        msg=[
            '\n',
            '=======================================================\n',
            '  EsiBot SLAM — Saving Map to Disk\n',
            '=======================================================\n',
            '  map_saver_cli will write:\n',
            '    • <map_dir>/<map_name>.pgm   (grayscale map image)\n',
            '    • <map_dir>/<map_name>.yaml  (metadata for Nav2)\n',
            '\n',
            '  These files are Task 3.5 Deliverable #4.\n',
            '  Nav2 (Task 3.6) will use the .yaml file to load the map.\n',
            '=======================================================\n',
        ]
    )

    return LaunchDescription(
        [
            map_name_arg,
            map_dir_arg,
            log_info,
            save_map,
        ]
    )
