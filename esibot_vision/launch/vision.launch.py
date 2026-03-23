import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('esibot_vision')
    vision_params = os.path.join(pkg, 'config', 'vision_params.yaml')

    return LaunchDescription([

        Node(
            package='esibot_vision',
            executable='vision_node',
            name='vision_node',
            parameters=[vision_params],
            output='screen',
            emulate_tty=True,
        ),


    ])
