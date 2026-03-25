import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory("esibot_vision")
    vision_params = os.path.join(pkg, "config", "vision_params.yaml")

    return LaunchDescription([

        DeclareLaunchArgument(
            "sign_model_path",
            default_value="",
            description="Chemin absolu vers signs_best.pt (laisser vide si non entraîné)",
        ),
        DeclareLaunchArgument(
            "obstacle_model_path",
            default_value="",
            description="Chemin absolu vers yolov8n.pt",
        ),

        Node(
            package    = "esibot_vision",
            executable = "vision_node",
            name       = "vision_node",
            parameters = [
                vision_params,
                {
                    "sign_model_path":     LaunchConfiguration("sign_model_path"),
                    "obstacle_model_path": LaunchConfiguration("obstacle_model_path"),
                },
            ],
            remappings = [
                ("/image_raw", "/camera/image_raw"),
            ],
            output     = "screen",
            emulate_tty= True,
        ),

    ])
