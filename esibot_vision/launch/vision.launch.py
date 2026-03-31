import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    pkg = get_package_share_directory("esibot_vision")

    vision_params = os.path.join(pkg, "config", "vision_params.yaml")

    sign_model_path = os.path.join(pkg, "models", "signs_best.pt")
    obstacle_model_path = os.path.join(pkg, "models", "yolov8n.pt")

    return LaunchDescription(
        [
            Node(
                package="esibot_vision",
                executable="vision_node",
                name="vision_node",
                parameters=[
                    vision_params,
                    {
                        "sign_model_path": sign_model_path,
                        "obstacle_model_path": obstacle_model_path,
                    },
                ],
                remappings=[
                    ("/image_raw", "/camera/image_raw"),
                ],
                output="screen",
                emulate_tty=True,
            ),
        ]
    )
