import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    pkg = get_package_share_directory("esibot_vision")

    vision_params = os.path.join(pkg, "config", "vision_params.yaml")

    sign_model_default = os.path.join(pkg, "models", "signs_best.pt")
    obstacle_model_default = os.path.join(pkg, "models", "yolov8n.pt")

    sign_model_exists = os.path.isfile(sign_model_default)
    obstacle_model_exists = os.path.isfile(obstacle_model_default)

    # If models are missing, default to empty path (detectors disable themselves).
    sign_model_default = sign_model_default if sign_model_exists else ""
    obstacle_model_default = obstacle_model_default if obstacle_model_exists else ""

    sign_model_arg = DeclareLaunchArgument(
        "sign_model_path",
        default_value=sign_model_default,
        description="Absolute path to signs_best.pt (empty disables sign detection).",
    )
    obstacle_model_arg = DeclareLaunchArgument(
        "obstacle_model_path",
        default_value=obstacle_model_default,
        description="Absolute path to yolov8n.pt (empty disables obstacle detection).",
    )

    missing_logs = []
    if not sign_model_exists:
        missing_logs.append(
            LogInfo(
                msg=(
                    "[vision.launch.py] signs_best.pt not found — sign detection "
                    "disabled. Provide the model via sign_model_path:=/abs/path."
                )
            )
        )
    if not obstacle_model_exists:
        missing_logs.append(
            LogInfo(
                msg=(
                    "[vision.launch.py] yolov8n.pt not found — obstacle detection "
                    "disabled. Provide the model via obstacle_model_path:=/abs/path."
                )
            )
        )

    return LaunchDescription(
        [
            sign_model_arg,
            obstacle_model_arg,
            *missing_logs,
            Node(
                package="esibot_vision",
                executable="vision_node",
                name="vision_node",
                parameters=[
                    vision_params,
                    {
                        "sign_model_path": LaunchConfiguration("sign_model_path"),
                        "obstacle_model_path": LaunchConfiguration(
                            "obstacle_model_path"
                        ),
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
