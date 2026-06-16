import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    pkg = get_package_share_directory("esibot_vision")

    vision_params = os.path.join(pkg, "config", "vision_params.yaml")

    # Single-model setup: obstacle detection (yolov8n) only. Sign detection is
    # disabled by default for performance — pass sign_model_path:=/abs/path to
    # re-enable it.
    sign_model_default = ""
    obstacle_model_default = os.path.join(pkg, "models", "yolov8n.pt")

    obstacle_model_exists = os.path.isfile(obstacle_model_default)
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

    # ESP32-CAM source — vision pulls the MJPEG stream directly (no camera node)
    esp32_ip_arg = DeclareLaunchArgument(
        "esp32_ip",
        default_value="192.168.1.80",
        description="ESP32-CAM IP address (MJPEG source).",
    )
    esp32_port_arg = DeclareLaunchArgument(
        "esp32_port",
        default_value="80",
        description="ESP32-CAM HTTP port.",
    )
    stream_path_arg = DeclareLaunchArgument(
        "stream_path",
        default_value="/stream",
        description="ESP32-CAM MJPEG stream path.",
    )
    camera_image_topic_arg = DeclareLaunchArgument(
        "camera_image_topic",
        default_value="",
        description="If set, vision_node subscribes to this CompressedImage topic instead of the ESP32 HTTP stream.",
    )
    lane_detection_arg = DeclareLaunchArgument(
        "lane_detection",
        default_value="true",
        description="Enable lane detection overlay and topics (true/false).",
    )

    missing_logs = []
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
            esp32_ip_arg,
            esp32_port_arg,
            stream_path_arg,
            camera_image_topic_arg,
            lane_detection_arg,
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
                        "esp32_ip": LaunchConfiguration("esp32_ip"),
                        "esp32_port": LaunchConfiguration("esp32_port"),
                        "stream_path": LaunchConfiguration("stream_path"),
                        "camera_image_topic": LaunchConfiguration("camera_image_topic"),
                        "lane_detection": LaunchConfiguration("lane_detection"),
                    },
                ],
                output="screen",
                emulate_tty=True,
            ),
        ]
    )
