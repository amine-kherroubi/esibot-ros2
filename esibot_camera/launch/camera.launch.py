import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile


def generate_launch_description():
    pkg_share = get_package_share_directory("esibot_camera")
    default_params_file = os.path.join(pkg_share, "config", "camera_params.yaml")

    params_file = LaunchConfiguration("params_file")
    esp32_ip = LaunchConfiguration("esp32_ip")
    esp32_port = LaunchConfiguration("esp32_port")
    sim_mode = LaunchConfiguration("sim_mode")

    log_start = LogInfo(
        msg=[
            "\n",
            "=======================================================\n",
            "  EsiBot Camera Launch\n",
            "=======================================================\n",
            "  params_file : ",
            params_file,
            "\n",
            "  esp32_ip    : ",
            esp32_ip,
            "\n",
            "  esp32_port  : ",
            esp32_port,
            "\n",
            "  sim_mode    : ",
            sim_mode,
            "\n",
            "  topics      : /camera/image_raw, /camera/camera_info\n",
            "=======================================================\n",
        ]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=default_params_file,
                description="Path to camera parameters YAML",
            ),
            DeclareLaunchArgument(
                "esp32_ip",
                default_value="192.168.1.80",
                description="ESP32-CAM IP address",
            ),
            DeclareLaunchArgument(
                "esp32_port",
                default_value="80",
                description="ESP32-CAM HTTP port",
            ),
            DeclareLaunchArgument(
                "sim_mode",
                default_value="false",
                description="Enable simulation mode",
            ),
            log_start,
            Node(
                package="esibot_camera",
                executable="camera_stream_node",
                name="esibot_camera_node",
                parameters=[
                    ParameterFile(params_file, allow_substs=True),
                    {
                        "esp32_ip": esp32_ip,
                        "esp32_port": esp32_port,
                        "sim_mode": sim_mode,
                    },
                ],
                output="screen",
                emulate_tty=True,
            ),
            Node(
                package="image_transport",
                executable="republish",
                name="camera_compressed_republisher",
                arguments=["raw", "compressed"],
                remappings=[("in", "/camera/image_raw"), ("out", "/camera/compressed")],
                output="screen",
            ),
        ]
    )
