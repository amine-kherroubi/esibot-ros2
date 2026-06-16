import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.descriptions import ParameterFile


def generate_launch_description():
    pkg_share = get_package_share_directory("esibot_camera")
    params_file = os.path.join(pkg_share, "config", "camera_params.yaml")


def generate_launch_description():
    log_start = LogInfo(
        msg=[
            "\n",
            "=======================================================\n",
            "  EsiBot Camera Launch\n",
            "=======================================================\n",
            "  params_file : ", LaunchConfiguration("params_file"),
            "\n",
            "  esp32_ip    : ", LaunchConfiguration("esp32_ip"),
            "\n",
            "  sim_mode    : ", LaunchConfiguration("sim_mode"),
            "\n",
            "=======================================================\n",
        ]
    )

    return LaunchDescription([
        log_start,
        DeclareLaunchArgument(
            "params_file",
            default_value=params_file,
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
        # ── Camera node ───────────────────────────────────────────────
        Node(
            package="esibot_camera",
            executable="camera_stream_node",
            name="esibot_camera_node",
            parameters=[
                ParameterFile(params_file, allow_substs=True),
                {
                    "esp32_ip": LaunchConfiguration("esp32_ip"),
                    "esp32_port": LaunchConfiguration("esp32_port"),
                    "sim_mode": LaunchConfiguration("sim_mode"),
                },
            ],
            output="screen",
            emulate_tty=True,
        ),
        # ── Compressed republish → /camera/compressed ─────────────────
        Node(
            package="image_transport",
            executable="republish",
            name="camera_compressed_republisher",
            arguments=["raw", "compressed"],
            remappings=[("in", "/camera/image_raw"), ("out", "/camera/compressed")],
            output="screen",
        ),
    ])
