from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([

        DeclareLaunchArgument(
            'esp32_ip',
            default_value='192.168.1.100',
            description='Adresse IP ESP32-CAM'
        ),

        DeclareLaunchArgument(
            'esp32_port',
            default_value='80',
            description='Port HTTP ESP32-CAM'
        ),

        Node(
            package='esibot_camera',
            executable='camera_stream_node',
            name='esibot_camera_node',
            parameters=[{
                'esp32_ip':       LaunchConfiguration('esp32_ip'),
                'esp32_port':     LaunchConfiguration('esp32_port'),
                'stream_path':    '/stream',
                'frame_width':    320,
                'frame_height':   240,
                'publish_rate':   10.0,
                'show_fps':       True,
                'camera_frame':   'camera_optical_frame',
                'reconnect_delay': 3.0,
            }],
            output='screen',
            emulate_tty=True,
        )
    ])
