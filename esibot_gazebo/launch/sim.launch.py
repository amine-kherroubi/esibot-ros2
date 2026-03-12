import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    desc_pkg   = get_package_share_directory("esibot_description")
    gazebo_pkg = get_package_share_directory("esibot_gazebo")

    urdf_file  = os.path.join(desc_pkg,   "urdf",   "esibot.urdf.xacro")
    world_file = os.path.join(gazebo_pkg, "worlds", "esibot_world.sdf")

    use_foxglove_arg = DeclareLaunchArgument(
        "use_foxglove", default_value="true",
        description="Launch Foxglove Bridge (ws://localhost:8765)",
    )
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", default_value="true",
    )

    use_foxglove = LaunchConfiguration("use_foxglove")
    use_sim_time = LaunchConfiguration("use_sim_time")

    robot_description = ParameterValue(
        Command([FindExecutable(name="xacro"), " ", urdf_file]),
        value_type=str
    )

    robot_state_pub = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            {"robot_description": robot_description},
            {"use_sim_time": use_sim_time},
            {"publish_frequency": 50.0},
        ],
    )

    # Gazebo Harmonic (Jazzy): gz sim, NOT gazebo
    gz_sim = ExecuteProcess(
        cmd=["gz", "sim", "-r", world_file],
        output="screen",
    )

    # Spawn via ros_gz_sim, NOT gazebo_ros
    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        name="spawn_esibot",
        output="screen",
        arguments=[
            "-topic", "robot_description",
            "-name",  "esibot",
            "-x", "0.0", "-y", "0.0", "-z", "0.05",
        ],
    )

    delayed_spawn = TimerAction(period=4.0, actions=[spawn_robot])

    # Bridge: gz topics → ROS 2 topics
    gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="gz_bridge",
        output="screen",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
            "/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry",
            "/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
            "/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model",
            "/ultrasound_raw@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
            "/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image",
            "/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
        ],
    )

    foxglove_bridge = Node(
        package="foxglove_bridge",
        executable="foxglove_bridge",
        output="screen",
        parameters=[
            {"port": 8765},
            {"address": "0.0.0.0"},
            {"use_sim_time": use_sim_time},
        ],
        condition=IfCondition(use_foxglove),
    )

    return LaunchDescription([
        use_foxglove_arg,
        use_sim_time_arg,
        robot_state_pub,
        gz_sim,
        delayed_spawn,
        gz_bridge,
        foxglove_bridge,
    ])