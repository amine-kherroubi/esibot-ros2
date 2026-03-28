"""
esibot_slam — launch/slam.launch.py
=====================================
=== USAGE ===

  # Simulation (Gazebo must already be running):
  ros2 launch esibot_slam slam.launch.py

  # Simulation with RViz2 + teleoperation:
  ros2 launch esibot_slam slam.launch.py use_rviz:=true teleop:=true

  # All-in-one (Gazebo + SLAM in one command):
  ros2 launch esibot_slam slam_sim.launch.py

  # Real hardware (esibot_bringup + esibot_sensors must already be running):
  ros2 launch esibot_slam slam.launch.py mode:=hw use_rviz:=true teleop:=true
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    LogInfo,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.events import matches_action
from launch.substitutions import (
    AndSubstitution,
    LaunchConfiguration,
    NotSubstitution,
    PythonExpression,
)
from launch_ros.actions import LifecycleNode
from launch_ros.descriptions import ParameterFile
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch_ros.actions import Node
from lifecycle_msgs.msg import Transition


def generate_launch_description():

    # ── Package directories ───────────────────────────────────────────────────
    slam_pkg = get_package_share_directory("esibot_slam")

    slam_params_sim = os.path.join(slam_pkg, "config", "slam_params_sim.yaml")
    slam_params_hw  = os.path.join(slam_pkg, "config", "slam_params_hw.yaml")
    rviz_config     = os.path.join(slam_pkg, "config", "esibot_slam.rviz")

    # ── Launch arguments ──────────────────────────────────────────────────────
    mode_arg = DeclareLaunchArgument(
        "mode",
        default_value="sim",
        choices=["sim", "hw"],
        description=(
            "'sim' = Gazebo Harmonic simulation (use_sim_time=true, "
            "loads slam_params_sim.yaml). "
            "'hw'  = Real Raspberry Pi 4 hardware (use_sim_time=false, "
            "loads slam_params_hw.yaml). "
            "IMPORTANT: passing the wrong mode on real hardware will cause "
            "slam_toolbox to stall waiting for a /clock topic."
        ),
    )

    autostart_arg = DeclareLaunchArgument(
        "autostart",
        default_value="true",
        description=(
            "Automatically configure and activate slam_toolbox at launch. "
            "Set to false if you want to transition the node manually."
        ),
    )

    use_lifecycle_manager_arg = DeclareLaunchArgument(
        "use_lifecycle_manager",
        default_value="false",
        description=(
            "Set to true only when an external lifecycle manager (e.g. Nav2) "
            "will manage slam_toolbox transitions and hold the bond connection. "
            "Leave false for standalone SLAM use."
        ),
    )

    use_rviz_arg = DeclareLaunchArgument(
        "use_rviz",
        default_value="false",
        description="Launch RViz2 with the EsiBot SLAM visualization config.",
    )

    teleop_arg = DeclareLaunchArgument(
        "teleop",
        default_value="false",
        description=(
            "Launch teleop_twist_keyboard in a separate xterm window. "
            "Requires: sudo apt install xterm"
        ),
    )

    mode                  = LaunchConfiguration("mode")
    autostart             = LaunchConfiguration("autostart")
    use_lifecycle_manager = LaunchConfiguration("use_lifecycle_manager")
    use_rviz              = LaunchConfiguration("use_rviz")
    teleop                = LaunchConfiguration("teleop")

    # PythonExpression conditions — compatible with all ROS 2 Galactic+
    is_sim = IfCondition(PythonExpression(["'", mode, "' == 'sim'"]))
    is_hw  = IfCondition(PythonExpression(["'", mode, "' == 'hw'"]))

    # Condition: autostart=true AND use_lifecycle_manager=false
    # When use_lifecycle_manager=true the external manager drives transitions.
    do_autostart = IfCondition(
        AndSubstitution(autostart, NotSubstitution(use_lifecycle_manager))
    )

    # ── [1] relay_node — SIMULATION ONLY ────────────────────────────────────
    #
    # gz_bridge maps the Gazebo gpu_lidar to /ultrasound_raw (LaserScan).
    # slam_toolbox expects /scan. This relay bridges the gap.
    relay_node = Node(
        package="topic_tools",
        executable="relay",
        name="ultrasound_to_scan_relay",
        output="screen",
        arguments=["/ultrasound_raw", "/scan"],
        condition=is_sim,
    )

    # ── [2] static_tf_laser_link — HARDWARE ONLY ────────────────────────────

    static_tf_laser_link = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_tf_laser_link",
        output="screen",
        # Zero transform: laser_link has the same pose as itself.
        # args: x y z yaw pitch roll parent_frame child_frame
        arguments=["0", "0", "0", "0", "0", "0", "base_footprint", "laser_link"],
        condition=is_hw,
    )

    # ── [3a] async_slam_toolbox_node — SIMULATION MODE ───────────────────────
    #
    # Executable: async_slam_toolbox_node — this is the online_async solver.
    # Do NOT use slam_toolbox_node (synchronous) — it blocks the executor.
    # The YAML mode: mapping parameter is consistent with async operation.
    slam_toolbox_sim = LifecycleNode(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        namespace="",
        parameters=[
            ParameterFile(slam_params_sim, allow_substs=True),
            {
                "use_lifecycle_manager": use_lifecycle_manager,
                "use_sim_time": True,
            },
        ],
        remappings=[
            ("/scan", "/scan"),
            ("/odom", "/odom"),
        ],
        condition=is_sim,
    )

    # ── [3b] async_slam_toolbox_node — HARDWARE MODE ─────────────────────────
    slam_toolbox_hw = LifecycleNode(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        namespace="",
        parameters=[
            ParameterFile(slam_params_hw, allow_substs=True),
            {
                "use_lifecycle_manager": use_lifecycle_manager,
                "use_sim_time": False,
            },
        ],
        remappings=[
            ("/scan", "/scan"),
            ("/odom", "/odom"),
        ],
        condition=is_hw,
    )

    # ── [4] CONFIGURE event ───────────────────────────────────────────────────
    configure_event_sim = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(slam_toolbox_sim),
            transition_id=Transition.TRANSITION_CONFIGURE,
        ),
        condition=IfCondition(
            AndSubstitution(
                AndSubstitution(autostart, NotSubstitution(use_lifecycle_manager)),
                PythonExpression(["'", mode, "' == 'sim'"])
            )
        ),
    )

    configure_event_hw = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(slam_toolbox_hw),
            transition_id=Transition.TRANSITION_CONFIGURE,
        ),
        condition=IfCondition(
            AndSubstitution(
                AndSubstitution(autostart, NotSubstitution(use_lifecycle_manager)),
                PythonExpression(["'", mode, "' == 'hw'"])
            )
        ),
    )

    # ── [5] ACTIVATE event ────────────────────────────────────────────────────
    activate_event_sim = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=slam_toolbox_sim,
            start_state="configuring",
            goal_state="inactive",
            entities=[
                LogInfo(msg="[EsiBot SLAM] slam_toolbox configured — activating now..."),
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(slam_toolbox_sim),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    )
                ),
            ],
        ),
        condition=IfCondition(
            AndSubstitution(
                AndSubstitution(autostart, NotSubstitution(use_lifecycle_manager)),
                PythonExpression(["'", mode, "' == 'sim'"])
            )
        ),
    )

    activate_event_hw = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=slam_toolbox_hw,
            start_state="configuring",
            goal_state="inactive",
            entities=[
                LogInfo(msg="[EsiBot SLAM] slam_toolbox configured — activating now..."),
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(slam_toolbox_hw),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    )
                ),
            ],
        ),
        condition=IfCondition(
            AndSubstitution(
                AndSubstitution(autostart, NotSubstitution(use_lifecycle_manager)),
                PythonExpression(["'", mode, "' == 'hw'"])
            )
        ),
    )

    # ── [6] RViz2 — OPTIONAL ─────────────────────────────────────────────────
    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[{
            "use_sim_time": PythonExpression(["'", mode, "' == 'sim'"])
        }],
        condition=IfCondition(use_rviz),
    )

    # ── [7] Keyboard Teleoperation — OPTIONAL ─────────────────────────────────
    teleop_node = Node(
        package="teleop_twist_keyboard",
        executable="teleop_twist_keyboard",
        name="teleop_twist_keyboard",
        output="screen",
        remappings=[("/cmd_vel", "/cmd_vel")],
        prefix="xterm -e",
        condition=IfCondition(teleop),
    )

    # ── Startup logs ──────────────────────────────────────────────────────────
    log_sim = LogInfo(
        msg=[
            "\n",
            "=======================================================\n",
            "  EsiBot SLAM — SIMULATION MODE (Gazebo Harmonic)\n",
            "=======================================================\n",
            "  Nodes:\n",
            "    • relay_node               : /ultrasound_raw → /scan\n",
            "    • async_slam_toolbox_node  (LifecycleNode, online_async)\n",
            "      lifecycle: CONFIGURE → ACTIVATE  (event-driven)\n",
            "  Config : slam_params_sim.yaml  |  use_sim_time: true\n",
            "\n",
            "  Prerequisite: ros2 launch esibot_gazebo sim.launch.py\n",
            "  All-in-one : ros2 launch esibot_slam slam_sim.launch.py\n",
            "=======================================================\n",
        ],
        condition=is_sim,
    )

    log_hw = LogInfo(
        msg=[
            "\n",
            "=======================================================\n",
            "  EsiBot SLAM — HARDWARE MODE (Raspberry Pi 4)\n",
            "=======================================================\n",
            "  Nodes:\n",
            "    • static_tf_laser_link     : laser_link TF fallback\n",
            "    • async_slam_toolbox_node  (LifecycleNode, online_async)\n",
            "      lifecycle: CONFIGURE → ACTIVATE  (event-driven)\n",
            "  Config : slam_params_hw.yaml  |  use_sim_time: false\n",
            "\n",
            "  Prerequisites:\n",
            "    ros2 launch esibot_bringup bringup.launch.py\n",
            "  Verify:\n",
            "    ros2 topic hz /scan   # ~0.5 Hz expected (HC-SR04 sweep)\n",
            "    ros2 topic hz /odom   # ~10+ Hz expected\n",
            "=======================================================\n",
        ],
        condition=is_hw,
    )

    return LaunchDescription([
        # Arguments
        mode_arg,
        autostart_arg,
        use_lifecycle_manager_arg,
        use_rviz_arg,
        teleop_arg,

        # Startup logs
        log_sim,
        log_hw,

        # Topic/TF fixes
        relay_node,            # sim only: /ultrasound_raw → /scan
        static_tf_laser_link,  # hw only:  laser_link TF fallback

        # slam_toolbox lifecycle nodes (only one active depending on mode)
        slam_toolbox_sim,
        slam_toolbox_hw,

        # Lifecycle transitions — official slam_toolbox pattern
        configure_event_sim,
        configure_event_hw,
        activate_event_sim,
        activate_event_hw,

        # Optional tools
        rviz2,
        teleop_node,
    ])