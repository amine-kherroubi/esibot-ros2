"""
esibot_slam — launch/slam.launch.py
=====================================
Main SLAM launch file for the EsiBot robot (Task 3.5).

=== LIFECYCLE MANAGEMENT — OFFICIAL slam_toolbox PATTERN ===

async_slam_toolbox_node is a ROS 2 Lifecycle Node. It must go through
two transitions before it starts processing scans:

  unconfigured ──CONFIGURE──> inactive ──ACTIVATE──> active

This file uses the OFFICIAL pattern from SteveMacenski's slam_toolbox
repository (online_async_launch.py), adapted for EsiBot:

  1. Declare slam_toolbox as a LifecycleNode  (not a plain Node)
  2. EmitEvent(TRANSITION_CONFIGURE)          fired immediately at launch
  3. RegisterEventHandler(OnStateTransition)  waits for configuring → inactive
     then fires EmitEvent(TRANSITION_ACTIVATE)

This is self-contained — no external nav2_lifecycle_manager is needed.

The `use_lifecycle_manager` parameter (default: false):
  • false → this launch file drives the transitions itself (our case)
  • true  → an external lifecycle manager (e.g. Nav2) drives the transitions
            and holds a bond connection. Set this to true only if you later
            integrate slam_toolbox into a full Nav2 bringup that manages it.

=== MODES ===

  sim (default) — Gazebo Harmonic simulation
    • use_sim_time = true
    • relay_node: /ultrasound_raw → /scan
      gz_bridge publishes the Gazebo gpu_lidar scan on /ultrasound_raw;
      slam_toolbox and the rest of the stack expect /scan.

  hw — Real hardware (Raspberry Pi 4)
    • use_sim_time = false
    • static_transform_publisher: zero TF ultrasound_sensor → laser_link
      radar_node (esibot_sensors) uses frame_id='laser_link',
      but the EsiBot URDF defines the sensor link as 'ultrasound_sensor'.
      Without this fix slam_toolbox raises a TF lookup error.

=== NODES LAUNCHED ===

  1. relay_node               (topic_tools/relay)                  — SIM ONLY
  2. static_tf_laser_link     (tf2_ros/static_transform_publisher) — HW ONLY
  3. async_slam_toolbox_node  (slam_toolbox, LifecycleNode)        — ALWAYS
  4. configure_event          (EmitEvent CONFIGURE)                — ALWAYS
  5. activate_event           (RegisterEventHandler + EmitEvent)   — ALWAYS
  6. rviz2                                                          — optional
  7. teleop_twist_keyboard                                          — optional

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
            "'sim' = Gazebo Harmonic simulation (use_sim_time=true). "
            "'hw'  = Real Raspberry Pi 4 hardware (use_sim_time=false)."
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

    mode                 = LaunchConfiguration("mode")
    autostart            = LaunchConfiguration("autostart")
    use_lifecycle_manager = LaunchConfiguration("use_lifecycle_manager")
    use_rviz             = LaunchConfiguration("use_rviz")
    teleop               = LaunchConfiguration("teleop")

    # PythonExpression conditions — compatible with all ROS 2 Galactic+
    is_sim = IfCondition(PythonExpression(["'", mode, "' == 'sim'"]))
    is_hw  = IfCondition(PythonExpression(["'", mode, "' == 'hw'"]))

    # Condition: autostart=true AND use_lifecycle_manager=false
    # This matches the official slam_toolbox pattern exactly.
    # When use_lifecycle_manager=true the external manager drives transitions.
    do_autostart = IfCondition(
        AndSubstitution(autostart, NotSubstitution(use_lifecycle_manager))
    )

    # ── [1] relay_node — SIMULATION ONLY ────────────────────────────────────
    #
    # gz_bridge (esibot_gazebo/sim.launch.py) maps the Gazebo gpu_lidar to:
    #   ROS 2 topic /ultrasound_raw  (sensor_msgs/LaserScan)
    #
    # slam_toolbox expects the standard /scan topic.
    # This relay forwards every message from /ultrasound_raw → /scan.
    relay_node = Node(
        package="topic_tools",
        executable="relay",
        name="ultrasound_to_scan_relay",
        output="screen",
        arguments=["/ultrasound_raw", "/scan"],
        condition=is_sim,
    )

    # ── [2] static_tf_laser_link — HARDWARE ONLY ─────────────────────────────
    #
    # radar_node.py (esibot_sensors) publishes LaserScan with frame_id='laser_link'.
    # The EsiBot URDF defines the sensor as link 'ultrasound_sensor'.
    # → slam_toolbox cannot resolve: base_footprint → ... → laser_link
    #
    # Fix: zero-transform  ultrasound_sensor (URDF) → laser_link (radar_node)
    # This attaches laser_link to the TF tree without moving it.
    #
    # Permanent fix: edit radar_node.py to use frame_id='ultrasound_sensor'.
    static_tf_laser_link = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_tf_ultrasound_to_laser_link",
        output="screen",
        # format: x y z yaw pitch roll parent_frame child_frame
        arguments=["0", "0", "0", "0", "0", "0",
                   "ultrasound_sensor", "laser_link"],
        condition=is_hw,
    )

    # ── [3a] async_slam_toolbox_node — SIMULATION MODE ───────────────────────
    #
    # Declared as LifecycleNode — the correct ROS 2 type for lifecycle nodes.
    # This is the key difference from using a plain Node:
    #   • LifecycleNode gives us handles for EmitEvent(ChangeState)
    #   • Plain Node does not expose lifecycle transitions in the launch graph
    #
    # ParameterFile with allow_substs=True: allows $(find-pkg-share) style
    # substitutions inside the YAML file (consistent with official slam_toolbox).
    #
    # use_lifecycle_manager is passed INTO the node as a parameter.
    # When true, slam_toolbox enables the bond heartbeat for the external manager.
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

    # ── [4] CONFIGURE event — fired immediately after node starts ─────────────
    #
    # EmitEvent sends the CONFIGURE transition signal to slam_toolbox.
    # slam_toolbox moves: unconfigured → configuring → inactive
    # During this transition it loads YAML parameters and allocates memory.
    #
    # Condition: only if autostart=true AND use_lifecycle_manager=false.
    # When use_lifecycle_manager=true, the external manager sends this instead.
    #
    # matches_action(slam_toolbox_sim) targets the specific LifecycleNode
    # instance — if both sim and hw nodes were somehow running, only the
    # correct one would receive the event (defensive coding).
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

    # ── [5] ACTIVATE event — fired when CONFIGURE completes ──────────────────
    #
    # RegisterEventHandler monitors the slam_toolbox lifecycle state machine.
    # When it detects the transition: configuring → inactive (CONFIGURE done),
    # it immediately fires TRANSITION_ACTIVATE.
    # slam_toolbox moves: inactive → activating → active
    # During activation it subscribes to /scan and /odom, starts publishing
    # /map, and broadcasts the TF map→odom.
    #
    # This is an event-driven chain: CONFIGURE completes → ACTIVATE fires.
    # No sleep() or polling required — purely reactive.
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
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(use_rviz),
    )

    # ── [7] Keyboard Teleoperation — OPTIONAL ─────────────────────────────────
    # Controls: i=forward, ,=backward, j=left, l=right, k=stop
    # Requires: sudo apt install xterm
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
            "    • relay_node          : /ultrasound_raw → /scan\n",
            "    • async_slam_toolbox_node  (LifecycleNode)\n",
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
            "    • static_tf  ultrasound_sensor → laser_link\n",
            "    • async_slam_toolbox_node  (LifecycleNode)\n",
            "      lifecycle: CONFIGURE → ACTIVATE  (event-driven)\n",
            "  Config : slam_params_hw.yaml  |  use_sim_time: false\n",
            "\n",
            "  Prerequisites:\n",
            "    ros2 launch esibot_bringup bringup.launch.py\n",
            "  Verify:\n",
            "    ros2 topic hz /scan   # ~1–2 Hz expected\n",
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

        # Logs
        log_sim,
        log_hw,

        # EsiBot-specific topic/TF fixes
        relay_node,            # sim only: /ultrasound_raw → /scan
        static_tf_laser_link,  # hw only:  ultrasound_sensor → laser_link

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
