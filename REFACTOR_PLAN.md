# EsiBot Repository Finalization Plan (reviewed 2026-06-16)

## Summary

- Make `esibot_bringup` the canonical owner of the final one-command robot launcher (`robot.launch.py`).
- Keep Gazebo, RViz, and Foxglove available but fully optional and off by default.
- Clearly separate hardware, mock simulation, and Gazebo simulation behavior and parameters.
- Remove development-only scripts and stale docs after a verification pass.
- Fix launch/config mismatches, stale dashboard topics, and map/navigation path bugs discovered in the repo.
- Improve logging, package READMEs, and verification so the repo is teacher-ready and reproducible.

This document has been updated after a repository pass. The "Findings" section below lists concrete mismatches observed and exact files to change. The "Implementation Plan" that follows is ordered from low-risk edits to more invasive architectural changes so reviewers can land incremental, verifiable PRs.

## Public Interfaces

- Add canonical launcher: `esibot_bringup/launch/robot.launch.py` (top-level orchestrator).

  - Public launcher arguments (recommended API):
    - `runtime:=sim|hardware` (default `sim`) — overall runtime target.
    - `sim_backend:=mock|gazebo` (default `mock`) — simulation backend when `runtime:=sim`.
    - `mode:=base|slam|nav|vision` (default `base`) — high-level mode.
    - `dashboard:=true|false` (default `true`) — enable/disable HTTP dashboard.
    - `bridge:=true|false` (default `true`) — enable/disable rosbridge/foxglove.
    - `visualization:=none|rviz|foxglove|both` (default `none`) — visualization policy.
    - `log_level:=debug|info|warn|error` (default `info`) — global log verbosity.

  - Example canonical commands:
    - Mock/safe local run: `ros2 launch esibot_bringup robot.launch.py runtime:=sim sim_backend:=mock mode:=base`
    - Gazebo SLAM: `ros2 launch esibot_bringup robot.launch.py runtime:=sim sim_backend:=gazebo mode:=slam`
    - Pi hardware SLAM: `ros2 launch esibot_bringup robot.launch.py runtime:=hardware mode:=slam`
    - Pi navigation: `ros2 launch esibot_bringup robot.launch.py runtime:=hardware mode:=nav`

  - Keep `esibot_description/launch/full.launch.py` as a compatibility wrapper only (thin include), and do not recommend it for new docs.

- Add a simple hardware telemetry topic: `/esibot/servo_angle` as `std_msgs/Float32` (rad or degrees — choose one and document). This provides a single-field gauge for the dashboard while preserving `/joint_states` for TF.

## Implementation Changes (high-level)

The following changes are the actionable items discovered during the pass. Each item below references the files to change and the expected outcome.

1) Launch isolation and top-level launcher
  - Add `esibot_bringup/launch/robot.launch.py` that composes existing launch fragments (`bringup.launch.py`, `radar.launch.py`, `camera.launch.py`, `vision.launch.py`, `esibot_slam/launch/slam.launch.py`, `esibot_navigation/launch/nav2.launch.py`, `esibot_gazebo/launch/sim.launch.py`, `esibot_ui/launch/dashboard.launch.py`).
  - Behavior mapping: `runtime:=sim` → set `sim_mode:=true`; `sim_backend:=gazebo` → set `use_sim_time:=true` and include Gazebo-only pieces; `sim_backend:=mock` → simulation but no gazebo/clock.
  - Ensure visualization nodes (RViz, foxglove) only start when `visualization` enables them.

2) Fix full.launch.py → slam interaction (low risk)
  - Problem: `esibot_description/launch/full.launch.py` currently passes `{"mode": "hw"}` to `esibot_slam/launch/slam.launch.py` unconditionally, which forces hardware SLAM even in simulation.
  - Fix: pass `mode` to `slam.launch.py` computed from `sim_mode` (e.g. `sim` when `sim_mode==true`, else `hw`) or include two conditional IncludeLaunchDescription entries. File: [esibot_description/launch/full.launch.py](esibot_description/launch/full.launch.py#L1-L200).

3) Nav2 default map path (trivial)
  - Problem: `esibot_navigation/launch/nav2.launch.py` defaults to `esibot_slam/maps/esibot_map.yaml` which does not exist; actual map is in `esibot_navigation/maps/esibot_map.yaml`.
  - Fix: change `default_map = os.path.join(slam_pkg_share, 'maps', 'esibot_map.yaml')` → `default_map = os.path.join(pkg_share, 'maps', 'esibot_map.yaml')`.

4) Radar node parameterization and canonical pins (low → medium)
  - Problem: `esibot_sensors/launch/radar.launch.py` declares `servo_pin`, `trig_pin`, `echo_pin` but `esibot_sensors/esibot_sensors/radar_node.py` uses module-level constants `SERVO_PIN = 12, TRIG_PIN=23, ECHO_PIN=25` and does not read parameters. Documentation and readmes disagree on defaults (17/27/22 vs 12/23/25).
  - Fixes:
    - Update `radar_node.py` to `declare_parameter('servo_pin', 12)` etc., read the parameters in `_load_params()` and use instance attributes `self._servo_pin` etc. (replace module constants usages accordingly).
    - Canonical defaults: adopt the hardware constants used in code and docs: `servo_pin=12`, `trig_pin=23`, `echo_pin=25` (update `esibot_sensors/launch/radar.launch.py` default values and `esibot_sensors/README.md` to match).
    - Add a new lightweight topic `/esibot/servo_angle` (std_msgs/Float32) published at the same cadence as joint updates to make the dashboard gauge trivial.

5) Camera params and vision input (low → medium)
  - Problem: `esibot_camera/launch/camera.launch.py` embeds default parameters inline even though `esibot_camera/config/camera_params.yaml` exists. `esibot_vision/vision_node.py` currently pulls frames directly from the ESP32 stream.
  - Fixes:
    - Update `camera.launch.py` to load `camera_params.yaml` with `ParameterFile` / `PathJoinSubstitution` and allow launch-time overrides via `LaunchConfiguration`.
    - Refactor `vision_node` to accept either a `camera_image_topic` parameter or the existing ESP32-stream mode. Prefer `camera_image_topic` when provided so simulated, Gazebo, and hardware flows all produce `/camera/image_raw` for the vision pipeline.

6) Logging and startup banners (low)
  - Add consistent LogInfo startup banners in major launch files and nodes (bringup, slam, nav2, radar, camera, vision) indicating runtime, backend, key topics, and critical parameters.

7) Dashboard polishing and build (medium)
  - Update dashboard code to subscribe to `/esibot/servo_angle` (or the joint-state-based gauge if preferred). Confirm topic types and units (document clearly).
  - After edits, rebuild the Vite dashboard in `/dashboard` and replace `esibot_ui/web` with the dist output.

8) Cleanup and metadata fixes (low → medium)
  - Remove `tools/`, `launch_robot.sh`, and stale plan documents in `docs/plans/` after a final verification pass and preserving anything still referenced.
  - Remove redundant `CMakeLists.txt` from `esibot_camera` if the package is truly `ament_python` and not C++.
  - Align `package.xml` and `setup.py` versions across Python packages where appropriate.

9) Test, CI, and verification (always done after each PR)
  - Static checks: `colcon list`, Python compile checks for launch files and nodes, `npm run build` in `dashboard`.
  - Build: `rosdep install --from-paths src --ignore-src -r -y` then `colcon build --symlink-install`.
  - Smoke tests (see Test Plan section below).

## Dashboard Plan

- Keep the current dashboard layout but fix correctness, control defaults, and connectivity handling.

- Map and scan:
  - Preserve offscreen map rendering.
  - Ensure the `/scan` messages include `header` and that the dashboard draws scan using the best available TF-derived `laser_link` pose.
  - Subscribe to `/tf_static` and `/tf` and maintain a small transform cache. If laser TF is missing, clearly show degraded state and fall back to robot pose overlay only.

- Controls:
  - Add compact linear/angular speed controls for teleop and persist settings in localStorage.
  - Default teleop speeds must respect driver limits (document limits in `esibot_bringup/config`).
  - Maintain emergency stop as immediate and prominent.

- Status & topics:
  - Use `/esibot/servo_angle` for the servo gauge (publish Float32 in radians or degrees — pick one and document). Also keep `/joint_states` for TF continuity.
  - Prefer ROS camera topics (`/camera/image_raw`) in simulation; keep ESP32 MJPEG access as an optional hardware-only mode.

- Build:
  - After making UI changes in `/dashboard`, run `npm run build` and copy the `dist` output into [esibot_ui/web](esibot_ui/web).

## Cleanup And Docs

- Remove or archive development-only artifacts after verification:
  - `tools/`, `launch_robot.sh`, and stale plan files under `docs/plans/` (remove only after confirming nothing still references them).
  - Remove duplicate or unreferenced screenshots/maps in `docs/` after curation.

- Metadata and packaging fixes:
  - Ensure `esibot_sensors` license field in `package.xml` matches code license (e.g. `Apache-2.0`).
  - Align `package.xml` and `setup.py` versions across Python packages where appropriate.
  - Add missing `esibot_ui` runtime dependencies to its packaging metadata if the node requires message packages such as `geometry_msgs` or `nav2_msgs` for compilation or ros2 interface generation.

- Documentation:
  - Rewrite the root README to present the canonical launcher and three user workflows: local mock (safe), Gazebo, and Pi hardware.
  - Update each package README to document topics, parameters, launch arguments, and quick verification commands (one-liner `ros2` checks).
  - Update [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) to reflect the final runtime graph and remove developer-only script/tool sections.

## Test Plan

Static and build checks (CI):

- `colcon list`
- `python -m py_compile` on package `.py` files or a lightweight lint to ensure imports/launch wiring compile
- `npm --prefix dashboard run build`

Build and run (manual):

- Install deps: `rosdep install --from-paths src --ignore-src -r -y`
- Build: `colcon build --symlink-install`
- Run tests: `colcon test --event-handlers console_direct+`

Launch argument checks (after top-level launcher added):

- `ros2 launch esibot_bringup robot.launch.py --show-args`
- `ros2 launch esibot_gazebo sim.launch.py --show-args`
- `ros2 launch esibot_ui dashboard.launch.py --show-args`

Runtime smoke tests (explicit verification):

- Mock mode (no hardware):
  - `ros2 launch esibot_bringup robot.launch.py runtime:=sim sim_backend:=mock visualization:=none dashboard:=true`
  - Verify topics: `ros2 topic hz /odom /scan /battery_state /tf` and dashboard HTTP reachable.

- Gazebo mode:
  - `ros2 launch esibot_bringup robot.launch.py runtime:=sim sim_backend:=gazebo mode:=slam`
  - Verify: `/clock`, `/odom`, `/ultrasound_raw`, `/scan`, `/camera/image_raw` present; SLAM uses `use_sim_time=true` and no hardware nodes attempt serial.

- Hardware Pi (integration smoke):
  - `ros2 launch esibot_bringup robot.launch.py runtime:=hardware visualization:=none`
  - Verify serial/GPIO initialization logs, radar sweeps, camera reconnect, SLAM or Nav2 startup as appropriate.

- Dashboard checks:
  - Verify servo gauge from `/esibot/servo_angle`, map overlay, scan overlay, teleop controls, and save map behavior.

## Assumptions

- Target platform: ROS 2 Jazzy on Ubuntu 24.04; Gazebo Harmonic optional when `sim_backend:=gazebo`.
- Canonical radar wiring chosen for the final repo: servo GPIO12, trig GPIO23, echo GPIO25 (this matches `esibot_sensors/esibot_sensors/radar_node.py` constant wiring). Update docs/launch files to match.
- `esibot_description/full.launch.py` will remain available as a compatibility wrapper but not the recommended user entrypoint.

If you want, I can now:

- 1) Apply the `PLAN.md` changes I just wrote (done),
- 2) Open PR-ready patches for the highest-impact fixes (`nav2` map path, `full.launch` slam mode, `radar_node` parameterization, `camera.launch` ParameterFile change), or
- 3) Run the static build checks described above in your workspace and report failures.

Which of these should I do next?
