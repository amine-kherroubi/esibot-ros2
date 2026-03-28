cmake_minimum_required(VERSION 3.8)
project(esibot_slam)

# ── Compiler flags ─────────────────────────────────────────────────────────────
# Enable C++17 with strict warnings to catch issues at compile time
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ── Dependencies ───────────────────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(slam_toolbox REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# ── Install resources ──────────────────────────────────────────────────────────
# Copy config/, launch/, and maps/ to share/esibot_slam/ inside the ROS2 workspace.
# These directories are then accessible at runtime via:
#   get_package_share_directory("esibot_slam")
install(
  DIRECTORY
    config
    launch
    maps
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
