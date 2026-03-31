cmake_minimum_required(VERSION 3.8)
project(esibot_camera)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ── Find dependencies ──────────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

# ── Install Python package ─────────────────────────────────────────────────
ament_python_install_package(${PROJECT_NAME})

# ── Install Python executables ─────────────────────────────────────────────
install(PROGRAMS
  esibot_camera/camera_stream_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME camera_stream_node
)

# ── Install launch files ───────────────────────────────────────────────────
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# ── Install config files ───────────────────────────────────────────────────
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
