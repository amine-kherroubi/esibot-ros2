cmake_minimum_required(VERSION 3.8)
project(esibot_gazebo)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch worlds config models
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

ament_package()
