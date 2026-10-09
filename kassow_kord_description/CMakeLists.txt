cmake_minimum_required(VERSION 3.5)
project(kassow_kord_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY
  kr810/meshes kr810/urdf kr810/rviz kr810/srdf
  DESTINATION share/${PROJECT_NAME}/kr810
)

install(
  DIRECTORY
  kr1018/meshes kr1018/urdf kr1018/srdf
  DESTINATION share/${PROJECT_NAME}/kr1018
)

ament_package()
