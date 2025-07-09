
cmake_minimum_required(VERSION 3.5)
project(asus_xtion_description)

find_package(ament_cmake REQUIRED)
find_package(xacro REQUIRED)

install(
  DIRECTORY launch meshes urdf robots
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
