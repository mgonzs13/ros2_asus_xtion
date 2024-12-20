
cmake_minimum_required(VERSION 3.5)
project(asus_xtion_visualization)

find_package(ament_cmake REQUIRED)
find_package(rviz2 REQUIRED)

install(
  DIRECTORY launch rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
