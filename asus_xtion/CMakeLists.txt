
cmake_minimum_required(VERSION 3.5)
project(asus_xtion)

find_package(ament_cmake REQUIRED)
find_package(openni2_camera REQUIRED)
find_package(asus_xtion_description REQUIRED)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
