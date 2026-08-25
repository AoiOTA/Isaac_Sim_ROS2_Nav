cmake_minimum_required(VERSION 3.16)
project(pointcloud_local_odometry)

find_package(ament_cmake REQUIRED)
install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)

ament_package()
