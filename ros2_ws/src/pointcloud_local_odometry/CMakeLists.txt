cmake_minimum_required(VERSION 3.16)
project(pointcloud_local_odometry)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(PCL 1.14 REQUIRED COMPONENTS common filters registration)
find_package(pcl_conversions REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_library(ndt_odometry src/ndt_odometry.cpp)
target_include_directories(ndt_odometry PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS}
)
target_link_libraries(ndt_odometry
  ${PCL_LIBRARIES}
  Eigen3::Eigen
)

add_executable(ndt_local_odometry_node src/ndt_local_odometry_node.cpp)
target_include_directories(ndt_local_odometry_node PRIVATE
  ${PCL_INCLUDE_DIRS}
)
ament_target_dependencies(ndt_local_odometry_node
  diagnostic_msgs
  geometry_msgs
  nav_msgs
  pcl_conversions
  rclcpp
  sensor_msgs
  tf2
  tf2_ros
)
target_link_libraries(ndt_local_odometry_node
  ndt_odometry
  ${PCL_LIBRARIES}
  Eigen3::Eigen
)

install(TARGETS
  ndt_local_odometry_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)

  ament_add_gtest(test_ndt_odometry test/test_ndt_odometry.cpp)
  target_include_directories(test_ndt_odometry PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${PCL_INCLUDE_DIRS}
  )
  target_link_libraries(test_ndt_odometry
    ndt_odometry
    ${PCL_LIBRARIES}
    Eigen3::Eigen
  )

  ament_add_pytest_test(
    test_contract
    test/test_contract.py
    TIMEOUT 30
  )
endif()

ament_package()
