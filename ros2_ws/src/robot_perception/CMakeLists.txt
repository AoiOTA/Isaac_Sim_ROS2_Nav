cmake_minimum_required(VERSION 3.8)
project(robot_perception)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)

add_library(lidar_self_filter_core
  src/lidar_self_filter_core.cpp
)
target_compile_features(lidar_self_filter_core PUBLIC cxx_std_17)
target_include_directories(lidar_self_filter_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(lidar_self_filter_core
  sensor_msgs
)

add_executable(lidar_self_filter
  src/lidar_self_filter_node.cpp
)
target_compile_features(lidar_self_filter PUBLIC cxx_std_17)
target_link_libraries(lidar_self_filter
  lidar_self_filter_core
)
ament_target_dependencies(lidar_self_filter
  rclcpp
  sensor_msgs
  tf2
  tf2_ros
  tf2_sensor_msgs
)

install(
  TARGETS lidar_self_filter_core lidar_self_filter
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_lidar_self_filter_core
    test/test_lidar_self_filter_core.cpp
  )
  target_link_libraries(test_lidar_self_filter_core
    lidar_self_filter_core
  )
  ament_target_dependencies(test_lidar_self_filter_core
    sensor_msgs
  )

  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_projection_config test/test_projection_config.py)

  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_dependencies(sensor_msgs)

ament_package()
