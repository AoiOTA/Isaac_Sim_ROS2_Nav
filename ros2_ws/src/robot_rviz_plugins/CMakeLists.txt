cmake_minimum_required(VERSION 3.8)
project(robot_rviz_plugins)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_lifecycle_manager REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Core Gui Widgets Concurrent)
find_package(nav2_rviz_plugins REQUIRED)
find_package(nav2_util REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_default_plugins REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)

set(headers_to_moc
  include/robot_rviz_plugins/nav2_panel.hpp
  include/robot_rviz_plugins/voxel_grid_display.hpp
)

add_library(${PROJECT_NAME} SHARED
  src/nav2_panel.cpp
  src/voxel_grid_display.cpp
  ${headers_to_moc}
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${Qt5Widgets_INCLUDE_DIRS}
)

ament_target_dependencies(${PROJECT_NAME}
  geometry_msgs
  nav2_lifecycle_manager
  nav2_msgs
  nav2_rviz_plugins
  nav2_util
  pluginlib
  Qt5
  rclcpp
  rclcpp_action
  rviz_common
  rviz_default_plugins
  sensor_msgs
  tf2_geometry_msgs
  tf2_ros
  visualization_msgs
  yaml_cpp_vendor
)

target_link_libraries(${PROJECT_NAME}
  nav2_rviz_plugins::nav2_rviz_plugins
  rviz_common::rviz_common
)

pluginlib_export_plugin_description_file(
  rviz_common plugins_description.xml)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)
install(DIRECTORY include/ DESTINATION include/)
install(FILES LICENSE NOTICE.md DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(
    test_safe_panel_contract test/test_safe_panel_contract.py)
endif()

ament_export_include_directories(include)
ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(rviz_common)
ament_package()
