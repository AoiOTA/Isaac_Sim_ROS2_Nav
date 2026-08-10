cmake_minimum_required(VERSION 3.8)
project(bio_nav_a20_nav2_plugins)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_mppi_controller REQUIRED)
find_package(nav2_util REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/tube_progress_critic.cpp
  src/box_goal_checker.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME}
  geometry_msgs
  nav2_core
  nav2_costmap_2d
  nav2_mppi_controller
  nav2_util
  pluginlib
  rclcpp
  rclcpp_lifecycle
  tf2
  tf2_geometry_msgs
)

pluginlib_export_plugin_description_file(
  nav2_mppi_controller bio_nav_a20_nav2_plugins.xml)
pluginlib_export_plugin_description_file(
  nav2_core bio_nav_a20_nav2_plugins.xml)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)
install(FILES bio_nav_a20_nav2_plugins.xml
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_path_coverage test/test_path_coverage.cpp)
  target_include_directories(test_path_coverage PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include)

  ament_add_gtest(test_box_goal_checker test/test_box_goal_checker.cpp)
  target_link_libraries(test_box_goal_checker ${PROJECT_NAME})
  ament_target_dependencies(test_box_goal_checker
    geometry_msgs nav2_core rclcpp tf2 tf2_geometry_msgs)
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  nav2_core nav2_mppi_controller pluginlib rclcpp_lifecycle tf2_geometry_msgs)
ament_package()
