cmake_minimum_required(VERSION 3.8)
project(bio_nav_fusion)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(bio_nav_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_smac_planner REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_library(bio_nav_fusion_plugins SHARED
  src/bio_nav_grid_based.cpp
  src/cognitive_risk_layer.cpp
)
target_include_directories(bio_nav_fusion_plugins PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(bio_nav_fusion_plugins
  bio_nav_interfaces
  geometry_msgs
  nav2_core
  nav2_costmap_2d
  nav2_smac_planner
  nav2_util
  nav_msgs
  pluginlib
  rclcpp
  rclcpp_lifecycle
  std_msgs
  tf2
  tf2_geometry_msgs
)

pluginlib_export_plugin_description_file(nav2_core bio_nav_planners.xml)
pluginlib_export_plugin_description_file(
  nav2_costmap_2d bio_nav_costmap_plugins.xml)

install(TARGETS bio_nav_fusion_plugins
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)
install(FILES bio_nav_planners.xml bio_nav_costmap_plugins.xml
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_equal_cost_search test/test_equal_cost_search.cpp)
  target_link_libraries(test_equal_cost_search bio_nav_fusion_plugins)
  ament_target_dependencies(test_equal_cost_search
    geometry_msgs nav2_costmap_2d rclcpp)
endif()

ament_export_include_directories(include)
ament_export_libraries(bio_nav_fusion_plugins)
ament_export_dependencies(
  bio_nav_interfaces nav2_core nav2_costmap_2d nav2_smac_planner)
ament_package()
