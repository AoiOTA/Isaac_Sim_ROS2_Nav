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
find_package(nav2_mppi_controller REQUIRED)
find_package(nav2_smac_planner REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rcutils REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)

add_library(bio_nav_planner_plugin SHARED
  src/bio_nav_grid_based.cpp
  src/tie_break_smac_planner_2d.cpp
)
target_include_directories(bio_nav_planner_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(bio_nav_planner_plugin
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
  visualization_msgs
)

add_library(bio_nav_costmap_plugin SHARED
  src/cognitive_obstacle_layer.cpp
  src/cognitive_risk_layer.cpp
  src/local_risk_grid_layer.cpp
  src/reachability_observer_layer.cpp
)

add_executable(local_risk_grid_shadow_auditor
  src/local_risk_grid_shadow_auditor.cpp)
target_link_libraries(
  local_risk_grid_shadow_auditor bio_nav_costmap_plugin)
ament_target_dependencies(local_risk_grid_shadow_auditor
  bio_nav_interfaces rclcpp std_msgs tf2 tf2_ros)
target_include_directories(bio_nav_costmap_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(bio_nav_costmap_plugin
  bio_nav_interfaces
  geometry_msgs
  nav2_costmap_2d
  nav2_util
  nav_msgs
  pluginlib
  rclcpp
  rclcpp_lifecycle
  std_msgs
  tf2
  tf2_geometry_msgs
  visualization_msgs
)

add_library(bio_nav_mppi_critic_plugin SHARED
  src/cognitive_risk_critic.cpp
)
target_include_directories(bio_nav_mppi_critic_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(bio_nav_mppi_critic_plugin
  bio_nav_interfaces
  geometry_msgs
  nav2_costmap_2d
  nav2_mppi_controller
  pluginlib
  rclcpp
  rclcpp_lifecycle
  tf2
  tf2_geometry_msgs
  tf2_ros
)

pluginlib_export_plugin_description_file(nav2_core bio_nav_planners.xml)
pluginlib_export_plugin_description_file(
  nav2_costmap_2d bio_nav_costmap_plugins.xml)
pluginlib_export_plugin_description_file(
  nav2_mppi_controller bio_nav_mppi_critics.xml)

install(TARGETS
  bio_nav_planner_plugin
  bio_nav_costmap_plugin
  bio_nav_mppi_critic_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS local_risk_grid_shadow_auditor
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(FILES
  bio_nav_planners.xml bio_nav_costmap_plugins.xml bio_nav_mppi_critics.xml
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_equal_cost_search test/test_equal_cost_search.cpp)
  target_link_libraries(
    test_equal_cost_search
    bio_nav_planner_plugin
    bio_nav_costmap_plugin
    bio_nav_mppi_critic_plugin
  )
  ament_target_dependencies(test_equal_cost_search
    geometry_msgs nav2_costmap_2d rclcpp)

  ament_add_gtest(
    test_plugin_loader_isolation test/test_plugin_loader_isolation.cpp)
  ament_target_dependencies(test_plugin_loader_isolation
    nav2_core nav2_costmap_2d nav2_mppi_controller pluginlib)
endif()

ament_export_include_directories(include)
ament_export_libraries(
  bio_nav_planner_plugin bio_nav_costmap_plugin bio_nav_mppi_critic_plugin)
ament_export_dependencies(
  bio_nav_interfaces nav2_core nav2_costmap_2d nav2_mppi_controller
  nav2_smac_planner tf2_ros)
ament_package()
