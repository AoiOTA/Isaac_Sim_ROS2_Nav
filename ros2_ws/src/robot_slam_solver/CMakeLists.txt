cmake_minimum_required(VERSION 3.16)
project(robot_slam_solver)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(Boost REQUIRED COMPONENTS serialization thread)
find_package(Ceres REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(slam_toolbox REQUIRED)

find_library(KARTO_SLAM_TOOLBOX_LIBRARY kartoSlamToolbox
  PATHS "${slam_toolbox_DIR}/../../../lib"
  NO_DEFAULT_PATH
  REQUIRED)

add_library(configurable_ceres_solver_plugin SHARED
  src/ceres_solver.cpp)
target_include_directories(configurable_ceres_solver_plugin
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  PRIVATE
    ${slam_toolbox_INCLUDE_DIRS})
ament_target_dependencies(configurable_ceres_solver_plugin
  pluginlib
  rclcpp
  rclcpp_lifecycle)
target_link_libraries(configurable_ceres_solver_plugin
  Boost::serialization
  Boost::thread
  Ceres::ceres
  Eigen3::Eigen
  ${KARTO_SLAM_TOOLBOX_LIBRARY})

pluginlib_export_plugin_description_file(slam_toolbox solver_plugins.xml)

install(TARGETS configurable_ceres_solver_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/
  DESTINATION include)
install(FILES LICENSE NOTICE.md
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_solver_contract test/test_solver_contract.py)
endif()

ament_export_dependencies(pluginlib rclcpp rclcpp_lifecycle)
ament_package()
