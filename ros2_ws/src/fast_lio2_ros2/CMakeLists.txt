cmake_minimum_required(VERSION 3.16)
project(fast_lio2_ros2)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(OpenMP REQUIRED COMPONENTS CXX)
find_package(PCL REQUIRED COMPONENTS common filters io)
find_package(pcl_conversions REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_ros REQUIRED)

include(ProcessorCount)
ProcessorCount(PROCESSOR_COUNT)
if(PROCESSOR_COUNT GREATER 4)
  add_compile_definitions(MP_EN MP_PROC_NUM=3)
elseif(PROCESSOR_COUNT GREATER 3)
  add_compile_definitions(MP_EN MP_PROC_NUM=2)
else()
  add_compile_definitions(MP_PROC_NUM=1)
endif()

add_library(fast_lio2_preprocess src/preprocess.cpp)
target_include_directories(fast_lio2_preprocess PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS}
)
ament_target_dependencies(fast_lio2_preprocess
  pcl_conversions
  rclcpp
  sensor_msgs
)
target_link_libraries(fast_lio2_preprocess
  ${PCL_LIBRARIES}
  Eigen3::Eigen
  OpenMP::OpenMP_CXX
)

add_executable(fastlio_mapping
  src/laserMapping.cpp
  include/ikd-Tree/ikd_Tree.cpp
)
target_compile_definitions(fastlio_mapping PRIVATE
  ROOT_DIR="${CMAKE_CURRENT_SOURCE_DIR}/"
)
target_include_directories(fastlio_mapping PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/src
  ${PCL_INCLUDE_DIRS}
)
ament_target_dependencies(fastlio_mapping
  geometry_msgs
  nav_msgs
  pcl_conversions
  rclcpp
  sensor_msgs
  std_srvs
  tf2_ros
)
target_link_libraries(fastlio_mapping
  fast_lio2_preprocess
  ${PCL_LIBRARIES}
  Eigen3::Eigen
  OpenMP::OpenMP_CXX
)

install(TARGETS
  fast_lio2_preprocess
  fastlio_mapping
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)
install(FILES LICENSE ORIGIN.md README.md DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)

  ament_add_gtest(test_ouster_preprocess test/test_ouster_preprocess.cpp)
  target_include_directories(test_ouster_preprocess PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${PCL_INCLUDE_DIRS}
  )
  ament_target_dependencies(test_ouster_preprocess
    pcl_conversions
    rclcpp
    sensor_msgs
  )
  target_link_libraries(test_ouster_preprocess
    fast_lio2_preprocess
    ${PCL_LIBRARIES}
    Eigen3::Eigen
  )

  ament_add_pytest_test(
    test_shadow_contract
    test/test_shadow_contract.py
    TIMEOUT 30
  )
endif()

ament_package()
