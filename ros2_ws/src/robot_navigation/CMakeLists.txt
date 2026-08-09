cmake_minimum_required(VERSION 3.8)
project(robot_navigation)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_core REQUIRED)

add_library(attempt27_cycle_trace SHARED src/attempt27_cycle_trace.cpp)
target_compile_features(attempt27_cycle_trace PUBLIC cxx_std_17)
target_link_libraries(attempt27_cycle_trace dl)
ament_target_dependencies(attempt27_cycle_trace geometry_msgs nav2_core)

add_library(attempt28_cycle_trace SHARED src/attempt28_cycle_trace.cpp)
target_compile_features(attempt28_cycle_trace PUBLIC cxx_std_17)
target_link_libraries(attempt28_cycle_trace dl)
ament_target_dependencies(attempt28_cycle_trace geometry_msgs nav2_core)

install(DIRECTORY behavior_trees config launch
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
)

install(TARGETS attempt27_cycle_trace attempt28_cycle_trace
  LIBRARY DESTINATION lib
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_nav2_config test/test_nav2_config.py)

  add_library(attempt27_cycle_trace_original_mock SHARED
    test/attempt27_cycle_trace_original_mock.cpp)
  target_compile_features(attempt27_cycle_trace_original_mock PUBLIC cxx_std_17)
  ament_target_dependencies(attempt27_cycle_trace_original_mock
    geometry_msgs nav2_core)

  add_executable(attempt27_cycle_trace_probe
    test/attempt27_cycle_trace_probe.cpp)
  target_compile_features(attempt27_cycle_trace_probe PUBLIC cxx_std_17)
  target_link_libraries(attempt27_cycle_trace_probe dl)
  ament_target_dependencies(attempt27_cycle_trace_probe
    geometry_msgs nav2_core)

  add_test(NAME attempt27_cycle_trace_equivalence
    COMMAND ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/test_attempt27_cycle_trace.py
      $<TARGET_FILE:attempt27_cycle_trace_probe>
      $<TARGET_FILE:attempt27_cycle_trace_original_mock>
      $<TARGET_FILE:attempt27_cycle_trace>)

  add_test(NAME attempt28_cycle_trace_equivalence
    COMMAND ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/test_attempt28_cycle_trace.py
      $<TARGET_FILE:attempt27_cycle_trace_probe>
      $<TARGET_FILE:attempt27_cycle_trace_original_mock>
      $<TARGET_FILE:attempt28_cycle_trace>)

  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
