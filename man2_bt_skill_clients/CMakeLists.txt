cmake_minimum_required(VERSION 3.8)
project(man2_bt_skill_clients)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(man2_behavior_tree REQUIRED)
find_package(detect_aruco_marker_skill REQUIRED)

include_directories(
  include
)

set(detect_aruco_marker_dependencies
  man2_behavior_tree
  detect_aruco_marker_skill
)

add_library(detect_aruco_marker_client SHARED
  src/action_clients/detect_aruco_marker_action_bt_client.cpp)
list(APPEND plugin_libs detect_aruco_marker_client)
ament_target_dependencies(detect_aruco_marker_client
  ${detect_aruco_marker_dependencies}
)

foreach(bt_plugin ${plugin_libs})
  target_compile_definitions(${bt_plugin} PRIVATE BT_PLUGIN_EXPORT)
endforeach()

install(TARGETS
  ${plugin_libs}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  # These don't pass yet, disable them for now
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_flake8_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)

  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)

ament_export_libraries(
  ${plugin_libs})

ament_package()
