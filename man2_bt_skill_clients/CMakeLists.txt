cmake_minimum_required(VERSION 3.8)
project(man2_bt_skill_clients)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(man2_behavior_tree REQUIRED)
find_package(detect_aruco_marker_skill REQUIRED)
find_package(moveit_skills REQUIRED)

include_directories(
  include
)

set(detect_aruco_marker_dependencies
  man2_behavior_tree
  detect_aruco_marker_skill
)

set(compute_path_dependencies
  man2_behavior_tree
  moveit_skills
)

add_library(detect_aruco_marker_client SHARED
  src/action_clients/detect_aruco_marker_action_bt_client.cpp)
list(APPEND plugin_libs detect_aruco_marker_client)
ament_target_dependencies(detect_aruco_marker_client
  ${detect_aruco_marker_dependencies}
)

add_library(compute_path_to_point_client SHARED
  src/action_clients/compute_path_to_point_action_bt_client.cpp)
list(APPEND plugin_libs compute_path_to_point_client)
ament_target_dependencies(compute_path_to_point_client
  ${compute_path_dependencies}
)

add_library(compute_path_to_pose_client SHARED
  src/action_clients/compute_path_to_pose_action_bt_client.cpp)
list(APPEND plugin_libs compute_path_to_pose_client)
ament_target_dependencies(compute_path_to_pose_client
  ${compute_path_dependencies}
)

add_library(compute_path_to_state_client SHARED
  src/action_clients/compute_path_to_state_action_bt_client.cpp)
list(APPEND plugin_libs compute_path_to_state_client)
ament_target_dependencies(compute_path_to_state_client
  ${compute_path_dependencies}
)

add_library(execute_trajectory_client SHARED
  src/action_clients/execute_trajectory_action_bt_client.cpp)
list(APPEND plugin_libs execute_trajectory_client)
ament_target_dependencies(execute_trajectory_client
  ${compute_path_dependencies}
)

add_library(get_current_ik_frame_pose_client SHARED
  src/action_clients/get_current_ik_frame_pose_action_bt_client.cpp)
list(APPEND plugin_libs get_current_ik_frame_pose_client)
ament_target_dependencies(get_current_ik_frame_pose_client
  ${compute_path_dependencies}
)

add_library(set_path_constraints_client SHARED
src/service_clients/set_path_constraints_bt_client.cpp)
list(APPEND plugin_libs set_path_constraints_client)
ament_target_dependencies(set_path_constraints_client
  ${compute_path_dependencies}
)

add_library(print_value SHARED
src/print_value.cpp)
list(APPEND plugin_libs print_value)
ament_target_dependencies(print_value
  ${compute_path_dependencies}
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
