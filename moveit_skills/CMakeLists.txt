cmake_minimum_required(VERSION 3.8)
project(moveit_skills)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(rmw_implementation_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)

include_directories(
  include
)

set(dependencies
  rclcpp
  geometry_msgs
  moveit_ros_planning
  nav2_util
)

set(executable_name moveit_skill_server_node)
set(library_name ${executable_name}_core)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/ComputePathToState.action"
  "action/ComputePathToPoint.action"
  "msg/PlanConfig.msg"
  "msg/ComputePathResult.msg"
  DEPENDENCIES
  builtin_interfaces
  std_msgs
  geometry_msgs
  moveit_msgs
)

# if define msg, action. service in the same pkg
# get the rmw implementations ahead of time
find_package(rmw_implementation_cmake REQUIRED)
get_available_rmw_implementations(rmw_implementations2)
foreach(rmw_implementation ${rmw_implementations2})
  find_package("${rmw_implementation}" REQUIRED)
endforeach()

rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")

add_library(compute_path_to_state_action_server SHARED
src/compute_path_to_state_action_server.cpp)
ament_target_dependencies(compute_path_to_state_action_server
  ${dependencies}
)
target_link_libraries(compute_path_to_state_action_server "${cpp_typesupport_target}")
list(APPEND libs compute_path_to_state_action_server)

add_library(compute_path_to_point_action_server SHARED
src/compute_path_to_point_action_server.cpp)
ament_target_dependencies(compute_path_to_point_action_server
  ${dependencies}
)
target_link_libraries(compute_path_to_point_action_server "${cpp_typesupport_target}")
list(APPEND libs compute_path_to_point_action_server)

add_library(moveit_skill_server SHARED
src/moveit_skill_server.cpp)
ament_target_dependencies(moveit_skill_server
  ${dependencies}
)
target_link_libraries(moveit_skill_server
compute_path_to_state_action_server
compute_path_to_point_action_server)
list(APPEND libs moveit_skill_server)

add_executable(${executable_name} src/main_moveit_skill_server.cpp)
target_link_libraries(${executable_name}
moveit_skill_server)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME})

install(TARGETS ${executable_name}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS ${libs}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
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
  ${libs}
)

ament_export_dependencies(${dependencies})
ament_export_dependencies(rosidl_default_runtime)

ament_package()
