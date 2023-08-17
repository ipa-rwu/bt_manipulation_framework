cmake_minimum_required(VERSION 3.8)
project(detect_aruco_marker_skill)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(man2_skill_server_core REQUIRED)
find_package(rmw_implementation_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(aruco_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

include_directories(
  include
)

set(detect_aruco_marker_action_server_dependencies
  rclcpp
  aruco_msgs
  geometry_msgs
  man2_skill_server_core
)

set(executable_name_detect_aruco_marker_action_server detect_aruco_marker_action_server)
set(library_name_detect_aruco_marker_action_server ${executable_name_detect_aruco_marker_action_server}_core)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/ArucoMarkerDetection.action"
  DEPENDENCIES
  builtin_interfaces
  std_msgs
  geometry_msgs
)

# if define msg, action. service in the same pkg
# get the rmw implementations ahead of time
find_package(rmw_implementation_cmake REQUIRED)
get_available_rmw_implementations(rmw_implementations2)
foreach(rmw_implementation ${rmw_implementations2})
  find_package("${rmw_implementation}" REQUIRED)
endforeach()

add_library(${library_name_detect_aruco_marker_action_server} SHARED
  src/detect_aruco_marker_action_server.cpp)
ament_target_dependencies(${library_name_detect_aruco_marker_action_server}
  ${detect_aruco_marker_action_server_dependencies}
)
add_executable(${executable_name_detect_aruco_marker_action_server} src/main_detect_aruco_marker_action_server.cpp)
rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
target_link_libraries(${library_name_detect_aruco_marker_action_server} "${cpp_typesupport_target}")
target_link_libraries(${executable_name_detect_aruco_marker_action_server} ${library_name_detect_aruco_marker_action_server})

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME})

install(TARGETS ${executable_name_detect_aruco_marker_action_server}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS ${library_name_detect_aruco_marker_action_server}
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
  ${library_name_detect_aruco_marker_action_server}
)

ament_export_dependencies(${detect_aruco_marker_action_server_dependencies})
ament_export_dependencies(rosidl_default_runtime)

ament_package()
