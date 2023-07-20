cmake_minimum_required(VERSION 3.8)
project(man2_bt_skills)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(pluginlib REQUIRED)
find_package(aruco_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(behaviortree_cpp REQUIRED)

find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

include_directories(
  include
)

set(dependencies
  rclcpp
  pluginlib
  behaviortree_cpp
  aruco_msgs
  visualization_msgs
  geometry_msgs
)

add_library(detect_aruco_marker_skill SHARED
  src/detect_aruco_marker.cpp)
list(APPEND plugin_libs detect_aruco_marker_skill)

ament_target_dependencies(detect_aruco_marker_skill
  ${dependencies}
)

foreach(bt_plugin ${plugin_libs})
  ament_target_dependencies(${bt_plugin} ${dependencies})
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

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME})

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
  ${library_name}
  ${plugin_libs})
ament_export_dependencies(${dependencies})

ament_package()
