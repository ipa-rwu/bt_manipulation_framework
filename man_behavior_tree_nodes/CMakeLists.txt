cmake_minimum_required(VERSION 3.5)
project(man_behavior_tree_nodes)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

set(dependencies
  behaviortree_cpp_v3
  roscpp
  man_msgs
  moveit_msgs
  moveit_ros_manipulation
  moveit_ros_move_group
  std_msgs
  geometry_msgs
  #tf2_ros
  #tf2_geometry_msgs
)

find_package(catkin REQUIRED COMPONENTS ${dependencies})


set(library_name ${PROJECT_NAME})

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES ${library_name}
   CATKIN_DEPENDS ${dependencies}
   DEPENDS system_lib Boost
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


add_library(${library_name} src/bt_engine.cpp)
add_dependencies(${library_name} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries( ${library_name}   ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_library(man_update_param_service_client_node SHARED src/bt_skill_nodes/update_param_service_client.cpp)
list(APPEND plugin_libs man_update_param_service_client_node)
add_dependencies(man_update_param_service_client_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries( man_update_param_service_client_node PRIVATE ${BEHAVIOR_TREE_LIBRARY}  ${catkin_LIBRARIES} ${Boost_LIBRARIES})
target_compile_definitions(man_update_param_service_client_node  PRIVATE BT_PLUGIN_EXPORT)

add_library(man_set_flag_task_service_client_node SHARED src/bt_skill_nodes/set_flag_task_service_client.cpp)
list(APPEND plugin_libs man_set_flag_task_service_client_node)
add_dependencies(man_set_flag_task_service_client_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries( man_set_flag_task_service_client_node PRIVATE ${BEHAVIOR_TREE_LIBRARY}  ${catkin_LIBRARIES} ${Boost_LIBRARIES})
target_compile_definitions(man_set_flag_task_service_client_node  PRIVATE BT_PLUGIN_EXPORT)

add_library(man_set_flag_task_condition_node SHARED src/bt_skill_nodes/condition/set_flag_task_condition.cpp)
list(APPEND plugin_libs man_set_flag_task_condition_node)
add_dependencies(man_set_flag_task_condition_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries( man_set_flag_task_condition_node PRIVATE ${BEHAVIOR_TREE_LIBRARY}  ${catkin_LIBRARIES} ${Boost_LIBRARIES})
target_compile_definitions(man_set_flag_task_condition_node  PRIVATE BT_PLUGIN_EXPORT)

add_library(man_check_task_result_condition_node SHARED src/bt_skill_nodes/condition/check_task_result.cpp)
list(APPEND plugin_libs man_check_task_result_condition_node)
add_dependencies(man_check_task_result_condition_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries( man_check_task_result_condition_node PRIVATE ${BEHAVIOR_TREE_LIBRARY}  ${catkin_LIBRARIES} ${Boost_LIBRARIES})
target_compile_definitions(man_check_task_result_condition_node  PRIVATE BT_PLUGIN_EXPORT)

add_library(man_find_objects_action_client_node SHARED src/bt_skill_nodes/find_objects_action_client.cpp)
list(APPEND plugin_libs man_find_objects_action_client_node)
add_dependencies(man_find_objects_action_client_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries( man_find_objects_action_client_node PRIVATE ${BEHAVIOR_TREE_LIBRARY}  ${catkin_LIBRARIES} ${Boost_LIBRARIES})
target_compile_definitions(man_find_objects_action_client_node  PRIVATE BT_PLUGIN_EXPORT)

add_library(man_update_goal_for_arm_service_client_node SHARED src/bt_skill_nodes/update_arm_goal_service_client.cpp)
list(APPEND plugin_libs man_update_goal_for_arm_service_client_node)
add_dependencies(man_update_goal_for_arm_service_client_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries( man_update_goal_for_arm_service_client_node PRIVATE ${BEHAVIOR_TREE_LIBRARY}  ${catkin_LIBRARIES} ${Boost_LIBRARIES})
target_compile_definitions(man_update_goal_for_arm_service_client_node  PRIVATE BT_PLUGIN_EXPORT)

add_library(man_compute_path_action_client_node SHARED src/bt_skill_nodes/compute_path_action_client.cpp)
list(APPEND plugin_libs man_compute_path_action_client_node)
add_dependencies(man_compute_path_action_client_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries( man_compute_path_action_client_node PRIVATE ${BEHAVIOR_TREE_LIBRARY}  ${catkin_LIBRARIES} ${Boost_LIBRARIES})
target_compile_definitions(man_compute_path_action_client_node  PRIVATE BT_PLUGIN_EXPORT)

add_library(man_execute_trajectory_action_client_node SHARED src/bt_skill_nodes/execute_trajectory_action_client.cpp)
list(APPEND plugin_libs man_execute_trajectory_action_client_node)
add_dependencies(man_execute_trajectory_action_client_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries( man_execute_trajectory_action_client_node PRIVATE ${BEHAVIOR_TREE_LIBRARY}  ${catkin_LIBRARIES} ${Boost_LIBRARIES})
target_compile_definitions(man_execute_trajectory_action_client_node  PRIVATE BT_PLUGIN_EXPORT)


add_library(man_execute_gripper_trajectory_action_client_node SHARED src/bt_skill_nodes/execute_gripper_trajectory_action_client.cpp)
list(APPEND plugin_libs man_execute_gripper_trajectory_action_client_node)
add_dependencies(man_execute_gripper_trajectory_action_client_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries( man_execute_gripper_trajectory_action_client_node PRIVATE ${BEHAVIOR_TREE_LIBRARY}  ${catkin_LIBRARIES} ${Boost_LIBRARIES})
target_compile_definitions(man_execute_gripper_trajectory_action_client_node  PRIVATE BT_PLUGIN_EXPORT)

 install(TARGETS ${library_name} ${plugin_libs}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
 )

## Mark cpp header files for installation
 install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
 )
