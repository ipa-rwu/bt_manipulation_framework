cmake_minimum_required(VERSION 3.0)
project(manipulator_skills)

add_compile_options(-std=c++14)
set(CMAKE_CXX_STANDARD 14)

set(dependencies
  rospy
  roscpp
  geometry_msgs
  std_msgs
  moveit_msgs
  moveit_ros_manipulation
  moveit_ros_move_group
  moveit_ros_planning_interface
  actionlib
  man_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
  ar_marker_detector
  webots_ros
)

find_package(catkin REQUIRED COMPONENTS ${dependencies}
)

find_package(Boost REQUIRED COMPONENTS system filesystem)

find_package(Eigen3 REQUIRED)
add_definitions(${EIGEN3_DEFINITIONS})

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES ${library_name}
   CATKIN_DEPENDS ${dependencies}
   DEPENDS system_lib
)

catkin_python_setup()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)


#set(base_library_name ${PROJECT_NAME}_all)

#add_library(${base_library_name} SHARED
#  src/skill.cpp
#)

#target_link_libraries(${base_library_name} ${catkin_LIBRARIES} ${Boost_LIBRARIES})


#set(skills_library_name ${PROJECT_NAME}_subskills)

#### add_library
add_library(compute_path_skill SHARED
  src/skills/action_servers/compute_path_skill/compute_path_skill.cpp)
#### add_executable
add_executable(compute_path_skill_server 
  src/skills/action_servers/compute_path_skill/main.cpp)
#### target_link_libraries
target_link_libraries(compute_path_skill ${catkin_LIBRARIES} ${Boost_LIBRARIES})
target_link_libraries(compute_path_skill_server compute_path_skill)
#### add_dependencies
add_dependencies(compute_path_skill ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(compute_path_skill_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_library(find_objects_skill SHARED
  src/skills/action_servers/find_objects_skill/find_objects.cpp)
add_executable(find_objects_skill_server 
  src/skills/action_servers/find_objects_skill/main.cpp)
target_link_libraries(find_objects_skill ${catkin_LIBRARIES} ${Boost_LIBRARIES})
target_link_libraries(find_objects_skill_server find_objects_skill)
add_dependencies(find_objects_skill ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(find_objects_skill_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


add_library(update_arm_state_skill SHARED
  src/skills/service_servers/update_arm_state_skill/update_arm_state.cpp)
add_executable(update_arm_state_skill_server 
  src/skills/service_servers/update_arm_state_skill/main.cpp)
target_link_libraries(update_arm_state_skill ${catkin_LIBRARIES} ${Boost_LIBRARIES})
target_link_libraries(update_arm_state_skill_server update_arm_state_skill)
add_dependencies(update_arm_state_skill ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(update_arm_state_skill_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_library(update_arm_goal_skill SHARED
  src/skills/service_servers/update_arm_goal_skill/update_arm_goal.cpp)
add_executable(update_arm_goal_skill_server 
  src/skills/service_servers/update_arm_goal_skill/main.cpp)
target_link_libraries(update_arm_goal_skill ${catkin_LIBRARIES} ${Boost_LIBRARIES})
target_link_libraries(update_arm_goal_skill_server update_arm_goal_skill)
add_dependencies(update_arm_goal_skill ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(update_arm_goal_skill_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_library(update_param_skill SHARED
  src/skills/service_servers/update_param_skill/update_param.cpp)
add_executable(update_param_skill_server 
  src/skills/service_servers/update_param_skill/main.cpp)
target_link_libraries(update_param_skill ${catkin_LIBRARIES} ${Boost_LIBRARIES})
target_link_libraries(update_param_skill_server update_param_skill)
add_dependencies(update_param_skill ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(update_param_skill_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_library(check_collision_skill SHARED
  src/skills/service_servers/check_collision_skill/check_collision.cpp)
add_executable(check_collision_skill_server 
  src/skills/service_servers/check_collision_skill/main.cpp)
target_link_libraries(check_collision_skill ${catkin_LIBRARIES} ${Boost_LIBRARIES})
target_link_libraries(check_collision_skill_server check_collision_skill)
add_dependencies(check_collision_skill ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(check_collision_skill_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_library(set_parameter SHARED
  src/skills/service_servers/set_parameter/set_parameter.cpp)
add_executable(set_parameter_server 
  src/skills/service_servers/set_parameter/main.cpp)
target_link_libraries(set_parameter ${catkin_LIBRARIES} ${Boost_LIBRARIES})
target_link_libraries(set_parameter_server set_parameter)
add_dependencies(set_parameter ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(set_parameter_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_library(execute_gripper_io_skill SHARED
  src/skills/action_servers/execute_gripper_io_skill/execute_gripper_io.cpp)
add_executable(execute_gripper_io_skill_server 
  src/skills/action_servers/execute_gripper_io_skill/main.cpp)
target_link_libraries(execute_gripper_io_skill ${catkin_LIBRARIES} ${Boost_LIBRARIES})
target_link_libraries(execute_gripper_io_skill_server execute_gripper_io_skill)
add_dependencies(execute_gripper_io_skill ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(execute_gripper_io_skill_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_library(execute_trajectory_skill SHARED
  src/skills/action_servers/execute_trajectory_skill/execute_trajectory_skill.cpp)
add_executable(execute_trajectory_skill_server 
  src/skills/action_servers/execute_trajectory_skill/main.cpp)
target_link_libraries(execute_trajectory_skill ${catkin_LIBRARIES} ${Boost_LIBRARIES})

target_link_libraries(execute_trajectory_skill webots_elements)

target_link_libraries(execute_trajectory_skill_server execute_trajectory_skill)
add_dependencies(execute_trajectory_skill ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(execute_trajectory_skill_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_library(execute_gripper_trajectory_skill SHARED
  src/skills/action_servers/execute_gripper_trajectory/execute_gripper_trajectory.cpp)
add_executable(execute_gripper_trajectory_skill_server 
  src/skills/action_servers/execute_gripper_trajectory/main.cpp)
target_link_libraries(execute_gripper_trajectory_skill ${catkin_LIBRARIES} ${Boost_LIBRARIES})
target_link_libraries(execute_gripper_trajectory_skill_server execute_gripper_trajectory_skill)
add_dependencies(execute_gripper_trajectory_skill ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(execute_gripper_trajectory_skill_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_library(webots_elements SHARED
  src/webots_elements.cpp)
target_link_libraries(webots_elements ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(webots_elements ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

install(TARGETS compute_path_skill
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
   PATTERN ".svn" EXCLUDE
 )

#############
## Install ##
#############

catkin_install_python(PROGRAMS 
                      scripts/service_servers/help_extern_server/help
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})