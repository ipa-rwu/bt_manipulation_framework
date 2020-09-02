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
  src/skills/service_server/update_arm_state_skill/update_arm_state.cpp)
add_executable(update_arm_state_skill_server 
  src/skills/service_server/update_arm_state_skill/main.cpp)
target_link_libraries(update_arm_state_skill ${catkin_LIBRARIES} ${Boost_LIBRARIES})
target_link_libraries(update_arm_state_skill_server update_arm_state_skill)


add_library(update_arm_goal_skill SHARED
  src/skills/service_server/update_arm_goal_skill/update_arm_goal.cpp)
add_executable(update_arm_goal_skill_server 
  src/skills/service_server/update_arm_goal_skill/main.cpp)
target_link_libraries(update_arm_goal_skill ${catkin_LIBRARIES} ${Boost_LIBRARIES})
target_link_libraries(update_arm_goal_skill_server update_arm_goal_skill)
add_dependencies(update_arm_goal_skill ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(update_arm_goal_skill_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_library(execute_trajectory_skill SHARED
  src/skills/action_servers/execute_trajectory_skill/execute_trajectory_skill.cpp)
add_executable(execute_trajectory_skill_server 
  src/skills/action_servers/execute_trajectory_skill/main.cpp)
target_link_libraries(execute_trajectory_skill ${catkin_LIBRARIES} ${Boost_LIBRARIES})
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




add_executable(test_compute_path_skill_node 
  src/skills/test_compute_path.cpp)
target_link_libraries(test_compute_path_skill_node compute_path_skill)
add_dependencies(test_compute_path_skill_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})



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
                      scripts/service_server/help_extern_server/help
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})


## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_manipulator_skills.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
