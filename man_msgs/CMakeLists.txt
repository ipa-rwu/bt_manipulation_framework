cmake_minimum_required(VERSION 2.8.3)
project(man_msgs)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

set(MSG_DEPS
  std_msgs
  actionlib_msgs
  sensor_msgs
  geometry_msgs
  trajectory_msgs
  shape_msgs
  object_recognition_msgs
  octomap_msgs
  moveit_msgs)

find_package(catkin REQUIRED COMPONENTS message_generation ${MSG_DEPS})

#set(MSG_FILES
#)

set(SRV_FILES
UpdateArmState.srv
)

set(ACT_FILES
ComputePathSkill.action
)

add_action_files(DIRECTORY action FILES ${ACT_FILES})
#add_message_files(DIRECTORY msg FILES ${MSG_FILES})
add_service_files(DIRECTORY srv FILES ${SRV_FILES})

generate_messages(DEPENDENCIES ${MSG_DEPS})


catkin_package(CATKIN_DEPENDS message_runtime ${MSG_DEPS})

