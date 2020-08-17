cmake_minimum_required(VERSION 3.5)
project(man_bt_operator)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  man_behavior_tree
)

find_package(behaviortree_cpp_v3 REQUIRED)


set(library_name ${PROJECT_NAME})

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES ${library_name}
   CATKIN_DEPENDS roscpp std_msgs
  DEPENDS Boost
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


add_library(${library_name}
   src/bt_operator.cpp
 )

set(dependencies
  behaviortree_cpp_v3
  man_behavior_tree
)

target_link_libraries(${library_name}
  ${dependencies}
)
target_link_libraries(${library_name}
                      ${catkin_LIBRARIES}
                      ${Boost_LIBRARIES}
                      ${GSTREAMER_LIBRARIES})

 install(TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
 )

## Mark cpp header files for installation
 install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
 )

