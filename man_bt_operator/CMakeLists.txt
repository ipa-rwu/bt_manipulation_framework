cmake_minimum_required(VERSION 3.5)
project(man_bt_operator)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

set(dependencies
  behaviortree_cpp_v3
  roscpp
  man_behavior_tree_nodes
  #std_msgs
  #geometry_msgs
  #tf2_ros
  #tf2_geometry_msgs
)

find_package(catkin REQUIRED COMPONENTS ${dependencies})


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


add_library(bt_manipulator src/bt_manipulator.cpp)
add_dependencies(bt_manipulator ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries( bt_manipulator   ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(bt_manipulator_node src/main.cpp)
add_dependencies(bt_manipulator_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(bt_manipulator_node bt_manipulator)

 install(TARGETS bt_manipulator
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
