cmake_minimum_required(VERSION 2.8.3)
project(ar_marker_detector)

catkin_package(

)

install(PROGRAMS
   script/find_objects.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

install(FILES
  launch/skill_find_objects.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)