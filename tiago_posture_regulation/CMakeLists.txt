cmake_minimum_required(VERSION 3.0.2)
project(tiago_posture_regulation)
find_package(catkin REQUIRED COMPONENTS
  rospy
  geometry_msgs
  tf2
)
catkin_python_setup()
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES tiago_posture_regulation
  CATKIN_DEPENDS
    rospy
    geometry_msgs
    tf2
#  DEPENDS system_lib
)
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
install(PROGRAMS
  scripts/tiago_posture_regulation
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)