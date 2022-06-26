cmake_minimum_required(VERSION 3.0.2)
project(russell_fireworks_vision)

find_package(catkin REQUIRED COMPONENTS)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES russell_fireworks_vision
#  CATKIN_DEPENDS aruco_detect rospy tf
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
