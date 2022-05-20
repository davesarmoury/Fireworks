cmake_minimum_required(VERSION 3.0.2)
project(russell_fireworks_bringup)

find_package(catkin REQUIRED COMPONENTS)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES russell_fireworks_bringup
#  CATKIN_DEPENDS controller_manager kuka_rsi_hw_interface robot_state_publisher
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
