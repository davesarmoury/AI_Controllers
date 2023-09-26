cmake_minimum_required(VERSION 3.0.2)
project(hand_vision_controller)

## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES hand_vision_controller
#  CATKIN_DEPENDS bunker_base
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
