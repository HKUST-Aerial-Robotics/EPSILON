cmake_minimum_required(VERSION 2.8.3)
project(aux_tools)

set( CMAKE_CXX_FLAGS "-std=c++11 -O3" )

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  sensor_msgs
  roscpp
  std_msgs
  vehicle_msgs
  tf
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES mrpt_ros_tutorial
#  CATKIN_DEPENDS mrpt_navigation roscpp std_msgs
#  DEPENDS system_lib
)
###########
## Build ##
###########
## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${vehicle_msgs_INCLUDE_DIRS}
  ${OpenCV_DIRS}
)

