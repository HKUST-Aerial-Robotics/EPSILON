cmake_minimum_required(VERSION 2.8)
project(phy_simulator)

set(CMAKE_VERBOSE_MAKEFILE "true")
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11  -g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -Wall")

find_package(catkin REQUIRED COMPONENTS
    roscpp
    visualization_msgs
    sensor_msgs
    geometry_msgs
)

find_package(Eigen3 REQUIRED)
find_package(common REQUIRED)
find_package(vehicle_model REQUIRED)
find_package(vehicle_msgs REQUIRED)

#if this catkin packge's header is used by other packages, use catkin_package to
#declare the include directories of this package.
catkin_package(
)

include_directories(
    inc
    ${common_INCLUDE_DIRS}
    ${vehicle_model_INCLUDE_DIRS}
    ${vehicle_msgs_INCLUDE_DIRS}
    ${semantics_msgs_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
)

add_library(phy_simulator_lib
    src/phy_simulator/arena_loader.cc
    src/phy_simulator/phy_simulator.cc
)
target_link_libraries(phy_simulator_lib
    hkust_pl_vehicle_model
    hkust_pl_common
)

add_executable(phy_simulator_planning_node
    src/phy_simulator_planning_node.cc
    src/phy_simulator/visualizer.cc
    src/phy_simulator/ros_adapter.cc
)
target_link_libraries(phy_simulator_planning_node
    ${catkin_LIBRARIES}
    phy_simulator_lib
)
#install the hearder files so that other packages can include.
#install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
 #  DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_INCLUDE_DESTINATION}/
 #  FILES_MATCHING PATTERN "*.h"
 #  PATTERN ".svn" EXCLUDE
#)
