cmake_minimum_required(VERSION 2.8)
project(ai_agent_planner)

set(CMAKE_VERBOSE_MAKEFILE "true")
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11  -g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -Wall")

find_package(catkin REQUIRED COMPONENTS
    roscpp
    visualization_msgs
    sensor_msgs)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

find_package(common REQUIRED)
find_package(vehicle_model REQUIRED)
find_package(vehicle_model REQUIRED)
find_package(forward_simulator REQUIRED)
find_package(semantic_map_manager REQUIRED)
find_package(behavior_planner REQUIRED)
#if this catkin packge's header is used by other packages, use catkin_package to
#declare the include directories of this package.
catkin_package(
#	INCLUDE_DIRS inc
)

include_directories(
#    inc
    ${catkin_INCLUDE_DIRS}
    ${common_INCLUDE_DIRS}
    ${semantic_map_manager_INCLUDE_DIRS}
    ${vehicle_model_INCLUDE_DIRS}
    ${forward_simulator_INCLUDE_DIRS}
    ${behavior_planner_INCLUDE_DIRS}
)

add_executable(onlane_ai_agent
    src/onlane_ai_agent.cc
)
target_link_libraries(onlane_ai_agent
    ${catkin_LIBRARIES}
    hkust_smm_ros
    behavior_planner_ros
    hkust_pl_vehicle_model
)

#install the hearder files so that other packages can include.
#install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
 #  DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_INCLUDE_DESTINATION}/
 #  FILES_MATCHING PATTERN "*.h"
 #  PATTERN ".svn" EXCLUDE
#)
