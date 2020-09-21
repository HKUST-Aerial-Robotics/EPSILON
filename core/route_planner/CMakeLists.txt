cmake_minimum_required(VERSION 2.8)
project(route_planner)

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
find_package(semantic_map_manager REQUIRED)
#if this catkin packge's header is used by other packages, use catkin_package to
#declare the include directories of this package.
catkin_package(
    INCLUDE_DIRS inc
    LIBRARIES hkust_pl_rp
)

include_directories(
    inc
    ${catkin_INCLUDE_DIRS}
    ${common_INCLUDE_DIRS}
    ${semantic_map_manager_INCLUDE_DIRS}
)

add_library(hkust_pl_rp
    src/route_planner/route_planner.cc
)
target_link_libraries(hkust_pl_rp
    hkust_pl_common
    hkust_pl_smm
)

#install the hearder files so that other packages can include.
#install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
 #  DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_INCLUDE_DESTINATION}/
 #  FILES_MATCHING PATTERN "*.h"
 #  PATTERN ".svn" EXCLUDE
#)
