cmake_minimum_required(VERSION 2.8)
project(planning_integrated)

set(CMAKE_VERBOSE_MAKEFILE "true")
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11  -g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -Wall")

find_package(catkin REQUIRED COMPONENTS
    roscpp
    message_filters
    visualization_msgs
    sensor_msgs)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

find_package(common REQUIRED)
find_package(semantic_map_manager REQUIRED)
find_package(behavior_planner REQUIRED)
find_package(eudm_planner REQUIRED)
find_package(ssc_planner REQUIRED)
#if this catkin packge's header is used by other packages, use catkin_package to
#declare the include directories of this package.
catkin_package(
#	    INCLUDE_DIRS include
)

include_directories(
    inc
    ${catkin_INCLUDE_DIRS}
    ${common_INCLUDE_DIRS}
    ${semantic_map_manager_INCLUDE_DIRS}
    ${behavior_planner_INCLUDE_DIRS}
    ${eudm_planner_INCLUDE_DIRS}
    ${ssc_planner_INCLUDE_DIRS}
)

add_executable(test_ssc_with_mpdm
    src/test_ssc_with_mpdm.cc
)
target_link_libraries(test_ssc_with_mpdm
   ${catkin_LIBRARIES}
   ssc_server_ros
   behavior_planner_ros
)

add_executable(test_ssc_with_eudm
    src/test_ssc_with_eudm.cc
)
target_link_libraries(test_ssc_with_eudm
   ${catkin_LIBRARIES}
   ssc_server_ros
   hkust_eudm_ros
)

#install the hearder files so that other packages can include.
#install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
 #  DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_INCLUDE_DESTINATION}/
 #  FILES_MATCHING PATTERN "*.h"
 #  PATTERN ".svn" EXCLUDE
#)
