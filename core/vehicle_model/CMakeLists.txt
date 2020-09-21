cmake_minimum_required(VERSION 2.8)
project(vehicle_model)

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
find_package(common)

set(THIRD_PARTY_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/thirdparty)
#if this catkin packge's header is used by other packages, use catkin_package to
#declare the include directories of this package.
catkin_package(
    INCLUDE_DIRS  inc
    LIBRARIES hkust_pl_vehicle_model
)

include_directories(
    inc
    ${common_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${THIRD_PARTY_INCLUDE_DIRS}
)

add_library(hkust_pl_vehicle_model
    src/vehicle_model/vehicle_model.cc
    src/vehicle_model/ideal_steer_model.cc
    src/vehicle_model/idm_model.cc
    src/vehicle_model/ctx_idm_model.cc
    src/vehicle_model/controllers/pure_pursuit_controller.cc
    src/vehicle_model/controllers/pid_controller.cc
    src/vehicle_model/controllers/idm_velocity_controller.cc
    src/vehicle_model/controllers/ctx_idm_velocity_controller.cc
)
target_link_libraries(hkust_pl_vehicle_model
    hkust_pl_common
)

#install the hearder files so that other packages can include.
#install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
 #  DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_INCLUDE_DESTINATION}/
 #  FILES_MATCHING PATTERN "*.h"
 #  PATTERN ".svn" EXCLUDE
#)
