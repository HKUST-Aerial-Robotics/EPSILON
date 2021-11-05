cmake_minimum_required(VERSION 2.8)
project(common)

set(CMAKE_VERBOSE_MAKEFILE "true")
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11  -g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -Wall")

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
find_package(catkin REQUIRED COMPONENTS
    roscpp
    visualization_msgs
    sensor_msgs
    tf
)

find_package(Protobuf REQUIRED)
set(THIRD_PARTY_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/thirdparty)
set(THIRD_PARTY_INCLUDE_DIRS  ${THIRD_PARTY_INCLUDE_DIRS} ${PROTOBUF_INCLUDE_DIR})
set(OOQP_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/thirdparty/ooqp/include/ooqp)
set(PROTOBUF_GENERATE_CPP_APPEND_PATH TRUE PARENT_SCOPE)

find_package(Eigen3 REQUIRED)
find_package(Glog REQUIRED)
find_package(OpenCV REQUIRED)

find_package(OOQP REQUIRED)
if(NOT OOQP_FOUND)
    message(FATAL_ERROR "OOQP not found.")
endif()

#if this catkin packge's header is used by other packages, use catkin_package to
#declare the include directories of this package.
catkin_package(
    INCLUDE_DIRS inc ${THIRD_PARTY_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR} ${OOQP_INCLUDE_DIRS}
    LIBRARIES hkust_pl_common
    CATKIN_DEPENDS roscpp visualization_msgs sensor_msgs tf
)

set(THIRD_PARTY_LIBRARIES
    dw
    ${GLOG_LIBRARIES}
    ${PROTOBUF_LIBRARY}
)

include_directories(
    inc
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${THIRD_PARTY_INCLUDE_DIRS}
    ${OOQP_INCLUDE_DIRS}
)

add_library(hkust_pl_ooqp_itf
    src/common/solver/ooqp_interface.cc
    src/common/solver/qp_solver.cc
)
target_link_libraries(hkust_pl_ooqp_itf
    ${OOQP_LIBRARIES}
)
set(COMMON_SRC
    src/common/basics/colormap.cc
    src/common/basics/semantics.cc
    src/common/basics/shapes.cc
    src/common/basics/tool_func.cc
    src/common/math/calculations.cc
    src/common/rss/rss_checker.cc
    src/common/idm/intelligent_driver_model.cc
    src/common/mobil/mobil_model.cc
    src/common/mobil/mobil_behavior_prediction.cc
    src/common/spline/lookup_table.cc
    src/common/spline/spline_generator.cc
    src/common/lane/lane.cc
    src/common/lane/lane_generator.cc
    src/common/primitive/frenet_primitive.cc
    src/common/state/state_transformer.cc
    src/common/state/free_state.cc
    src/common/circle_arc/circle_arc.cc
    src/common/circle_arc/circle_arc_branch.cc
)

add_library(hkust_pl_common
    ${COMMON_SRC}
)
target_link_libraries(hkust_pl_common
    ${THIRD_PARTY_LIBRARIES}
    opencv_core
    opencv_highgui
    hkust_pl_ooqp_itf
)

#install the hearder files so that other packages can include.
#install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
 #  DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_INCLUDE_DESTINATION}/
 #  FILES_MATCHING PATTERN "*.h"
 #  PATTERN ".svn" EXCLUDE
#)
