#-cmake_minimum_required(VERSION 2.4.6)
#-include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

#-rosbuild_init()

#set the default path for built executables to the "bin" directory
#-set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
#-set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#-\rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

#-rosbuild_add_library(encode_msgs src/encode_msgs.cpp)
#-rosbuild_add_link_flags(encode_msgs -Wl,--as-needed)

#list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
#find_package(Eigen3 REQUIRED)

#include_directories(${EIGEN3_INCLUDE_DIR})
#-rosbuild_add_library(decode_msgs src/decode_msgs.cpp)
#-rosbuild_add_link_flags(decode_msgs -Wl,--as-needed)

#------------------------------------------------------------------
cmake_minimum_required(VERSION 2.8.3)
project(vehicle_msgs)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  nav_msgs
  geometry_msgs
  message_generation
)

find_package(common)

include_directories(
    ${common_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
)

add_message_files(
  FILES
  State.msg
  ControlSignal.msg
  Lane.msg
  LaneNet.msg
  VehicleParam.msg
  Vehicle.msg
  VehicleSet.msg
  OccupancyGridFloat.msg
  OccupancyGridUInt8.msg
  Circle.msg
  CircleObstacle.msg
  PolygonObstacle.msg
  ObstacleSet.msg
  FreeState.msg

  ArenaInfo.msg
  ArenaInfoDynamic.msg
  ArenaInfoStatic.msg
)

generate_messages(
    DEPENDENCIES
    geometry_msgs
    nav_msgs
    std_msgs
)

catkin_package(
  INCLUDE_DIRS inc ${common_INCLUDE_DIRS}
  LIBRARIES hkust_pl_common
 # LIBRARIES encode_msgs decode_msgs
  #CATKIN_DEPENDS geometry_msgs nav_msgs
  #DEPENDS system_lib
  CATKIN_DEPENDS message_runtime
)

# add_executable(test_exe src/test_exe.cpp)
#add_library(decode_msgs src/decode_msgs.cpp)
#add_library(encode_msgs src/encode_msgs.cpp)

# add_dependencies(test_exe quadrotor_msgs_generate_messages_cpp)
#add_dependencies(encode_msgs quadrotor_msgs_generate_messages_cpp)
#add_dependencies(decode_msgs quadrotor_msgs_generate_messages_cpp)

#find_package(Eigen3 REQUIRED)
#include_directories(${EIGEN3_INCLUDE_DIR})


# target_link_libraries(test_exe
#     decode_msgs
#     encode_msgs
# )
