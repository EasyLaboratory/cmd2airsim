cmake_minimum_required(VERSION 3.0.2)
project(se3controller)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  std_msgs
  traj_msgs
)


set(AIRSIM_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../thirdparty)

if(NOT TARGET AirLib)
    add_subdirectory("${AIRSIM_ROOT}/cmake/AirLib" AirLib)
endif()

if(NOT TARGET rpc)
    add_subdirectory("${AIRSIM_ROOT}/cmake/rpclib_wrapper" rpclib_wrapper)
endif()

if(NOT TARGET MavLinkCom)
    add_subdirectory("${AIRSIM_ROOT}/cmake/MavLinkCom" MavLinkCom)
endif()


set(RPC_LIB_INCLUDES " ${AIRSIM_ROOT}/external/rpclib/rpclib-2.3.0/include")
set(RPC_LIB rpc) # name of .a file with lib prefix



set(CMAKE_CXX_STANDARD 17)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES se3controller
#  CATKIN_DEPENDS geometry_msgs roscpp std_msgs
#  DEPENDS system_lib

)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include 
  ${catkin_INCLUDE_DIRS} 
)

include_directories(
        include
        ${Eigen3_INCLUDE_DIRS}
        ${catkin_INCLUDE_DIRS}
        ${AIRSIM_ROOT}/AirLib/deps/eigen3
        ${AIRSIM_ROOT}/AirLib/include
        ${RPC_LIB_INCLUDES}
        ${AIRSIM_ROOT}/MavLinkCom/include
        ${AIRSIM_ROOT}/MavLinkCom/common_utils
        ${OpenCV_INCLUDE_DIRS}
)


include_directories(
  include
  "/usr/include/eigen3"
)

## Declare a C++ library
add_library(se3control
        src/se3controller/se3control.cpp
        src/se3controller/jerk_tracking_control.cpp
        src/se3controller/nonlinear_attitude_control.cpp
        src/se3controller/nonlinear_geometric_control.cpp
        )


add_executable( se3controller_ros_node 
src/se3controller_ros_node/se3controller_ros_node.cpp )
target_link_libraries( se3controller_ros_node ${catkin_LIBRARIES} se3control AirLib)


add_executable(odom_msg2pose_msg  src/odom_msg2pose_msg/odom_msg2pose_msg.cpp)
target_link_libraries(odom_msg2pose_msg ${catkin_LIBRARIES})



add_executable( flying_example src/example/flying_example.cpp  )
target_link_libraries( flying_example ${catkin_LIBRARIES} )

add_executable( flying_traj_example src/example/flying_traj_example.cpp  )
target_link_libraries( flying_traj_example ${catkin_LIBRARIES} )