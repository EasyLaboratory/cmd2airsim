cmake_minimum_required(VERSION 3.10.0)
project(easysim_ros_wrapper)


set(CMAKE_CXX_STANDARD 17)              # 设置 C++ 标准为 C++17
set(CMAKE_CXX_STANDARD_REQUIRED ON)    # 强制要求使用 C++17


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


# todo eigen3 in AirLib already
find_package(Eigen3 REQUIRED) 
find_package(Boost REQUIRED)
find_package(OpenCV REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  roscpp
  rospy
  std_msgs
  geometry_msgs
  tf
  cv_bridge
  image_transport
  sensor_msgs
)


catkin_package(
  INCLUDE_DIRS include
  # LIBRARIES airsim_ros
  CATKIN_DEPENDS  roscpp std_msgs 
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${AIRSIM_ROOT}/AirLib/deps/eigen3
  ${AIRSIM_ROOT}/AirLib/include
  ${RPC_LIB_INCLUDES}
  ${AIRSIM_ROOT}/MavLinkCom/include
  ${AIRSIM_ROOT}/MavLinkCom/common_utils
  ${Boost_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_library(easysim_ros_wrapper 
src/easysim_ros_wrapper.cpp)

target_link_libraries(easysim_ros_wrapper 
${catkin_LIBRARIES} 
AirLib)



add_executable(easysim_ros_node src/easysim_ros_node.cpp )

target_link_libraries(easysim_ros_node
  ${catkin_LIBRARIES}
  AirLib
  easysim_ros_wrapper
)


add_library(image_udp_receiver src/image_udp_receiver.cpp)
target_link_libraries(image_udp_receiver
  ${catkin_LIBRARIES} 
  ${Boost_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

add_executable(image_ros_wrapper src/image_ros_wrapper.cpp)
target_link_libraries(image_ros_wrapper
  image_udp_receiver
)

