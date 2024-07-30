cmake_minimum_required(VERSION 3.0.2)
project(traj_server)


set(AIRSIM_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../../thirdparty)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
# add_subdirectory("/usr/AirSim/cmake/AirLib" AirLib)
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages


# add_subdirectory("${AIRSIM_ROOT}/cmake/rpclib_wrapper" rpclib_wrapper)
# add_subdirectory("${AIRSIM_ROOT}/cmake/AirLib" AirLib)
# add_subdirectory("${AIRSIM_ROOT}/cmake/MavLinkCom" MavLinkCom)

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


find_package(catkin_simple REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_simple(ALL_DEPS_REQUIRED)
add_definitions(-std=c++14)
catkin_package()


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


cs_add_executable(traj_server
        src/traj_server.cpp
        )

cs_add_executable(example
        src/example.cpp
        )

cs_add_executable(cmd2airsim
        src/cmd2airsim.cpp
        )

cs_add_executable(frame_transform
        src/frame_transform.cpp
        )

        
target_link_libraries(example ${catkin_LIBRARIES} AirLib)
target_link_libraries(cmd2airsim ${catkin_LIBRARIES} AirLib)
target_link_libraries(traj_server ${catkin_LIBRARIES})
target_link_libraries(frame_transform ${catkin_LIBRARIES})
