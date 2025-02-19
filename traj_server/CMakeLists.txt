cmake_minimum_required(VERSION 3.0.2)
project(traj_server)


find_package(catkin_simple REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(mavros_msgs REQUIRED)


catkin_simple(ALL_DEPS_REQUIRED)
add_definitions(-std=c++14)
catkin_package()


include_directories(
        include
        ${Eigen3_INCLUDE_DIRS}
        ${catkin_INCLUDE_DIRS}
)


cs_add_executable(traj_server
        src/traj_server.cpp
        )

        
target_link_libraries(traj_server ${catkin_LIBRARIES})
