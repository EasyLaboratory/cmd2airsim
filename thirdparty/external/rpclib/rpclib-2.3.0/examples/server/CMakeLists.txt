cmake_minimum_required(VERSION 3.0.0)
project(server)

# Normally, you would want to distribute Findrpclib.cmake
# within your project directory; I wanted to avoid copies
# of this file here, so I'm just setting the CMake module path.
# In your project, you don't need the following line:
#set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/../../cmake")

find_package(rpclib REQUIRED)

include_directories(${RPCLIB_INCLUDE_DIR})

add_executable(server main.cc)
target_link_libraries(server ${RPCLIB_LIBS})
set_property(TARGET server PROPERTY CXX_STANDARD 14)

