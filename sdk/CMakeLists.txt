cmake_minimum_required(VERSION 3.4.1)

add_compile_options(-fPIC)
add_definitions(-std=c++11)

set(CONFIGURING_IP_DIR ../3rdparty/ConfiguringIP/)
set(CMAKE_CXX_FLAGS "-Wl,-rpath=./")


include_directories(../3rdparty/ConfiguringIP/)
 

project(open_cam3d_sdk CXX)
find_package(Threads)
#set(APP_SRC open_cam3d.cpp)

add_library(open_cam3d_sdk SHARED open_cam3d.cpp ../test/triangulation.cpp socket_tcp.cpp utils.cpp ../firmware/easylogging++.cc ../firmware/basic_function.cpp) 
target_link_libraries(open_cam3d_sdk PRIVATE
 Threads::Threads
 ${PROJECT_SOURCE_DIR}/../3rdparty/ConfiguringIP/libconfiguring_network_static.a)
