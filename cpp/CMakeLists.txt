cmake_minimum_required(VERSION 3.4.1)

add_compile_options(-fPIC)
add_definitions(-std=c++11)


set(CMAKE_CXX_FLAGS "-Wl,-rpath=./")

 

project(xema_sdk CXX)
find_package(Threads)
#set(APP_SRC open_cam3d.cpp)

add_library(xema_sdk SHARED xema_camera.cpp ../test/triangulation.cpp ../sdk/socket_tcp.cpp ../sdk/utils.cpp ../firmware/easylogging++.cc ../firmware/basic_function.cpp) 
target_link_libraries(xema_sdk PRIVATE
 Threads::Threads)
