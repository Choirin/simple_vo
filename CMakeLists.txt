cmake_minimum_required(VERSION 2.8)

set(CMAKE_CXX_FLAGS "-O2 -std=c++11")

project(simple_vo)
find_package(OpenCV REQUIRED)
add_executable(simple_vo
    Decomposer.cpp
    Frame.cpp
    simple_vo.cpp)
target_link_libraries(simple_vo ${OpenCV_LIBS})
