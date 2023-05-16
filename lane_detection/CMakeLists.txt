cmake_minimum_required(VERSION 3.25)
project(lane_detection)

set(CMAKE_CXX_STANDARD 17)

find_package(OpenCV)
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(lane_detection main.cpp)

target_link_libraries(lane_detection ${OpenCV_LIBS})
