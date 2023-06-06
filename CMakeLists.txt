cmake_minimum_required(VERSION 3.10)

project(Driving2)

set(CMAKE_CXX_STANDARD 17)
include(GNUInstallDirs)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set output directories
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

# Locate the libserial library
find_library(LIBSERIAL_LIBRARY NAMES serial)

find_package(Eigen3 3.1.0 REQUIRED)
find_package(Pangolin REQUIRED)

find_package(OpenCV 4.4)
   if(NOT OpenCV_FOUND)
      message(FATAL_ERROR "OpenCV > 4.4 not found.")
   endif()

set(ORB_SLAM_DIR ${CMAKE_SOURCE_DIR}/libs/ORB_SLAM3)
include_directories(${ORB_SLAM_DIR}
    ${ORB_SLAM_DIR}/include
    ${ORB_SLAM_DIR}/include/CameraModels
    ${ORB_SLAM_DIR}/Thirdparty/Sophus
    ${EIGEN3_INCLUDE_DIR}
    ${Pangolin_INCLUDE_DIRS}
    )

# Add the include directory for libserial
include_directories(${LIBSERIAL_INCLUDE_DIRS})

# Set the root directories as the location for local includes
include_directories(${CMAKE_SOURCE_DIR}/include ${CMAKE_SOURCE_DIR}/src /usr/include/libcamera)

include_directories(/usr/local/include/)

# Source files, recursively
file(GLOB_RECURSE SOURCES "src/*.cpp")

# Header files, recursively
file(GLOB_RECURSE HEADERS "src/*.hpp" "include/*.hpp")

# Remote control library
include_directories(${CMAKE_SOURCE_DIR}/lib/remote_control)
file(GLOB_RECURSE REMOTE_CONTROL_SOURCES "lib/remote_control/*.cpp")
file(GLOB_RECURSE REMOTE_CONTROL_HEADERS "lib/remote_control/*.hpp")

# Add executable target
add_executable(${PROJECT_NAME} ${SOURCES} ${HEADERS} ${REMOTE_CONTROL_SOURCES} ${REMOTE_CONTROL_HEADERS})

# Include directories
target_include_directories(${PROJECT_NAME} PUBLIC ${PROJECT_SOURCE_DIR}/include)

# Link libraries
target_link_libraries(${PROJECT_NAME} pthread stdc++fs bluetooth ${LIBSERIAL_LIBRARY})

# Add mapping executable
add_executable(orb_slam_raspi_test src/mapping.cpp)
target_link_libraries(orb_slam_raspi_test
    ${OpenCV_LIBS}
    ${EIGEN3_LIBS}
    ${Pangolin_LIBRARIES}
    ${ORB_SLAM_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
    ${ORB_SLAM_DIR}/Thirdparty/g2o/lib/libg2o.so
    ${ORB_SLAM_DIR}/lib/libORB_SLAM3.so
    )
