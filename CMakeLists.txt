cmake_minimum_required(VERSION 3.10)

project(Driving2)

set(CMAKE_CXX_STANDARD 17)

# Set output directories
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

# Locate the libserial library
find_library(LIBSERIAL_LIBRARY NAMES serial)

# Add the include directory for libserial
include_directories(${LIBSERIAL_INCLUDE_DIRS})

# Set the root directories as the location for local includes
include_directories(${CMAKE_SOURCE_DIR}/include ${CMAKE_SOURCE_DIR}/src)

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

add_executable(orb_slam_raspi_test src/mapping.cpp)
