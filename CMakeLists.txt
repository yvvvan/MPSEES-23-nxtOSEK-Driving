cmake_minimum_required(VERSION 3.10)

project(geordi)

set(CMAKE_CXX_STANDARD 17)
include(GNUInstallDirs)

if(NOT CMAKE_BUILD_TYPE)
<<<<<<< HEAD
  set(CMAKE_BUILD_TYPE "Debug")
=======
   set(CMAKE_BUILD_TYPE "Debug")
>>>>>>> 2ecef7725ec08a3e31b2f30f9a8d0a999447a8a7
endif()

# Set output directories
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

# Locate the libserial library
find_library(LIBSERIAL_LIBRARY NAMES serial)

find_package(OpenCV 4.4)
   if(NOT OpenCV_FOUND)
      message(FATAL_ERROR "OpenCV > 4.4 not found.")
   endif()

# Add the include directory for libserial
include_directories(${LIBSERIAL_INCLUDE_DIRS})

# Set the root directories as the location for local includes
include_directories(${CMAKE_SOURCE_DIR}/include ${CMAKE_SOURCE_DIR}/src)

include_directories(/usr/local/include/)

# Remote control library
include_directories(${CMAKE_SOURCE_DIR}/lib/remote_control)
file(GLOB_RECURSE REMOTE_CONTROL_SOURCES "lib/remote_control/*.cpp")
file(GLOB_RECURSE REMOTE_CONTROL_HEADERS "lib/remote_control/*.hpp")

include_directories(/usr/local/include/)

# Source files, recursively
file(GLOB_RECURSE SOURCES "src/*.cpp")

# Header files, recursively
file(GLOB_RECURSE HEADERS "src/*.hpp" "include/*.hpp")

# ------------------------------------------------------- #
# ------------------- Local libraries ------------------- #

# Add library targets
add_library(${PROJECT_NAME}_lib ${SOURCES} ${HEADERS} ${REMOTE_CONTROL_SOURCES} ${REMOTE_CONTROL_HEADERS})
add_library(${PROJECT_NAME}_test_lib ${SOURCES} ${HEADERS} ${REMOTE_CONTROL_SOURCES} ${REMOTE_CONTROL_HEADERS})

# Add compile definitions to the test library
target_compile_definitions(${PROJECT_NAME}_test_lib PUBLIC TEST_BUILD)

# Include directories for both libraries
target_include_directories(${PROJECT_NAME}_lib PUBLIC ${PROJECT_SOURCE_DIR}/include)
target_include_directories(${PROJECT_NAME}_test_lib PUBLIC ${PROJECT_SOURCE_DIR}/include ${PROJECT_SOURCE_DIR}/src)

# Link libraries
target_link_libraries(${PROJECT_NAME}_lib 
   pthread
   stdc++fs
   bluetooth
   ${LIBSERIAL_LIBRARY}
   ${OpenCV_LIBS}
   )

target_link_libraries(${PROJECT_NAME}_test_lib
   pthread
   stdc++fs
   bluetooth
   ${LIBSERIAL_LIBRARY}
   ${OpenCV_LIBS}
   )
# ------------------------------------------------------- #

## Add executable targets
add_executable(${PROJECT_NAME} main.cpp)

# Link libraries
target_link_libraries(${PROJECT_NAME} ${PROJECT_NAME}_lib)

add_subdirectory(test)
