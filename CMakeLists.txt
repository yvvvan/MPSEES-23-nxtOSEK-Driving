cmake_minimum_required(VERSION 3.10)

project(geordi)

set(CMAKE_CXX_STANDARD 17)

# Set output directories
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

# Set the root directories as the location for local includes
include_directories(${CMAKE_SOURCE_DIR}/include ${CMAKE_SOURCE_DIR}/src)

# Source files, recursively
file(GLOB_RECURSE SOURCES "src/*.cpp")

# Header files, recursively
file(GLOB_RECURSE HEADERS "src/*.hpp" "include/*.hpp")

# Local libraries
include_directories(${CMAKE_SOURCE_DIR}/lib/)
file(GLOB_RECURSE LOCAL_LIBRARY_SOURCES "lib/remote_control/*.cpp")
file(GLOB_RECURSE LOCAL_LIBRARY_HEADERS "lib/remote_control/*.hpp")

# Locate the libserial and opencv libraries
find_package(OpenCV)
find_library(LIBSERIAL_LIBRARY NAMES serial)

# Add the include directory for libserial
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${LIBSERIAL_INCLUDE_DIRS})

# Add library targets
add_library(${PROJECT_NAME}_lib ${SOURCES} ${HEADERS} ${LOCAL_LIBRARY_SOURCES} ${LOCAL_LIBRARY_HEADERS})
add_library(${PROJECT_NAME}_test_lib ${SOURCES} ${HEADERS} ${LOCAL_LIBRARY_SOURCES} ${LOCAL_LIBRARY_HEADERS})

# Add compile definitions to the test library
target_compile_definitions(${PROJECT_NAME}_test_lib PUBLIC TEST_BUILD)

# Include directories for both libraries
target_include_directories(${PROJECT_NAME}_lib PUBLIC ${PROJECT_SOURCE_DIR}/include)
target_include_directories(${PROJECT_NAME}_test_lib PUBLIC ${PROJECT_SOURCE_DIR}/include ${PROJECT_SOURCE_DIR}/src)

# Link libraries
target_link_libraries(${PROJECT_NAME}_lib pthread stdc++fs bluetooth ${LIBSERIAL_LIBRARY} ${OpenCV_LIBS})
target_link_libraries(${PROJECT_NAME}_test_lib pthread stdc++fs bluetooth ${LIBSERIAL_LIBRARY} ${OpenCV_LIBS})

# Add executable targets
add_executable(${PROJECT_NAME} main.cpp)
add_executable(${PROJECT_NAME}_calibrate_colors calibrate_colors.cpp)
#add_executable(${PROJECT_NAME}_test test/main.cpp)

# Link libraries
target_link_libraries(${PROJECT_NAME} ${PROJECT_NAME}_lib)
target_link_libraries(${PROJECT_NAME}_calibrate_colors ${PROJECT_NAME}_lib)
#target_link_libraries(${PROJECT_NAME}_test ${PROJECT_NAME}_test_lib)

# Add custom "build all" target
add_custom_target(BUILD_ALL ALL DEPENDS ${PROJECT_NAME} ${PROJECT_NAME}_calibrate_colors)

## Add testing
#enable_testing()
#add_test(NAME ${PROJECT_NAME}_test COMMAND ${PROJECT_NAME}_test)