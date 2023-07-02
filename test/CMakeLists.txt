include(FetchContent)

FetchContent_Declare(
        googletest
        GIT_REPOSITORY https://github.com/google/googletest.git
        GIT_TAG        release-1.12.0
)
FetchContent_MakeAvailable(googletest)

include(GoogleTest)

# necessary?
add_library(GTest::GTest INTERFACE IMPORTED)
target_link_libraries(GTest::GTest INTERFACE gtest_main)

file(GLOB_RECURSE TEST_FILES *.cpp)
add_executable(geordi_test ${TEST_FILES} helper.hpp)
add_executable(mapping_test test_mapping.cpp helper.cpp)

target_link_libraries(geordi_test
        PRIVATE
        GTest::gtest_main
        ${PROJECT_NAME}_test_lib
        )
target_link_libraries(mapping_test
        PRIVATE
        GTest::gtest_main
        ${PROJECT_NAME}_test_lib)

# add your tests here:
gtest_discover_tests(geordi_test)
gtest_discover_tests(mapping_test)
