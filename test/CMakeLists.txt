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

file(GLOB_RECURSE CI_TEST_FILES ci_tests/*.cpp)
add_executable(ci_test ${CI_TEST_FILES})

target_link_libraries(ci_test
  PRIVATE
  GTest::gtest_main
  ${PROJECT_NAME}_test_lib
)

add_executable(controller_test manual_tests/controller_test.cpp)
target_link_libraries(controller_test
  PRIVATE
  GTest::gtest_main
  ${PROJECT_NAME}_test_lib
)

add_executable(drive_test manual_tests/drive_test.cpp)
target_link_libraries(drive_test
    PRIVATE
    GTest::gtest_main
    ${PROJECT_NAME}_test_lib
)

# add your tests here:
gtest_discover_tests(ci_test)
