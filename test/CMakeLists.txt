include(FetchContent)

FetchContent_Declare(
  googletest
  GIT_REPOSITORY https://github.com/google/googletest.git
  GIT_TAG        release-1.13.0
)
FetchContent_MakeAvailable(googletest)

# necessary?
add_library(GTest::GTest INTERFACE IMPORTED)
target_link_libraries(GTest::GTest INTERFACE gtest_main)

add_executable(localization_test localization_test.cpp)

target_link_libraries(localization_test
 PRIVATE
  GTest::gtest_main
  ${PROJECT_NAME}_test_lib)

gtest_discover_tests(localization_test)
