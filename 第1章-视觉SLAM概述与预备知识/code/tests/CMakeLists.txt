cmake_minimum_required(VERSION 3.12.2)


# find_package(gtest REQUIRED)

add_executable(demo_test demo.cpp)
target_link_libraries(demo_test gtest gtest_main)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin)