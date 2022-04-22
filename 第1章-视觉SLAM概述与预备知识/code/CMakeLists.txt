cmake_minimum_required(VERSION 3.12.2)
PROJECT(hello)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

# include(cmake/FindHELLO.cmake)

add_subdirectory(src)


enable_testing()
add_subdirectory(third_party/googletest-release-1.11.0)
include_directories(third_party/googletest-release-1.11.0/googletest/include)
set(GTEST_LIB gtest gtest_main)


add_subdirectory(tests)


# ADD_SUBDIRECTORY(libhello)
# ADD_SUBDIRECTORY(src)

# SET(CMAKE_BUILD_TYPE "Release")

#INSTALL(TARGETS libhello LIBRARY DESTINATION /usr/local/lib)
#INSTALL(./libhello/hello.h /usr/local/include)                                                                                                                                                     
