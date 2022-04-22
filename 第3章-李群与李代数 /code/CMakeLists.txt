cmake_minimum_required(VERSION 2.8)
project(drawTrajectory)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-O3")

include_directories("/usr/include/eigen3")

find_package(Pangolin REQUIRED)

find_package (Sophus REQUIRED)

include_directories(
    ${Pangolin_INCLUDE_DIRS}
    ${Sophus_INCLUDE_DIRS}
)

add_executable(drawTrajectory draw_trajectory.cpp)
target_link_libraries(drawTrajectory ${Pangolin_LIBRARIES} ${Sophus_LIBRARIES} fmt)