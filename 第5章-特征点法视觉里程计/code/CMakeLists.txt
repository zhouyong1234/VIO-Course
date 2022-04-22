cmake_minimum_required(VERSION 3.10)
project(computeORB)


set(CMAKE_CXX_STANDARD 17)


include_directories("/usr/include/eigen3")

find_package(OpenCV REQUIRED)

# find_package(TBB REQUIRED)
find_package(Pangolin REQUIRED)

find_package (Sophus REQUIRED)

include_directories(
    ${OpenCV_INCLUDE_DIRS}
    ${Sophus_INCLUDE_DIRS}
    ${Pangolin_INCLUDE_DIRS}
)

add_executable(computeORB computeORB.cpp)
target_link_libraries(computeORB ${OpenCV_LIBRARIES} tbb)


add_executable(E2Rt E2Rt.cpp)
target_link_libraries(E2Rt ${Sophus_LIBRARIES} fmt)


add_executable(GN-BA GN-BA.cpp)
target_link_libraries(GN-BA ${Sophus_LIBRARIES} fmt)

add_executable(drawTrajectory draw_trajectory.cpp)
target_link_libraries(drawTrajectory ${Pangolin_LIBRARIES} ${Sophus_LIBRARIES} fmt)

