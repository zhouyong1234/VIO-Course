cmake_minimum_required(VERSION 3.10)
project(flow)


set(CMAKE_CXX_STANDARD 17)


include_directories("/usr/include/eigen3")

find_package(OpenCV REQUIRED)

find_package(Pangolin REQUIRED)

find_package (Sophus REQUIRED)


include_directories(
    ${OpenCV_INCLUDE_DIRS}
    ${Sophus_INCLUDE_DIRS}
    ${Pangolin_INCLUDE_DIRS}
)

add_executable(flow optical_flow.cpp)
target_link_libraries(flow ${OpenCV_LIBRARIES})

add_executable(direct direct_method.cpp)
target_link_libraries(direct ${OpenCV_LIBRARIES} ${Pangolin_LIBRARIES} ${Sophus_LIBRARIES} fmt)




