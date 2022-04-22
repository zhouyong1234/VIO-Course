cmake_minimum_required(VERSION 2.8)
project(image_undistort)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-O3")

include_directories("/usr/include/eigen3")

find_package(OpenCV REQUIRED)

find_package(Pangolin REQUIRED)


include_directories(
    ${OpenCV_INCLUDE_DIRS}
    ${Pangolin_INCLUDE_DIRS}
)

add_executable(image_undistort undistort_image.cpp)
target_link_libraries(image_undistort ${OpenCV_LIBRARIES})

add_executable(fisheye fisheye.cpp)
target_link_libraries(fisheye ${OpenCV_LIBRARIES})

add_executable(disparity disparity.cpp)
target_link_libraries(disparity ${OpenCV_LIBRARIES} ${Pangolin_LIBRARIES})

add_executable(gaussnewton gaussnewton.cpp)
target_link_libraries(gaussnewton ${OpenCV_LIBRARIES})


