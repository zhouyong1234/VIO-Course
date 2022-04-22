cmake_minimum_required(VERSION 2.8)

project(directBA)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-O3")

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

include_directories("/usr/include/eigen3")

find_package(G2O REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Pangolin REQUIRED)
find_package (Sophus REQUIRED)
Find_Package(CSparse REQUIRED)


SET(G2O_LIBS g2o_csparse_extension g2o_stuff g2o_core cxsparse g2o_types_slam3d g2o_types_sba)

include_directories(${PROJECT_SOURCE_DIR} ${EIGEN3_INCLUDE_DIR} ${CSPARSE_INCLUDE_DIR} ${Sophus_INCLUDE_DIRS})

add_library(bal_common
    bundle_adjustment.cpp 
    camera.cpp
    common.cpp
    cost_function.cpp
    mappoint.cpp 
)


add_executable(directBA directBA.cpp)
target_link_libraries(directBA 
    ${OpenCV_LIBRARIES} 
    ${Pangolin_LIBRARIES} 
    ${G2O_LIBS}
    ${Sophus_LIBRARIES} 
    fmt 
)


add_executable(bal_g2o bal_g2o.cpp)
target_link_libraries(bal_g2o
    ${G2O_LIBS}
    bal_common
    fmt
)

add_executable(bal_eigen bal_eigen.cpp)
target_link_libraries(bal_eigen
    bal_common
    fmt
)

# add_executable(bal_test bal_test.cpp)
# target_link_libraries(bal_test
#     ${OpenCV_LIBRARIES}
#     ${Sophus_LIBRARIES}
#     bal_common
#     fmt
# )

add_executable(bal_data_test bal_data_test.cpp)
target_link_libraries(bal_data_test
    ${OpenCV_LIBRARIES}
    ${Sophus_LIBRARIES}
    bal_common
    fmt
)










