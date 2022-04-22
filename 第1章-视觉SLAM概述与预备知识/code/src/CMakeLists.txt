# LINK_DIRECTORIES(${PROJECT_SOURCE_DIR})
# INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/libhello)

# SET(APP_SRC useHello.cpp)
# SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin)

# ADD_EXECUTABLE(sayHello ${APP_SRC})

# TARGET_LINK_LIBRARIES(sayHello libhello)

cmake_minimum_required(VERSION 3.12.2)

find_package(HELLO)

find_package(GLOG)

find_package(gflags REQUIRED)


# message(${HELLO_FOUND})

if(HELLO_FOUND)
  add_executable(sayHello useHello.cpp)
  include_directories(${HELLO_INCLUDE_DIR} ${GLOG_INCLUDE_DIRS})
  target_link_libraries(sayHello ${HELLO_LIBRARY} ${GLOG_LIBRARIES} gflags)
else(HELLO_FOUND)
  message(FATAL_ERROR "HELLO library not found")
endif(HELLO_FOUND)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin)