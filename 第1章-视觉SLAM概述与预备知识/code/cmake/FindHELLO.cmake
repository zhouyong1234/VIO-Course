find_path(HELLO_INCLUDE_DIR hello.h /usr/local/include/hello.h)
set(HELLO_LIBRARY /usr/local/lib/libsayHello.so)

# message(${HELLO_INCLUDE_DIR})
# message(${HELLO_LIBRARY})

if(HELLO_INCLUDE_DIR AND HELLO_LIBRARY)
  SET(HELLO_FOUND true)
else()
endif(HELLO_INCLUDE_DIR AND HELLO_LIBRARY)

if(HELLO_FOUND)
  if(NOT HELLO_FIND_QUIETLY)
    message(STATUS "Found Hello: ${HELLO_INCLUDE_DIR} ${HELLO_LIBRARY}")
  endif(NOT HELLO_FIND_QUIETLY)
else(HELLO_FOUND)
  if(HELLO_FIND_REQUIRED)
    message(FATAL_ERROR "Could not find hello library")
  endif(HELLO_FIND_REQUIRED)
endif(HELLO_FOUND)

 