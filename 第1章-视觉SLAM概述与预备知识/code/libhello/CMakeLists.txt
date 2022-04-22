SET(LIB_SRC hello.cpp)

ADD_DEFINITIONS("-DLIBHELLO_BUILD")

ADD_LIBRARY(libhello SHARED ${LIB_SRC})

SET(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/lib)

SET_TARGET_PROPERTIES(libhello PROPERTIES OUTPUT_NAME "sayHello")

INSTALL(TARGETS libhello LIBRARY DESTINATION /usr/local/lib)
INSTALL(FILES "hello.h" DESTINATION /usr/local/include)
                                                                                                                                                          
