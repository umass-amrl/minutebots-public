cmake_minimum_required(VERSION 2.8.11)

PROJECT(srtr)

# Flag for toggling test compilation on or off.
OPTION(test "Build all tests." ON)

SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
SET(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
SET(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/third_party/")

INCLUDE(FindProtobuf)

FIND_PACKAGE(Protobuf REQUIRED)

# INCLUDE (FindZ3.cmake)
# INCLUDE(FindRE2.cmake)
FIND_PACKAGE(Z3 REQUIRED)

#FIND_PACKAGE(Ceres REQUIRED PATHS "${CMAKE_MODULE_PATH}ceres-solver-1.12.0")

SET(CMAKE_CXX_FLAGS "-std=c++11 -Wall -Wsign-compare -Werror -g")
SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -rdynamic")
SET(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -rdynamic")

IF(${CMAKE_BUILD_TYPE} MATCHES "Release")
  MESSAGE(STATUS "Additional Flags for Release mode")
  SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -openmp -fno-builtin-malloc")
  SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -fno-builtin-calloc")
  SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -fno-builtin-realloc")
  SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -fno-builtin-free -O3 -DNDEBUG")
  #SET(BUILD_SPECIFIC_LIBRARIES "tcmalloc")
ELSEIF(${CMAKE_BUILD_TYPE} MATCHES "Debug")
  MESSAGE(STATUS "Additional Flags for Debug mode")
  SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -g -O1")
  SET(BUILD_SPECIFIC_LIBRARIES "")
ENDIF()

SET(libs
    ${PROTOBUF_LIBRARY} ${Z3_LIBRARIES} glog ${BUILD_SPECIFIC_LIBRARIES})

FILE(GLOB PROTO_FILES src/proto/*.proto)

PROTOBUF_GENERATE_CPP(PROTO_SRCS PROTO_HDRS ${PROTO_FILES})

PROTOBUF_GENERATE_PYTHON(PROTO_PY ${PROTO_FILES})

INCLUDE_DIRECTORIES(src/)
INCLUDE_DIRECTORIES(third_party/)
INCLUDE_DIRECTORIES(${PROTOBUF_INCLUDE_DIR})
INCLUDE_DIRECTORIES(${Z3_INCLUDE_DIRS})

INCLUDE_DIRECTORIES(${RE2_INCLUDE_DIR})
ADD_LIBRARY(trace_library ${PROTO_SRCS} ${PROTO_PY})
TARGET_LINK_LIBRARIES(trace_library ${libs})

ADD_LIBRARY(srtr_library
            src/srtr/srtr.cpp
            ${PROTO_SRCS} ${PROTO_PY}
            src/srtr/state_machine.cc)

TARGET_LINK_LIBRARIES(srtr_library
                      trace_library ${libs})

# ADD_EXECUTABLE(z3_tuner src/magic_debugging/tuning_sketch.cpp)
# TARGET_LINK_LIBRARIES(z3_tuner
# protobuf_library ${libs})