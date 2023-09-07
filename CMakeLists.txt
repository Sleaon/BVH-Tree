
cmake_minimum_required (VERSION 3.0.2 FATAL_ERROR)
SET(CMAKE_C_STANDARD 11)
SET(CMAKE_CXX_STANDARD 17)
SET(CMAKE_CXX_FLAGS "-rdynamic -Wall -O2")

SET (PACKAGE_NAME "bvh")
PROJECT(${PACKAGE_NAME} LANGUAGES CXX)
SET(LIB_NAME "lib${PACKAGE_NAME}")
SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)	
SET(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
SET(ROOT_DIR ${PROJECT_SOURCE_DIR})

MESSAGE("project source dir is ${PROJECT_SOURCE_DIR}")

IF (CMAKE_VERSION VERSION_LESS 3.4)
  enable_language (C)
ENDIF ()
OPTION(BUILD_TEST "build tests" OFF)
OPTION(BUILD_BENCHMARK "build benchmark" OFF)
add_subdirectory(src)

IF(BUILD_BENCHMARK)
  add_subdirectory(benchmark)
ENDIF()

IF(BUILD_TEST)
  add_subdirectory(third_party/googletest)
  include(CTest)
  add_subdirectory(test)
  enable_testing()
ENDIF()