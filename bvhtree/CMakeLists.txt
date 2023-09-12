add_library(${LIB_NAME} INTERFACE)
find_package(Threads)
if (Threads_FOUND)
    target_link_libraries(${LIB_NAME} INTERFACE Threads::Threads)
endif()

target_include_directories(${LIB_NAME} INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/src)

IF(BUILD_TEST)
  include(CTest)
  add_subdirectory(test)
  enable_testing()
ENDIF()