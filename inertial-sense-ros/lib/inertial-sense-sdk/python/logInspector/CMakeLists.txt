cmake_minimum_required(VERSION 2.8.3)
project(log_reader)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -fPIC")

find_package(pybind11 REQUIRED)
find_package(Threads)


add_definitions(-DPYBIND11_PYTHON_VERSION=3.6.6)

set(SDK_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../)
add_subdirectory(${SDK_DIR} SDK)

include_directories(
    include
    ${SDK_DIR}/src
)


pybind11_add_module(log_reader
    src/log_reader.cpp
    include/log_reader.h
)
target_link_libraries(log_reader InertialSense)

# Don't prepend wrapper library name with lib and add to Python libs.
set_target_properties(log_reader PROPERTIES
    PREFIX ""
    OUTPUT_NAME "log_reader"
    SUFFIX ".so"
    LIBRARY_OUTPUT_DIRECTORY ..
)
