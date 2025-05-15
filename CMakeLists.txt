cmake_minimum_required(VERSION 3.16)
project(gstundistort LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# OpenMP
find_package(OpenMP)
if(OpenMP_CXX_FOUND)
  message(STATUS "OpenMP enabled")
endif()

# GStreamer
find_package(PkgConfig REQUIRED)
pkg_check_modules(GST REQUIRED gstreamer-1.0 gstreamer-video-1.0)
include_directories(${GST_INCLUDE_DIRS})
link_directories(${GST_LIBRARY_DIRS})
add_definitions(${GST_CFLAGS_OTHER})

# Your plugin source
add_library(gstundistort SHARED gstundistort.cpp)
set_target_properties(gstundistort PROPERTIES OUTPUT_NAME "gstundistort")

# SIMD flags for ARM (optional)
if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
  target_compile_options(gstundistort PRIVATE -march=armv8-a+simd -ftree-vectorize -O3 -DNDEBUG)
else()
  target_compile_options(gstundistort PRIVATE -O3 -DNDEBUG)
endif()

target_link_libraries(gstundistort ${GST_LIBRARIES})
if(OpenMP_CXX_FOUND)
  target_link_libraries(gstundistort OpenMP::OpenMP_CXX)
endif()