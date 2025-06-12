cmake_minimum_required(VERSION 3.16)
project(gstundistort)

# GStreamer
find_package(PkgConfig REQUIRED)
pkg_check_modules(GST REQUIRED gstreamer-1.0 gstreamer-video-1.0)
include_directories(${GST_INCLUDE_DIRS})
link_directories(${GST_LIBRARY_DIRS})
add_definitions(${GST_CFLAGS_OTHER})

find_package(OpenCV REQUIRED COMPONENTS core calib3d imgproc)
include_directories(${OpenCV_INCLUDE_DIRS})

# Your plugin
add_library(gstundistort SHARED gstundistort.cpp)
set_target_properties(gstundistort PROPERTIES OUTPUT_NAME "gstundistort")

if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
  target_compile_options(gstundistort PRIVATE -march=armv8-a+simd -ftree-vectorize -O3 -DNDEBUG)
else()
  target_compile_options(gstundistort PRIVATE -O3 -DNDEBUG)
endif()

target_link_libraries(gstundistort ${GST_LIBRARIES} ${OpenCV_LIBS})
target_compile_definitions(gstundistort PRIVATE USE_CV=1)

if(OpenMP_CXX_FOUND)
  target_link_libraries(gstundistort OpenMP::OpenMP_CXX)
endif()