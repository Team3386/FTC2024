project(tornades_jni)

cmake_minimum_required(VERSION 3.22)

set(OpenCV_DIR ${CMAKE_SOURCE_DIR}/opencv/sdk/native/jni)
find_package(OpenCV REQUIRED COMPONENTS core imgproc)

set(ncnn_DIR ${CMAKE_SOURCE_DIR}/ncnn/lib/cmake/ncnn)
find_package(ncnn REQUIRED)

set(CMAKE_CXX_FLAGS
        "${CMAKE_CXX_FLAGS} -ffast-math -Ofast -fstrict-aliasing -mcpu=cortex-a53 -DNDEBUG -fno-exceptions -fomit-frame-pointer -fno-asynchronous-unwind-tables -fno-unwind-tables -fopenmp"
        )
set(CMAKE_CXX_FLAGS
        "${CMAKE_CXX_FLAGS} -fvisibility=hidden -fvisibility-inlines-hidden -fdata-sections -ffunction-sections"
        )
set(CMAKE_SHARED_LINKER_FLAGS
        "${CMAKE_SHARED_LINKER_FLAGS} -Wl,--gc-sections -Wl,-z,nocopyreloc")

# disable rtti and exceptions
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fno-rtti -fno-exceptions")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mno-outline-atomics")

add_library(ncnn_processor SHARED NcnnProcessor.cpp jni.cpp)

target_link_libraries(ncnn_processor PUBLIC ncnn opencv_imgproc opencv_core)