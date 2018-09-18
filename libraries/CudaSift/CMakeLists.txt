cmake_minimum_required(VERSION 2.6)

project(cudaSift)
set(cudaSift_VERSION_MAJOR 2)
set(cudaSift_VERSION_MINOR 0)
set(cudaSift_VERSION_PATCH 0)

set(CMAKE_CXX_STANDARD 17)

find_package(CUDA REQUIRED)
list(APPEND CUDA_NVCC_FLAGS "-lineinfo;--compiler-options;-O2;")

set(SOURCES
    cudaImage.cu
    cudaImage.h
    cudaSift.h
    cudaSiftD.h
    cudaSiftD.cu
    cudaSiftH.h
    cudaSiftH.cu
    cudautils.h
    matching.cu
)

cuda_add_library(cudasift ${SOURCES} STATIC)
set_target_properties(
    cudasift PROPERTIES
    CUDA_SEPARABLE_COMPILATION ON
    COMPILE_FLAGS "-O2 -msse2"
)

target_include_directories(cudasift PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(cudasift ${CUDA_LIBRARIES})