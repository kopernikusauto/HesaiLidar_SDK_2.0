cmake_minimum_required(VERSION 3.20)

PROJECT(HesaiSDK CXX)

set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 17)

set(HESAI_VERSION_MAJOR 2)
set(HESAI_VERSION_MINOR 0)
set(HESAI_VERSION_TINY  11)

#cmake_policy(SET CMP0053 NEW)
#set(CMAKE_BUILD_TYPE ON)
#set(CMAKE_BUILD_TYPE Release)
#add_definitions(-O3)
#add_compile_options(-Wall)

# 设置生成平台为 x64
if(_MSC_VER)
    set(CMAKE_GENERATOR_PLATFORM x64)
    set(OPENSSL_ROOT_DIR "C:/Program Files/OpenSSL-Win64")
endif()

option(HESAI_BUILD_SAMPLES "Build the included sample applications" OFF)
option(HESAI_ENABLE_CUDA "Enable Cuda targets" OFF)

configure_file(
  "${CMAKE_CURRENT_SOURCE_DIR}/Version.h.in"
  "${CMAKE_CURRENT_BINARY_DIR}/Version.h"
)

find_package(OpenSSL REQUIRED)
find_package(Threads)

add_library(HesaiDriverInterface INTERFACE driver/hesai_lidar_sdk.hpp)
target_include_directories(HesaiDriverInterface INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/driver>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>
    $<INSTALL_INTERFACE:include>
)
install(TARGETS HesaiDriverInterface)
install(FILES
    driver/hesai_lidar_sdk.hpp
    ${CMAKE_CURRENT_BINARY_DIR}/Version.h
    DESTINATION include)

add_subdirectory(libhesai)

if (HESAI_BUILD_SAMPLES)

    add_executable(sample
      test/test.cc
    )

    target_link_libraries(sample
        HesaiDriverInterface
        HesaiLidar
        HesaiSource
        HesaiPtcClient
        HesaiPtcParser
        OpenSSL::SSL
        OpenSSL::Crypto
    )

    if(HESAI_ENABLE_CUDA)
        set(CUDA_SOURCE_PROPERTY_FORMAT OBJ)
        set(CUDA_SEPARABLE_COMPILATION ON)
        set(CUDA_PROPAGATE_HOST_FLAGS OFF)

        enable_language(CUDA)

        # include_directories(${CUDA_INCLUDE_DIRS})
        # set(CUDA_NVCC_FLAGS -arch=sm_61;-O3;-G;-g;-std=c++14)#根据具体GPU性能更改算力参数
        # link_directories($ENV{CUDA_PATH}/lib/x64)

        file(GLOB_RECURSE CURRENT_HEADERS  *.h *.hpp *.cuh)
        file(GLOB CURRENT_SOURCES  *.cpp *.cu)
        file(GLOB CURRENT_SOURCES  *.cpp *.cu)

        source_group("Include" FILES ${CURRENT_HEADERS})
        source_group("Source" FILES ${CURRENT_SOURCES})

        add_executable(sample_gpu ${CURRENT_HEADERS} ${CURRENT_SOURCES}
            ./test/test.cu
            ./libhesai/UdpParserGpu/src/buffer.cu)

        target_link_libraries(sample_gpu
            HesaiLidar
            HesaiSource
            HesaiPtcCLient
            HesaiPtcParser
            OpenSSL::SSL
            OpenSSL::Crypto)
    endif(HESAI_ENABLE_CUDA)

endif(HESAI_BUILD_SAMPLES)
