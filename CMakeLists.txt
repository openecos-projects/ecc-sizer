cmake_minimum_required(VERSION 3.24)

project(ecc_sizer LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

option(ENABLE_VG_CUDA
       "Build the optional CUDA VG solver for Sizer VG" OFF)
option(ENABLE_GOBI_CUDA
       "Build the CUDA GOBI solver migrated from OpenROAD rsz" OFF)

include(CTest)

set(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3")
set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS} -O3")

set(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -g -Wall -Wextra")
set(CMAKE_C_FLAGS_DEBUG  "${CMAKE_C_FLAGS} -O0 -g -Wall -Wextra")
add_compile_options("-D_GLIBCXX_USE_CXX11_ABI=1")
set(OpenROAD_DIR ${CMAKE_SOURCE_DIR}/thirdparty/OpenROAD)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(ENABLE_TESTS OFF)
set(BUILD_SYN OFF CACHE BOOL "Disable OpenROAD synthesis support for ECC Sizer" FORCE)

add_subdirectory(src)
add_subdirectory(thirdparty/OpenROAD)
