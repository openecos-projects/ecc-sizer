cmake_minimum_required(VERSION 3.10)
project(sizer)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_INSTALL_PREFIX ${CMAKE_CURRENT_SOURCE_DIR}/module/OpenSTA/install-sp)
add_subdirectory(module/OpenSTA)
add_subdirectory(src)