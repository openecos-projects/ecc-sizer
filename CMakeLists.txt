cmake_minimum_required(VERSION 3.10)

# 项目信息
project(gsizer LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)

# 设置编译器选项
# set(CMAKE_CXX_COMPILER g++)
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopenmp -static-libgcc -static-libstdc++ -static -lpthread -lm -lgomp -finline-functions -fomit-frame-pointer -fno-operator-names -march=x86-64 -msse4.2 -mfpmath=sse -ffast-math -fexcess-precision=fast -ffinite-math-only")

# set(CMAKE_BUILD_TYPE Release)
# set(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -DNDEBUG") # 
set(CMAKE_VERBOSE_MAKEFILEON ON)

# -fstandalone-debug
# set(CMAKE_BUILD_TYPE Debug) -O0 -Og -fstandalone-debug
set(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -g -fstandalone-debug") # -Wl --start-group
add_compile_options("-D_GLIBCXX_USE_CXX11_ABI=1")
set(OpenROAD_DIR ${CMAKE_SOURCE_DIR}/thirdparty/OpenROAD)
set(iEDA_DIR ${CMAKE_SOURCE_DIR}/thirdparty/iEDA)
set(taskflow_DIR ${CMAKE_SOURCE_DIR}/thirdparty/taskflow)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/build)
# include_directories(${CMAKE_SOURCE_DIR}/src)
# include_directories(${CMAKE_SOURCE_DIR}/thirdparty/OpenROAD/include)

# add_subdirectory(src)
# add_subdirectory(thirdparty/iEDA)

# Road 1
# add_subdirectory(thirdparty/usizer)
# add_subdirectory(thirdparty/iEDA)

# Road 2
add_subdirectory(thirdparty/OpenROAD)

# Road 3
# set(CMAKE_INSTALL_PREFIX ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/TritonSizer/module/OpenSTA/install-sp)
# add_subdirectory(thirdparty/TritonSizer)
