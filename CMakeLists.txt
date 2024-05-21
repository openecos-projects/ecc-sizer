cmake_minimum_required(VERSION 3.10)

# 项目信息
project(gsizer LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 20)

# 设置编译器选项
set(CMAKE_CXX_COMPILER g++)
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopenmp -static-libgcc -static-libstdc++ -static -lpthread -lm -lgomp -finline-functions -fomit-frame-pointer -fno-operator-names -march=x86-64 -msse4.2 -mfpmath=sse -ffast-math -fexcess-precision=fast -ffinite-math-only")

# set(CMAKE_BUILD_TYPE Release)
# set(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -DNDEBUG") # 
set(CMAKE_VERBOSE_MAKEFILEON ON)

set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -g -ggdb ") # -Wl --start-group

include_directories(${CMAKE_SOURCE_DIR}/src)
add_subdirectory(src)

# 包含目录（如果有必要）
# include_directories(${CMAKE_SOURCE_DIR}/include)

# 链接库（如果有必要）
# target_link_libraries(sizerbin <library_name>)

# 自定义编译选项（如果有必要）
# target_compile_definitions(sizerbin PRIVATE -DPARALLEL -DFLACH -DCOMPARE_TIMING_ENGINES)

# # 清理目标
# add_custom_target(clean
#     COMMAND ${CMAKE_COMMAND} -E remove *.o sizerbin
# )
