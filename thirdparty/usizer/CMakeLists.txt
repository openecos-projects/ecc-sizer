cmake_minimum_required(VERSION 3.10)

# 项目信息
project(SizerBin LANGUAGES CXX)

# 设置编译器选项
set(CMAKE_CXX_COMPILER g++)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopenmp -O3 -DNDEBUG -static-libgcc -static-libstdc++ -static -lpthread -lm -lgomp -finline-functions -fomit-frame-pointer -fno-operator-names -march=x86-64 -msse4.2 -mfpmath=sse -ffast-math -fexcess-precision=fast -ffinite-math-only")

# 源文件
set(SRC
    parser_helper.cpp
    timer_interface.cpp
    Circuit.cpp
    global.cpp
    Vcell.cpp
)

# 生成可执行文件
add_executable(sizerbin main.cpp ${SRC})

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
