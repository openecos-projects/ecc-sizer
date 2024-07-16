cmake_minimum_required(VERSION 3.10)

# 项目信息
project(SizerBin LANGUAGES CXX)

# 设置编译器选项
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
