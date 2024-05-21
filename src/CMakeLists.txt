include_directories(${CMAKE_SOURCE_DIR}/src)
# 源文件
set(SRC
    parser_helper.cpp
    timer_interface.cpp
    Circuit.cpp
    global.cpp
    Vcell.cpp
)

add_executable(sizerbin main.cpp ${SRC})
