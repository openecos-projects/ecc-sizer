include_directories(${CMAKE_SOURCE_DIR})
# include_directories(${OpenROAD_DIR}/include/)
# message(STATUS "OpenROAD_DIR: ${OpenROAD_DIR}")
# 源文件
set(SRC
    parser_helper.cpp
    timer_interface.cpp
    Circuit.cpp
    global.cpp
    Vcell.cpp
    main.cpp
    # Stepwatch.cpp
    )
add_executable(gsizer ${SRC})

# target_link_libraries(contest_sizer
#     PUBLIC
#     stt_lib
#     odb
#     OpenSTA
#     dbSta_lib
#     grt_lib
#     utl_lib
#     )