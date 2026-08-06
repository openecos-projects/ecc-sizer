
# set(CMAKE_CXX_STANDARD 14)

# Please set your environment
# set(TCLLIB "-L ../module/tcl/unix/install-sp/lib -ltcl")
# set(ZLIB_HOME "__YOUR_ZLIB_HOME__/lib")
# set(CMAKE_BUILD_TYPE Debug)
# set(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -g -ggdb") 

# add_subdirectory(thirdparty/OpenROAD)
# include("openroad")
# Below references submodule
# set(CMAKE_BUILD_TYPE Debug) # -O0 -Og -fstandalone-debug
# set(CMAKE_BUILD_TYPE Release) # -O0 -Og -fstandalone-debug

set(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -g -Wall -Wextra") # -fsanitize=address -Wl --start-group
set(CMAKE_C_FLAGS_DEBUG  "${CMAKE_C_FLAGS} -O0 -g -Wall -Wextra")

set(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -DNDEBUG") #  -DNDEBUG # -DNDEBUG
set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS} -O3")

set(OPENROAD_HOME ${CMAKE_CURRENT_SOURCE_DIR}/../thirdparty/OpenROAD/src/)
set(OPENROAD_HEADERS ${CMAKE_CURRENT_SOURCE_DIR}/../thirdparty/OpenROAD/include/)  
set(OPENSTA_HOME ${OPENROAD_HOME}/sta)
set(OPENSTA_HDEAR_HOME "${OPENSTA_HOME}/include/")
set(OPENSTA_para_HDEAR_HOME "${OPENSTA_HOME}/parasitics/")
set(OPENDB_HDEAR_HOME "${OPENROAD_HOME}/odb/include/")
set(OPENDP_HDEAR_HOME "${OPENROAD_HOME}/dpl/include/")
set(FASTROUTE_HDEAR_HOME "${OPENROAD_HOME}/grt/include/")

list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake/")

option(ENABLE_PROFILER "Enable gperftools CPU profiler support" OFF)
if(ENABLE_PROFILER)
    find_package(PROFILER QUIET)
    if(NOT LIBPROFILER_FOUND)
        message(FATAL_ERROR "ENABLE_PROFILER=ON requires gperftools profiler headers and library")
    endif()
    message(STATUS "gperftools profiler support enabled")
else()
    set(LIBPROFILER_FOUND FALSE)
    message(STATUS "gperftools profiler support disabled")
endif()

message(STATUS )
set(dbSTA_HDEAR_HOME "${OPENROAD_HOME}/dbSta/include/")
message(STATUS "OPENSTA_HDEAR_HOME: ${OPENSTA_HDEAR_HOME}")
message(STATUS "OPENSTA_para_HDEAR_HOME: ${OPENSTA_para_HDEAR_HOME}")
message(STATUS "OPENSTA_HOME: ${OPENSTA_HOME}")
# include_directories(${ZLIB_HOME}/../include)
include_directories(${OPENSTA_HDEAR_HOME}
                    ${dbSTA_HDEAR_HOME}
                    ${OPENSTA_HDEAR_HOME}/sta
                    ${OPENSTA_para_HDEAR_HOME}
                    ${TCL_INCLUDE_PATH}
                    ${OPENSTA_HOME}
                    ${OPENDB_HDEAR_HOME}
                    ${OPENDP_HDEAR_HOME}
                    ${FASTROUTE_HDEAR_HOME}
                    )

file(GLOB SOURCES "*.cpp")

add_executable(Sizer ${SOURCES})

if(LIBPROFILER_FOUND)
    target_compile_definitions(Sizer PRIVATE HAVE_GPERFTOOLS_PROFILER)
    target_link_libraries(Sizer PRIVATE gperftools::profiler)
endif()

target_link_libraries(Sizer 
    openroad
    ${TCL_LIB}
    utl_lib
    utl
    odb
    OpenSTA
    dbSta
    grt
    # omp
    gui
    -L${ZLIB_HOME} -lz
    -lm -lpthread -ldl
    dbSta_lib
    grt_lib
    # openroad
    odb
    # utl
)
