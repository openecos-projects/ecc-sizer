
set(CMAKE_CXX_STANDARD 14)

# Please set your environment
# set(TCLLIB "-L ../module/tcl/unix/install-sp/lib -ltcl")
set(ZLIB_HOME "__YOUR_ZLIB_HOME__/lib")
# set(CMAKE_BUILD_TYPE Debug)
# set(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -g -ggdb") 

# Below references submodule 
set(OPENSTA_HOME ${CMAKE_CURRENT_SOURCE_DIR}/../module/OpenSTA)
set(OPENSTALIB_HOME "${CMAKE_CURRENT_SOURCE_DIR}/../module/OpenSTA/install-sp")

include_directories(${ZLIB_HOME}/../include)
include_directories(${OPENSTALIB_HOME}/include)

file(GLOB SOURCES "*.cpp")

add_executable(TritonSizer ${SOURCES})

target_link_libraries(TritonSizer 
    ${TCL_LIB}
    OpenSTA
    -L${ZLIB_HOME} -lz
    -lm -lpthread -ldl
)