
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -fomit-frame-pointer -funroll-all-loops -m64 -fprefetch-loop-arrays -ffast-math -fPIC -Wno-deprecated -g -gstrict-dwarf -Wunused -fpermissive")

# Please set your environment
set(TCLLIB "-L ../module/tcl/unix/install-sp/lib -ltcl")
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
    ${TCLLIB}
    OpenSTA
    -L${ZLIB_HOME} -lz
    -lm -lpthread -ldl
)