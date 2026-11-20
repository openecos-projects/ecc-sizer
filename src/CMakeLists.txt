find_package(spdlog REQUIRED)

set(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -g -Wall -Wextra")
set(CMAKE_C_FLAGS_DEBUG  "${CMAKE_C_FLAGS} -O0 -g -Wall -Wextra")

set(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -DNDEBUG")
set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS} -O3")

set(OPENROAD_HOME ${CMAKE_CURRENT_SOURCE_DIR}/../thirdparty/OpenROAD/src/)
# These two private headers implement STA power and OpenROAD buffer trees.
# All public include paths come from OpenROAD's transitive targets.
include_directories(${OPENROAD_HOME}/sta ${OPENROAD_HOME}/sta/include/sta
                    ${OPENROAD_HOME}/rsz/src)

# Repair algorithms depend on OpenDB/STA and edit primitives, not cell sizing.
set(SOURCES
    design_edit.cpp special_net.cpp lef_layer_rc.cpp
    repair_config.cpp repair_backend.cpp repair_drv.cpp repair_timing.cpp
    repair_path.cpp repair_vg.cpp repair_vg_cuda.cpp
    repair_gobi.cpp repair_gobi_cuda.cpp)

add_library(sizer_placement STATIC legalize_core.cpp
    ../thirdparty/dreamplace-legalize/greedy_legalize/src/greedy_legalize_cpu.cpp
    ../thirdparty/dreamplace-legalize/greedy_legalize/src/legalize_bin_cpu.cpp
    ../thirdparty/dreamplace-legalize/greedy_legalize/src/merge_bin_cpu.cpp
    ../thirdparty/dreamplace-legalize/utility/src/msg.cpp)
target_link_libraries(sizer_placement PUBLIC spdlog::spdlog)
target_include_directories(sizer_placement PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../thirdparty/dreamplace-legalize)

if(ENABLE_VG_CUDA OR ENABLE_GOBI_CUDA)
    include(CheckLanguage)
    check_language(CUDA)
    if(NOT CMAKE_CUDA_COMPILER)
        message(FATAL_ERROR "CUDA backend enabled but no CUDA compiler was found")
    endif()
    enable_language(CUDA)
    find_package(CUDAToolkit REQUIRED)
endif()

if(ENABLE_VG_CUDA)
    list(REMOVE_ITEM SOURCES repair_vg_cuda.cpp)
    list(APPEND SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/repair_vg_cuda.cu")
else()
    message(STATUS "VG CUDA solver disabled; using CPU VG solver")
endif()

if(ENABLE_GOBI_CUDA)
    list(REMOVE_ITEM SOURCES repair_gobi_cuda.cpp)
    list(APPEND SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/repair_gobi_cuda.cu")
else()
    message(STATUS "GOBI CUDA solver disabled; using the unavailable backend stub")
endif()

add_library(sizer_algorithms STATIC ${SOURCES})
add_executable(Sizer application.cpp design_io.cpp)
target_link_libraries(sizer_algorithms PUBLIC openroad sizer_placement)
target_link_libraries(Sizer sizer_algorithms)
# GUI objects refer back to OpenROAD's Tcl entry symbols. Retain those symbols
# without invoking the standalone entry (which would create a second session).
set_property(TARGET Sizer PROPERTY LINK_LIBRARY_OVERRIDE "WHOLE_ARCHIVE,openroad")

if(ENABLE_VG_CUDA OR ENABLE_GOBI_CUDA)
    set_target_properties(sizer_algorithms PROPERTIES CUDA_STANDARD 17
                          CUDA_STANDARD_REQUIRED ON
                          CUDA_ARCHITECTURES native)
    target_link_libraries(sizer_algorithms PUBLIC CUDA::cudart)
endif()
if(ENABLE_VG_CUDA)
    target_compile_definitions(sizer_algorithms PRIVATE ECC_SIZER_ENABLE_VG_CUDA)
    target_compile_options(sizer_algorithms PRIVATE $<$<COMPILE_LANGUAGE:CUDA>:--fmad=false>)
endif()
if(ENABLE_GOBI_CUDA)
    target_compile_definitions(sizer_algorithms PRIVATE ECC_SIZER_ENABLE_GOBI_CUDA)
    target_compile_options(sizer_algorithms PRIVATE $<$<COMPILE_LANGUAGE:CUDA>:--fmad=false>)
endif()

if(BUILD_TESTING)
    find_package(GTest REQUIRED)
    find_package(Python3 COMPONENTS Interpreter REQUIRED)
    add_test(NAME ecc_sizer_application
        COMMAND ${Python3_EXECUTABLE} ${CMAKE_SOURCE_DIR}/tests/application_test.py $<TARGET_FILE:Sizer>)
    set_tests_properties(ecc_sizer_application PROPERTIES
        ENVIRONMENT "PYTHONDONTWRITEBYTECODE=1" TIMEOUT 600)
    add_test(NAME ecc_sizer_repair_regression
             COMMAND ${Python3_EXECUTABLE} ${CMAKE_SOURCE_DIR}/tests/repair_regression_test.py)

    add_executable(ecc_sizer_repair_buffer_budget_test
        ${CMAKE_CURRENT_SOURCE_DIR}/../tests/repair_buffer_budget_test.cpp)
    target_include_directories(ecc_sizer_repair_buffer_budget_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
    target_link_libraries(ecc_sizer_repair_buffer_budget_test PRIVATE GTest::gtest_main)
    add_test(NAME ecc_sizer_repair_buffer_budget COMMAND ecc_sizer_repair_buffer_budget_test)

    add_executable(ecc_sizer_repair_config_test
        ${CMAKE_CURRENT_SOURCE_DIR}/../tests/repair_config_test.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/repair_config.cpp)
    target_include_directories(ecc_sizer_repair_config_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
    target_link_libraries(ecc_sizer_repair_config_test PRIVATE GTest::gtest_main)
    add_test(NAME ecc_sizer_repair_config COMMAND ecc_sizer_repair_config_test)

    add_executable(ecc_sizer_repair_path_feedback_test ../tests/repair_path_feedback_test.cpp)
    target_include_directories(ecc_sizer_repair_path_feedback_test PRIVATE .)
    target_link_libraries(ecc_sizer_repair_path_feedback_test PRIVATE GTest::gtest_main)
    add_test(NAME ecc_sizer_repair_path_feedback COMMAND ecc_sizer_repair_path_feedback_test)

    set(VG_CUDA_TEST_BACKEND
        ${CMAKE_CURRENT_SOURCE_DIR}/repair_vg_cuda.cpp)
    if(ENABLE_VG_CUDA)
        set(VG_CUDA_TEST_BACKEND
            ${CMAKE_CURRENT_SOURCE_DIR}/repair_vg_cuda.cu)
    endif()
    add_executable(ecc_sizer_repair_vg_cuda_test
        ${CMAKE_CURRENT_SOURCE_DIR}/../tests/repair_vg_cuda_test.cpp
        ${VG_CUDA_TEST_BACKEND})
    target_include_directories(ecc_sizer_repair_vg_cuda_test PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR})
    target_link_libraries(ecc_sizer_repair_vg_cuda_test PRIVATE
        GTest::gtest
        GTest::gtest_main)
    if(ENABLE_VG_CUDA)
        set_target_properties(ecc_sizer_repair_vg_cuda_test PROPERTIES
            CUDA_STANDARD 17 CUDA_STANDARD_REQUIRED ON
            CUDA_ARCHITECTURES native)
        target_compile_definitions(ecc_sizer_repair_vg_cuda_test PRIVATE ECC_SIZER_ENABLE_VG_CUDA)
        target_compile_options(ecc_sizer_repair_vg_cuda_test PRIVATE $<$<COMPILE_LANGUAGE:CUDA>:--fmad=false>)
        target_link_libraries(ecc_sizer_repair_vg_cuda_test PRIVATE CUDA::cudart)
    endif()
    add_test(NAME ecc_sizer_repair_vg_cuda
             COMMAND ecc_sizer_repair_vg_cuda_test)

    set(GOBI_CUDA_TEST_BACKEND
        ${CMAKE_CURRENT_SOURCE_DIR}/repair_gobi_cuda.cpp)
    if(ENABLE_GOBI_CUDA)
        set(GOBI_CUDA_TEST_BACKEND
            ${CMAKE_CURRENT_SOURCE_DIR}/repair_gobi_cuda.cu)
    endif()
    add_executable(ecc_sizer_repair_gobi_cuda_test
        ${CMAKE_CURRENT_SOURCE_DIR}/../tests/repair_gobi_cuda_test.cpp
        ${GOBI_CUDA_TEST_BACKEND})
    target_include_directories(ecc_sizer_repair_gobi_cuda_test PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR})
    target_link_libraries(ecc_sizer_repair_gobi_cuda_test PRIVATE
        GTest::gtest
        GTest::gtest_main spdlog::spdlog)
    if(ENABLE_GOBI_CUDA)
        set_target_properties(ecc_sizer_repair_gobi_cuda_test PROPERTIES
            CUDA_STANDARD 17 CUDA_STANDARD_REQUIRED ON
            CUDA_ARCHITECTURES native)
        target_compile_definitions(ecc_sizer_repair_gobi_cuda_test PRIVATE ECC_SIZER_ENABLE_GOBI_CUDA)
        target_compile_options(ecc_sizer_repair_gobi_cuda_test PRIVATE $<$<COMPILE_LANGUAGE:CUDA>:--fmad=false>)
        target_link_libraries(ecc_sizer_repair_gobi_cuda_test PRIVATE CUDA::cudart)
    endif()
    add_test(NAME ecc_sizer_repair_gobi_cuda
             COMMAND ecc_sizer_repair_gobi_cuda_test)

    add_executable(ecc_sizer_buffer_inserter_test
        ${CMAKE_CURRENT_SOURCE_DIR}/../tests/buffer_inserter_test.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/design_edit.cpp
    )
    target_include_directories(ecc_sizer_buffer_inserter_test PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${OPENROAD_HOME}/utl/include
    )
    target_link_libraries(ecc_sizer_buffer_inserter_test openroad sizer_placement)
    add_test(NAME ecc_sizer_buffer_inserter
             COMMAND ecc_sizer_buffer_inserter_test)

    add_executable(ecc_sizer_fanout_repair_test
        ${CMAKE_CURRENT_SOURCE_DIR}/../tests/fanout_repair_test.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/design_edit.cpp
    )
    target_include_directories(ecc_sizer_fanout_repair_test PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${OPENROAD_HOME}/utl/include
    )
    target_link_libraries(ecc_sizer_fanout_repair_test sizer_algorithms)
    add_test(NAME ecc_sizer_fanout_repair
             COMMAND ecc_sizer_fanout_repair_test)

    add_executable(ecc_sizer_buffer_inserter_sta_test
        ${CMAKE_CURRENT_SOURCE_DIR}/../tests/buffer_inserter_sta_test.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/design_edit.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/repair_drv.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/repair_backend.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/repair_config.cpp

        ${CMAKE_CURRENT_SOURCE_DIR}/repair_vg.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/repair_vg_cuda.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/repair_gobi.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/repair_gobi_cuda.cpp

        ${CMAKE_CURRENT_SOURCE_DIR}/repair_path.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/repair_timing.cpp

    )
    target_include_directories(ecc_sizer_buffer_inserter_sta_test PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}
    )
    target_compile_definitions(ecc_sizer_buffer_inserter_sta_test PRIVATE
        ECC_SIZER_TEST_DATA_DIR="${CMAKE_SOURCE_DIR}/tests/data"
        ECC_SIZER_TEST_OUTPUT_DIR="${CMAKE_BINARY_DIR}"
    )
    target_link_libraries(ecc_sizer_buffer_inserter_sta_test PRIVATE
        GTest::gtest
        GTest::gtest_main
        tst
        openroad sizer_placement
    )
    set_property(TARGET ecc_sizer_buffer_inserter_sta_test PROPERTY
                 LINK_LIBRARY_OVERRIDE "WHOLE_ARCHIVE,openroad")
    add_test(NAME ecc_sizer_buffer_inserter_sta
             COMMAND ecc_sizer_buffer_inserter_sta_test)
    set_tests_properties(ecc_sizer_buffer_inserter_sta PROPERTIES
        WORKING_DIRECTORY ${OpenROAD_DIR}/src/est
    )

    add_executable(ecc_sizer_lef_layer_rc_test
        ${CMAKE_CURRENT_SOURCE_DIR}/../tests/lef_layer_rc_test.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/lef_layer_rc.cpp
    )
    target_include_directories(ecc_sizer_lef_layer_rc_test PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}
    )
    add_test(NAME ecc_sizer_lef_layer_rc COMMAND ecc_sizer_lef_layer_rc_test)

    add_executable(ecc_sizer_special_net_test
        ${CMAKE_CURRENT_SOURCE_DIR}/../tests/special_net_test.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/special_net.cpp
    )
    target_include_directories(ecc_sizer_special_net_test PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${OPENROAD_HOME}/utl/include
    )
    target_link_libraries(ecc_sizer_special_net_test odb)
    add_test(NAME ecc_sizer_special_net COMMAND ecc_sizer_special_net_test)

    add_executable(ecc_sizer_legalize_test
        ${CMAKE_CURRENT_SOURCE_DIR}/../tests/legalize_test.cpp
    )
    target_include_directories(ecc_sizer_legalize_test PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/../thirdparty/dreamplace-legalize
    )
    target_link_libraries(ecc_sizer_legalize_test PRIVATE sizer_placement)
    add_test(NAME ecc_sizer_legalize COMMAND ecc_sizer_legalize_test)
endif()
