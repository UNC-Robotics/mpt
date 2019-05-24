# 3.10 introduces C++ 17 support
cmake_minimum_required (VERSION 3.10)
project (mpt_test)

# Enable C++17 (required for MPT)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Disable extensions to keep compatible with standards
set(CMAKE_CXX_EXTENSIONS OFF)

# See https://cmake.org/cmake/help/v3.2/prop_gbl/JOB_POOLS.html#prop_gbl:JOB_POOLS
# (TODO: it would be nice to be able to run concurrent tests on a pool
# that only allows 1 job.  It does not appear that CMake supports pool
# selection yet.)
# set_property(GLOBAL PROPERTY JOB_POOLS concurrent_pool=1)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(OPENMP REQUIRED)

if (OPENMP_FOUND)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

# Currently we assume that Nigh is checked out at the same level as
# MPT.
include_directories(../src ../../nigh/src)

# Create a test for each "*_test.cpp" file in this directory.
file(GLOB files "*_test.cpp")
foreach(file ${files})
    get_filename_component(test_name ${file} NAME_WE)
    # Add the executable based upon the test source
    add_executable(${test_name} ${file})
    target_link_libraries(${test_name} Eigen3::Eigen)

    # Add a custom command for the test's output (.log), making sure
    # that it depends on the test executable.
    add_custom_command(
        OUTPUT  ${test_name}.log
        COMMAND ${test_name} | tee ${test_name}.tmp && mv ${test_name}.tmp ${test_name}.log
        DEPENDS ${test_name}
        COMMENT "TESTING ${test_name}")

    # Add the test to the top-level 'all' target
    add_custom_target(run_${test_name} ALL DEPENDS ${test_name}.log)
endforeach()
