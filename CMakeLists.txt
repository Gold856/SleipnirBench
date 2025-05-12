# Disable in-source builds to prevent source tree corruption
if("${CMAKE_CURRENT_SOURCE_DIR}" STREQUAL "${CMAKE_CURRENT_BINARY_DIR}")
    message(
        FATAL_ERROR
        "
FATAL: In-source builds are not allowed.
       You should create a separate directory for build files.
"
    )
endif()

cmake_minimum_required(VERSION 3.24)

# Make `GIT_SUBMODULES ""` initialize no submodules
cmake_policy(SET CMP0097 NEW)

set(CMAKE_COLOR_DIAGNOSTICS ON)

# Set default build type to release
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "" FORCE)
endif()

project(SleipnirBench CXX)

# Generate compile_commands.json by default
if(NOT CMAKE_EXPORT_COMPILE_COMMANDS)
    set(CMAKE_EXPORT_COMPILE_COMMANDS "YES" CACHE STRING "" FORCE)
endif()

file(GLOB_RECURSE cpp_src src/*.cpp)
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    set_property(
        SOURCE ${cpp_src}
        PROPERTY
            COMPILE_FLAGS
                "-Wno-unused-parameter -Wno-array-bounds -Wno-class-memaccess -Wno-dangling-reference -Wno-maybe-uninitialized -Wno-stringop-overflow"
    )
endif()

add_executable(SleipnirBench ${cpp_src})

if(NOT MSVC)
    target_compile_options(SleipnirBench PRIVATE -Wall -Wextra -Werror -pedantic)
else()
    target_compile_options(SleipnirBench PRIVATE /MP /utf-8 /WX)
endif()
target_compile_definitions(SleipnirBench PRIVATE NDEBUG)
target_compile_features(SleipnirBench PUBLIC cxx_std_23)
target_include_directories(SleipnirBench PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)

# FIXME: Linker emits LTO type mismatch warning for sb02od_(), and benchmark
#        crashes on sb02od_()
# set_property(TARGET SleipnirBench PROPERTY INTERPROCEDURAL_OPTIMIZATION TRUE)

include(FetchContent)

# Options for using a package manager (e.g., vcpkg) for certain dependencies
option(USE_SYSTEM_BENCHMARK "Use system benchmark" OFF)
set(BUILD_TESTING OFF)
fetchcontent_declare(
    Sleipnir
    GIT_REPOSITORY https://github.com/calcmogul/Sleipnir
    GIT_TAG allwpilib-downgrade-to-cpp20
)
fetchcontent_makeavailable(Sleipnir)

# benchmark dependency
option(BENCHMARK_ENABLE_TESTING "Enable testing of the benchmark library." OFF)
if(NOT USE_SYSTEM_BENCHMARK)
    fetchcontent_declare(
        benchmark
        GIT_REPOSITORY https://github.com/google/benchmark.git
        GIT_TAG v1.9.2
        PATCH_COMMAND
            git apply ${CMAKE_CURRENT_SOURCE_DIR}/benchmark-athena-cpuinfo.patch
        UPDATE_DISCONNECTED TRUE
    )
    fetchcontent_makeavailable(benchmark)
else()
    find_package(benchmark CONFIG REQUIRED)
endif()
target_link_libraries(SleipnirBench PRIVATE benchmark::benchmark Sleipnir)
