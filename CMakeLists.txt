cmake_minimum_required(VERSION 3.21)

project(ArmForge
    VERSION 1.0.0
    DESCRIPTION "Six axis arm kinematics, trajectory planning and motion simulation in modern C++"
    LANGUAGES CXX)

# ------------------------------------------------------------------ language
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

option(ARMFORGE_BUILD_TESTS "Build unit tests" ON)
option(ARMFORGE_BUILD_BENCH "Build benchmarks" ON)
option(ARMFORGE_SANITIZE "Enable ASan/UBSan (gcc/clang only)" OFF)
option(ARMFORGE_WARNINGS_AS_ERRORS "Treat warnings as errors" OFF)

find_package(Threads REQUIRED)

# ------------------------------------------------------------------ warnings
set(ARMFORGE_WARN_FLAGS "")
if(MSVC)
    list(APPEND ARMFORGE_WARN_FLAGS /W4 /permissive- /utf-8 /Zc:__cplusplus)
else()
    list(APPEND ARMFORGE_WARN_FLAGS -Wall -Wextra -Wpedantic -Wshadow -Wnon-virtual-dtor)
endif()
if(ARMFORGE_WARNINGS_AS_ERRORS)
    if(MSVC)
        list(APPEND ARMFORGE_WARN_FLAGS /WX)
    else()
        list(APPEND ARMFORGE_WARN_FLAGS -Werror)
    endif()
endif()

# ------------------------------------------------------------------ sources
set(ARMFORGE_SRC
    src/model/ArmModel.cpp
    src/kin/Kinematics.cpp
    src/traj/Trajectory.cpp
    src/collision/Collision.cpp
    src/control/ServoSim.cpp
    src/ai/ActionPlanner.cpp
    src/viz/SvgRenderer.cpp
)

set(ARMFORGE_HDR
    src/core/Math.h
    src/core/Status.h
    src/model/ArmModel.h
    src/kin/Kinematics.h
    src/traj/Trajectory.h
    src/collision/Collision.h
    src/control/ServoSim.h
    src/ai/ActionPlanner.h
    src/viz/SvgRenderer.h
)

add_library(armforge_core STATIC ${ARMFORGE_SRC} ${ARMFORGE_HDR})
target_include_directories(armforge_core PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>)
target_link_libraries(armforge_core PUBLIC Threads::Threads)
target_compile_options(armforge_core PRIVATE ${ARMFORGE_WARN_FLAGS})

if(ARMFORGE_SANITIZE AND NOT MSVC)
    target_compile_options(armforge_core PUBLIC -fsanitize=address,undefined -fno-omit-frame-pointer)
    target_link_options(armforge_core PUBLIC -fsanitize=address,undefined)
endif()

# ------------------------------------------------------------------ binary
add_executable(armforge src/app/main.cpp)
target_link_libraries(armforge PRIVATE armforge_core)
target_compile_options(armforge PRIVATE ${ARMFORGE_WARN_FLAGS})

# ------------------------------------------------------------------ tests
if(ARMFORGE_BUILD_TESTS)
    enable_testing()
    add_executable(armforge_tests
        tests/test_main.cpp
        tests/test_math.cpp
        tests/test_model.cpp
        tests/test_kinematics.cpp
        tests/test_trajectory.cpp
        tests/test_collision.cpp
        tests/test_servo.cpp
        tests/test_planner.cpp
    )
    target_link_libraries(armforge_tests PRIVATE armforge_core)
    target_compile_options(armforge_tests PRIVATE ${ARMFORGE_WARN_FLAGS})
    add_test(NAME armforge_tests COMMAND armforge_tests)
endif()

# ------------------------------------------------------------------ bench
if(ARMFORGE_BUILD_BENCH)
    add_executable(armforge_bench bench/bench.cpp)
    target_link_libraries(armforge_bench PRIVATE armforge_core)
    target_compile_options(armforge_bench PRIVATE ${ARMFORGE_WARN_FLAGS})
endif()

foreach(target armforge armforge_tests armforge_bench)
    if(TARGET ${target})
        set_target_properties(${target} PROPERTIES
            RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
            RUNTIME_OUTPUT_DIRECTORY_DEBUG "${CMAKE_BINARY_DIR}/bin"
            RUNTIME_OUTPUT_DIRECTORY_RELEASE "${CMAKE_BINARY_DIR}/bin")
    endif()
endforeach()

message(STATUS "ArmForge | compiler ${CMAKE_CXX_COMPILER_ID} ${CMAKE_CXX_COMPILER_VERSION}"
               " | build ${CMAKE_BUILD_TYPE} | c++${CMAKE_CXX_STANDARD}")
