cmake_minimum_required(VERSION 3.16)
project(CANSimulator VERSION 1.0.0 LANGUAGES CXX)

if(POLICY CMP0135)
    cmake_policy(SET CMP0135 NEW)
endif()

set(CMAKE_CXX_STANDARD          17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS        OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_compile_options(-Wall -Wextra -Wpedantic -Wno-unused-parameter)

if(CMAKE_BUILD_TYPE STREQUAL "Release")
    add_compile_options(-O2)
endif()

if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    add_compile_options(-g -O0 -fsanitize=address,undefined)
    add_link_options(-fsanitize=address,undefined)
endif()

find_package(Threads REQUIRED)

add_library(cansimulator_lib STATIC
    src/Logger.cpp
    src/Metrics.cpp
    src/PIDHandler.cpp
    src/CANSocket.cpp
    src/Simulator.cpp
    src/OBDServer.cpp
    src/OBDClient.cpp
)
target_include_directories(cansimulator_lib PUBLIC include)
target_link_libraries(cansimulator_lib PUBLIC Threads::Threads)

add_executable(cansimulator src/main.cpp)
target_include_directories(cansimulator PRIVATE include)
target_link_libraries(cansimulator PRIVATE cansimulator_lib)

find_package(PkgConfig QUIET)
if(PkgConfig_FOUND)
    pkg_check_modules(SDL2     QUIET sdl2)
    pkg_check_modules(SDL2_TTF QUIET SDL2_ttf)
endif()

if(SDL2_FOUND AND SDL2_TTF_FOUND)
    message(STATUS "SDL2 found -- dashboard enabled (--dashboard)")
    target_sources(cansimulator PRIVATE src/Dashboard.cpp)
    target_include_directories(cansimulator PRIVATE ${SDL2_INCLUDE_DIRS} ${SDL2_TTF_INCLUDE_DIRS})
    target_link_libraries(cansimulator PRIVATE ${SDL2_LIBRARIES} ${SDL2_TTF_LIBRARIES})
    target_compile_definitions(cansimulator PRIVATE DASHBOARD_ENABLED)
else()
    message(STATUS "SDL2/SDL2_ttf not found -- dashboard disabled")
    message(STATUS "  install: sudo apt install libsdl2-dev libsdl2-ttf-dev")
endif()

option(BUILD_TESTS "Build GoogleTest unit tests" ON)

if(BUILD_TESTS)
    enable_testing()

    include(FetchContent)
    FetchContent_Declare(
        googletest
        URL      https://github.com/google/googletest/archive/refs/tags/v1.14.0.zip
        URL_HASH MD5=b4911e882c51cba34bebfb5df500a650
    )
    set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(googletest)

    add_executable(cansimulator_tests tests/TestPID.cpp)
    target_include_directories(cansimulator_tests PRIVATE include)
    target_link_libraries(cansimulator_tests PRIVATE cansimulator_lib GTest::gtest_main)

    include(GoogleTest)
    gtest_discover_tests(cansimulator_tests)
endif()

install(TARGETS cansimulator DESTINATION bin)
