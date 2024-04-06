cmake_minimum_required(VERSION 3.16)
project(grstapse
        VERSION 4.0.1
        DESCRIPTION "Graphically Recursive Simultaneous Task Allocation, Planning, Scheduling, and Execution"
        LANGUAGES CXX)
message("Building ${PROJECT_NAME}...")

# Versioning guide
# version major increases with each publication
# version minor increases with a new feature
# version patch increases with smaller changes that need to be distinguished

# Store the language
get_property(LANGUAGES GLOBAL PROPERTY ENABLED_LANGUAGES)

set(DEFAULT_BUILD_TYPE "Release")
if (NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    message(STATUS "Setting build type to '${DEFAULT_BUILD_TYPE}' as none was specified.")
    set(CMAKE_BUILD_TYPE "${DEFAULT_BUILD_TYPE}" CACHE
            STRING "Choose the type of build." FORCE)
    # Set the possible values of build type for cmake-gui
    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS
            "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif (NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)

include(GNUInstallDirs)
list(APPEND CMAKE_PREFIX_PATH "/usr/local")

# Add external cmake modules
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

# Determine if this project is built as a subproject (using add_subdirectory)
# or if it is the master project.
set(MASTER_PROJECT OFF)
if (CMAKE_CURRENT_SOURCE_DIR STREQUAL CMAKE_SOURCE_DIR)
    set(MASTER_PROJECT ON)
    message(STATUS "CMake version: ${CMAKE_VERSION}")
endif (CMAKE_CURRENT_SOURCE_DIR STREQUAL CMAKE_SOURCE_DIR)

# Options
include(CMakeDependentOption)
#cmake_policy(SET CMP0127 NEW)
option(BUILD_DOCUMENTATION "Whether to build the documentation (Note: gets built in the docs folder in the source)" ${MASTER_PROJECT})
string(TOUPPER "${CMAKE_BUILD_TYPE}" CMAKE_BUILD_TYPE_UPPER)
cmake_dependent_option(BUILD_UNITTESTS "Whether to build the unit tests (Note: this only works for DEBUG builds" ON "${MASTER_PROJECT} AND '${CMAKE_BUILD_TYPE_UPPER}' STREQUAL 'DEBUG'" OFF)
cmake_dependent_option(BUILD_COVERAGE "Whether to build with coverage output (Note: this only works for debug builds)" OFF "'${CMAKE_BUILD_TYPE_UPPER}' STREQUAL 'DEBUG'" OFF)
cmake_dependent_option(USE_PROFILING "Whether to set the optimization level for profiling (Note: this only works for debug builds)" OFF "${MASTER_PROJECT} AND '${CMAKE_BUILD_TYPE_UPPER}' STREQUAL 'DEBUG'" OFF)
unset(CMAKE_BUILD_TYPE_UPPER)
option(BUILD_EXECUTABLES "Whether to build the executables in the executable folder" ON)
option(BUILD_PYTHON_WRAPPER "Whether to build the python wrapper" OFF)

# Basically string options
set(DATA_DIR "${CMAKE_CURRENT_SOURCE_DIR}/data" CACHE PATH "Directory to the data folder")
set(PYTHON_EMBED_SCRIPTS_DIR "${CMAKE_CURRENT_SOURCE_DIR}/python/embed" CACHE PATH "Directory to the embed python folder")

# include cmake modules
include(CodeCoverage)


find_package(pybind11 REQUIRED CONFIG)

# Add external libraries
add_subdirectory(extern)
find_package(OpenMP)

find_package(concurrencpp REQUIRED)

# Add pthreads
set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

# Git hash cannot be retrieved in the docker - would be nice to have in the config.hpp, so it can be printed with executables
# Git Hash
#if (MASTER_PROJECT)
#    add_custom_target(check_git COMMAND ${CMAKE_COMMAND}
#            -P ${PROJECT_SOURCE_DIR}/cmake/CheckGit.cmake
#            DEPENDENCIES ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/config.hpp.in
#            BYPRODUCT ${PROJECT_BINARY_DIR}/include/${PROJECT_NAME}/config.hpp)
#endif (MASTER_PROJECT)

# Place to fix an issue with fmt and pybind11
# See: https://en.wikipedia.org/wiki/Position-independent_code
# See: https://github.com/fmtlib/fmt/issues/2456
# See: https://github.com/fmtlib/fmt/issues/548
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

set(CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 20)

# Collect header and source files
add_subdirectory(include)
add_subdirectory(src)

find_package(
        Python COMPONENTS Interpreter Development REQUIRED
)

# Create library
add_library(_${PROJECT_NAME} STATIC)
target_sources(_${PROJECT_NAME}
        PUBLIC
        ${HEADERS}
        PRIVATE
        ${SOURCES})
target_precompile_headers(_${PROJECT_NAME}
        PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include/${PROJECT_NAME}/config.hpp>
        $<INSTALL_INTERFACE:include/${PROJECT_NAME}/config.hpp>)
target_compile_features(_${PROJECT_NAME}
        PRIVATE
        cxx_std_20)
target_link_libraries(_${PROJECT_NAME}
        PUBLIC
        cli11
        cppcoro::cppcoro
        duck_invoke
        fmt::fmt
        gurobi
        magic_enum
        nlohmann_json::nlohmann_json
        ompl
        OpenMP::OpenMP_CXX
        robin_hood
        spdlog::spdlog
        torch
        yaml-cpp
        concurrencpp::concurrencpp
)
target_link_libraries(_${PROJECT_NAME}
        PUBLIC
        pybind11::pybind11
        pybind11::embed
        Python::Python
        Python::Module
)
target_include_directories(_${PROJECT_NAME}
        PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>
        $<INSTALL_INTERFACE:include>)
target_compile_options(_${PROJECT_NAME}
        PUBLIC
        "-fcoroutines"
        "-fvisibility=hidden"
        "-Werror=return-type"
        "$<$<AND:$<CONFIG:DEBUG>,$<BOOL:${USE_PROFILING}>>:-O0>"
        "$<$<CONFIG:RELEASE>:-Ofast>")
target_compile_definitions(_${PROJECT_NAME}
        PUBLIC
        "$<$<CONFIG:DEBUG>:DEBUG>")
if (BUILD_COVERAGE)
    target_code_coverage(_${PROJECT_NAME})
endif (BUILD_COVERAGE)
# Git hash cannot be retrieved in the docker - would be nice to have in the config.hpp, so it can be printed with executables
#if (MASTER_PROJECT)
#    add_dependencies(_${PROJECT_NAME} check_git)
#endif (MASTER_PROJECT)

# Build documentation
if (BUILD_DOCUMENTATION)
    add_subdirectory(docs)
endif (BUILD_DOCUMENTATION)

# Build unit tests
if (BUILD_UNITTESTS)
    add_subdirectory(tests)
endif (BUILD_UNITTESTS)

# Build executables
if (BUILD_EXECUTABLES)
    add_subdirectory(executables)
endif (BUILD_EXECUTABLES)

# Build the python wrappers/interfaces
if (BUILD_PYTHON_WRAPPER)
    add_subdirectory(python)
endif (BUILD_PYTHON_WRAPPER)