## Third Party Libraries

if (NOT TARGET cppcoro::cppcoro)
    find_package(cppcoro REQUIRED CONFIG)
    set_target_properties(cppcoro::cppcoro PROPERTIES IMPORTED_GLOBAL ON)
endif (NOT TARGET cppcoro::cppcoro)

if (NOT TARGET range-v3)
    find_package(range-v3 REQUIRED CONFIG)
    set_target_properties(range-v3 PROPERTIES IMPORTED_GLOBAL ON)
endif (NOT TARGET range-v3)

# Use: Formatting of strings (Will be the implementation for string formatting in C++20)
if (NOT TARGET fmt::fmt)
    find_package(fmt REQUIRED CONFIG)
    set_target_properties(fmt::fmt PROPERTIES IMPORTED_GLOBAL ON)
endif (NOT TARGET fmt::fmt)


# Use:Logger
if (NOT TARGET spdlog::spdlog)
    find_package(spdlog REQUIRED CONFIG)
    set_target_properties(spdlog::spdlog
            PROPERTIES
            IMPORTED_GLOBAL ON
            POSITION_INDEPENDENT_CODE ON)
endif (NOT TARGET spdlog::spdlog)

# Use: json and a "any" alternative
if (NOT TARGET nlohmann_json::nlohmann_json)
    find_package(nlohmann_json 3.10.2 REQUIRED CONFIG)
    set_target_properties(nlohmann_json::nlohmann_json PROPERTIES IMPORTED_GLOBAL ON)
endif (NOT TARGET nlohmann_json::nlohmann_json)

# Use yaml parsing
if (NOT TARGET yaml-cpp)
    find_package(yaml-cpp REQUIRED CONFIG)
    set_target_properties(yaml-cpp PROPERTIES IMPORTED_GLOBAL ON)
endif (NOT TARGET yaml-cpp)

# Use: Mixed-Integer Linear Programming Solver
if (NOT TARGET gurobi)
    find_package(gurobi REQUIRED)
endif (NOT TARGET gurobi)

# Use: Matrix algebra library
if (NOT TARGET Eigen3)
    # Install via apt
    find_package(Eigen3 3.3 REQUIRED CONFIG)
endif (NOT TARGET Eigen3)

# Use: Motion Planning library
if (NOT TARGET ompl)
    # Build and install
    find_package(ompl 1.5.2 REQUIRED CONFIG)
endif (NOT TARGET ompl)

# Use: Machine Learning library
if (NOT TARGET torch)
    set(CMAKE_CUDA_COMPILER /usr/local/cuda-12.3/bin/nvcc)
    find_package(CUDAToolkit 12.3 REQUIRED)
    #set(Torch_DIR /opt/libtorch/share/cmake/Torch)
    set(Torch_DIR /usr/local/lib/python3.10/dist-packages/torch/share/cmake/Torch)
    find_package(Torch REQUIRED CONFIG)
    set_target_properties(torch PROPERTIES IMPORTED_GLOBAL ON)
    set_target_properties(torch_cpu PROPERTIES IMPORTED_GLOBAL ON)
    set_target_properties(torch_cuda PROPERTIES IMPORTED_GLOBAL ON)
    #    set_target_properties(torch_cuda_cpp PROPERTIES IMPORTED_GLOBAL ON)
    #    set_target_properties(torch_cuda_cu PROPERTIES IMPORTED_GLOBAL ON)
endif (NOT TARGET torch)

# Use: fast maps and sets
if (NOT TARGET robin_hood)
    add_subdirectory(robin_hood)
endif (NOT TARGET robin_hood)

# Use: enum reflection
if (NOT TARGET magic_enum)
    add_subdirectory(magic_enum)
endif (NOT TARGET magic_enum)

# Use: command line arguments
if (NOT TARGET cli11)
    add_subdirectory(cli11)
endif (NOT TARGET cli11)

# Use: tag invoke
if (NOT TARGET duck_invoke)
    add_subdirectory(duck_invoke)
endif (NOT TARGET duck_invoke)

list(APPEND LCOV_REMOVE_PATTERNS "'${CMAKE_CURRENT_SOURCE_DIR}/*' ")