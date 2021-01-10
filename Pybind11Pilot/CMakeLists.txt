cmake_minimum_required(VERSION 3.15...3.18)
project(Pybind11Pilot LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Method 1: add_subdirectory with pybind11 repository internally
add_subdirectory(pybind11)

# Method 2:find_package to detect external installation
# find_package(pybind11 REQUIRED)

# Method 3: CMake 3.12+ (3.15+ recommended) added a new module called FindPython
# that had a highly improved search algorithm and modern targets and tools.
# If you use FindPython, pybind11 will detect this and use the existing targets instead:
# find_package(Python COMPONENTS Interpreter Development REQUIRED)
# find_package(pybind11 CONFIG REQUIRED)

# pybind11_add_module(<name> [MODULE | SHARED] [EXCLUDE_FROM_ALL]
#                    [NO_EXTRAS] [THIN_LTO] [OPT_SIZE] source1 [source2 ...])
pybind11_add_module(Pybind11Pilot src/Source.cpp)

# Embedding the Python interpreter in C++ executable or library - TODO
# target_link_libraries(Pybind11Pilot PRIVATE pybind11::embed)
