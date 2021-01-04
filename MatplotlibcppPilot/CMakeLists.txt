cmake_minimum_required(VERSION 3.2)

project(matplotlibCpp LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Python3 COMPONENTS Development NumPy)
target_include_directories(matplotlibCpp PRIVATE ${Python3_INCLUDE_DIRS} ${Python3_NumPy_INCLUDE_DIRS})
#target_link_libraries(matplotlibCpp Python3::Python Python3::NumPy)

add_executable(matplotlibCpp main.cpp)
