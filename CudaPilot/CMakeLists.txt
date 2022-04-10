cmake_minimum_required(VERSION 3.13)

set(PROJECT_VERSION "0.0.1")
# set(PROJECT_NAME "CMakePilot")
project(HelloCUDA
        VERSION ${PROJECT_VERSION}
        LANGUAGES CXX CUDA)

if ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GUN" OR
    "${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
    set(warnings "-Wall -Wextra -Werror")
elseif ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "MSVC")
    set(warnings "/W4 /WX /EHsc")
endif()

set(CMAKE_CXX_STANDARD 17)

# Set build type if not set
set(default_build_type "Release")
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE ${default_build_type})
endif()


set(SOURCE_FILES ${CMAKE_CURRENT_SOURCE_DIR}/src/hello.cu)

add_executable(${PROJECT_NAME} ${SOURCE_FILES})
