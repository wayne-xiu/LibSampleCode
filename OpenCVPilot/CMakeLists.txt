cmake_minimum_required(VERSION 3.10)

set(PROJECT_VERSION "0.0.1")
project(OpenCVPilot
        VERSION ${PROJECT_VERSION}
        LANGUAGES CXX)

if ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GUN" OR
    "${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
    set(warnings "-Wall -Wextra -Werror")
elseif ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "MSVC")
    set(warnings "/W4 /WX /EHsc")
endif()

find_package(OpenCV REQUIRED)

add_subdirectory(Intro)
add_subdirectory(CoreFunc)
add_subdirectory(ImageProcessing)
add_subdirectory(2DFeatureFramework)


set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
set(CMAKE_CXX_STANDARD 17)
# Set build type if not set
set(default_build_type "Release")
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE ${default_build_type})
endif()

include_directories(${OpenCV_INCLUDE_DIRS})