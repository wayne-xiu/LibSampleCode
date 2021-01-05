cmake_minimum_required(VERSION 3.5)

set(PROJECT_VERSION "0.0.1")
project(BoostPilot
        VERSION ${PROJECT_VERSION}
        LANGUAGES CXX)

if ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GUN" OR
    "${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
    set(warnings "-Wall -Wextra -Werror")
elseif ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "MSVC")
    set(warnings "/W4 /WX /EHsc")
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
set(CMAKE_CXX_STANDARD 11)
# Set build type if not set
set(default_build_type "Release")
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE ${default_build_type})
endif()

# if(${UNIX})
#      set(CMAKE_PREFIX_PATH "/home/boost_1.70.0/boost")
# else()
# 	set(CMAKE_PREFIX_PATH "C:/boost_1.70.0/boost")
# endif()
FIND_PACKAGE (Boost REQUIRED)
INCLUDE_DIRECTORIES (
   ${Boost_INCLUDE_DIRS}
)

include_directories("${CMAKE_SOURCE_DIR}/include")

set(TEST_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/src/Source.cpp
)

add_executable(BoostTest ${TEST_SOURCES})