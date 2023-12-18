cmake_minimum_required(VERSION 3.10)

set(PROJECT_VERSION "0.0.1")
project(GraphPilot
        VERSION ${PROJECT_VERSION}
        LANGUAGES CXX)

if ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GUN" OR
    "${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
    set(warnings "-Wall -Wextra -Werror")
elseif ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "MSVC")
    set(warnings "/W4 /WX /EHsc")
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
set(CMAKE_CXX_STANDARD 17)

# Set build type if not set
set(default_build_type "Release")
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE ${default_build_type})
endif()

find_package(Boost 1.70 REQUIRED COMPONENTS graph)
include_directories(${Boost_INCLUDE_DIR})
link_directories(${Boost_LIBRARY_DIR})

# include_directories("${CMAKE_SOURCE_DIR}/include")

# set(SOURCE_FILES
#     ${CMAKE_CURRENT_SOURCE_DIR}/src/Source.cpp
# )

add_executable(dijkstra_demo ${CMAKE_CURRENT_SOURCE_DIR}/src/dijkstra.cpp)
target_link_libraries(dijkstra_demo ${BOOST_LIBRARIES})

add_executable(dfs_demo ${CMAKE_CURRENT_SOURCE_DIR}/src/dfs.cpp)
target_link_libraries(dfs_demo ${BOOST_LIBRARIES})
