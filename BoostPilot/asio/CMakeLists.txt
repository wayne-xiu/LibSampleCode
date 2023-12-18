cmake_minimum_required(VERSION 3.10)

set(PROJECT_VERSION 0.0.1)

project(AsioPilot VERSION ${PROJECT_VERSION})

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(default_build_type "Release")

find_package(Boost 1.70 REQUIRED COMPONENTS system)
include_directories(${Boost_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIRS})


add_executable(SimpleServer src/simple_server.cpp)
target_link_libraries(SimpleServer ${Boost_LIBRARIES} pthread)

add_executable(SimpleClient src/simple_client.cpp)
target_link_libraries(SimpleClient ${Boost_LIBRARIES} pthread)