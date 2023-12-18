cmake_minimum_required(VERSION 3.10)

set(PROJECT_VERSION "0.0.1")
project(BoostPilot
        VERSION ${PROJECT_VERSION}
        LANGUAGES CXX)

# Modules
add_subdirectory(installation)

add_subdirectory(geometry)

add_subdirectory(filesystem)

add_subdirectory(test)

add_subdirectory(multiindex)

add_subdirectory(graph)

add_subdirectory(asio)

add_subdirectory(property_tree)
