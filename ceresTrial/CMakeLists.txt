cmake_minimum_required(VERSION 3.2)

project(ceresTrial LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_BUILD_TYPE "Release")

# ceres
find_package(Ceres REQUIRED)
include_directories(${CERES_INCLUDE_DIRS})

# Eigen
include_directories("/usr/include/eigen3")

add_executable(ceresHelloWorld helloWorld.cpp)
target_link_libraries(ceresHelloWorld ${CERES_LIBRARIES})

add_executable(powell powell.cpp)
target_link_libraries(powell ${CERES_LIBRARIES})

add_executable(curveFitting curveFitting.cpp)
target_link_libraries(curveFitting ${CERES_LIBRARIES})
